# TinyUSB detatched from Pico SDK so we can use latest

file(GLOB_RECURSE TINY_USB_SRC_FILES ${CMAKE_CURRENT_LIST_DIR}/tinyusb/src/*.c)

add_library(tinyusb_mia STATIC EXCLUDE_FROM_ALL)
target_include_directories(tinyusb_mia PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/tinyusb/src
    mia
)
target_link_libraries(tinyusb_mia PUBLIC
    pico_stdlib
)
target_sources(tinyusb_mia PRIVATE
    ${TINY_USB_SRC_FILES}
)

string(APPEND CMAKE_EXE_LINKER_FLAGS "-Wl,--print-memory-usage")
# Creates C resources file from files in given directory
function(create_resources dir output)
    message(STATUS "Create Resource ${dir} ${output}")
    # Create empty output file
    file(WRITE ${output} "")
    # Collect input files
    file(GLOB bins ${dir}/*)
    # Iterate through input files
    foreach(bin ${bins})
        message(STATUS "Adding ${bin}")
        # Get short filename
        string(REGEX MATCH "([^/]+)$" filename ${bin})
        # Replace filename spaces & extension separator for C compatibility
        string(REGEX REPLACE "\\.| |-" "_" filename ${filename})
        # Read hex data from file
        file(READ ${bin} filedata HEX)
        # Convert hex data for C compatibility
        string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," filedata ${filedata})
        # Prepare filename part of define
        string(TOUPPER ${filename} filetoken)
        # Create define
        string(CONCAT definetoken "EMBEDDED_" ${filetoken})
        # Append data to output file
        file(APPEND ${output} "#define ${definetoken}\nconst unsigned char __in_flash() ${filename}[] = {${filedata}};\nconst unsigned ${filename}_size = sizeof(${filename});\n")
    endforeach()
endfunction()

create_resources(${CMAKE_CURRENT_LIST_DIR}/roms ${CMAKE_BINARY_DIR}/loci_roms.h)

# The Oric LOCI MIA (media interface adapter)

add_executable(loci-firmware)
pico_add_extra_outputs(loci-firmware)
pico_set_program_name(loci-firmware "LOCI firmware")
pico_set_binary_type(loci-firmware copy_to_ram)
pico_set_linker_script(loci-firmware ${CMAKE_CURRENT_LIST_DIR}/memmap_xram_mia.ld)

target_include_directories(loci-firmware PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/tinyusb/src
    mia
    ${PROJECT_BINARY_DIR}
)

target_compile_definitions(loci-firmware PRIVATE
    LFS_NO_MALLOC=1
    LFS_NAME_MAX=64
    #LFS_YES_TRACE=1
    PICO_FLASH_SIZE_BYTES=16777216
    PICO_USE_STACK_GUARDS=1
    PICO_DEBUG_MALLOC=1
    PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64
    PICO_CLOCK_AJDUST_PERI_CLOCK_WITH_SYS_CLOCK=1
)

target_compile_options(loci-firmware PRIVATE
  -Wall -Wextra -Os
)

target_link_libraries(loci-firmware PRIVATE
    pico_stdlib
    pico_multicore
    pico_rand
    hardware_pio
    hardware_dma
    hardware_pwm
    hardware_flash
    hardware_rtc
    hardware_i2c
    tinyusb_mia

)

pico_generate_pio_header(loci-firmware
    ${CMAKE_CURRENT_LIST_DIR}/mia/sys/mia.pio
)

target_sources(loci-firmware PRIVATE
    mia/main.c
    mia/str.c
    mia/api/api.c
    mia/api/clk.c
    mia/api/dir.c
    mia/api/mnt.c
    mia/api/oem.c
    mia/api/rng.c
    mia/api/std.c
 #   mia/aud/aud.c
    mia/mon/fil.c
    mia/mon/hlp.c
    mia/mon/mon.c
    mia/mon/ram.c
    mia/mon/rom.c
    mia/mon/set.c
    mia/mon/vip.c
    mia/sys/adj.c
    mia/sys/cfg.c
    mia/sys/com.c
    mia/sys/cpu.c
    mia/sys/ext.c
    mia/sys/led.c
    mia/sys/lfs.c
    mia/sys/mem.c
    mia/sys/mia.c
    mia/sys/pix.c
    mia/sys/pwr.c
    mia/sys/sys.c
    mia/sys/ssd.c
#    mia/sys/vga.c
    mia/usb/cdc.c
    mia/usb/hid_ps4.c
    mia/usb/hid.c
    mia/usb/kbd.c
    mia/usb/mou.c
    mia/usb/msc.c
    mia/usb/usb.c
    mia/oric/acia.c
    mia/oric/map.c
    mia/oric/dsk.c
    mia/oric/rom.c
    mia/oric/tap.c
    fatfs/ff.c
    fatfs/ffunicode.c
    littlefs/lfs.c
    littlefs/lfs_util.c
)

# Project defines available to both Pi Picos.
# Please change name for hardware forks.
# For release, set version in CMake project call and set code page to 0.

set_property(TARGET loci-firmware
    APPEND PROPERTY COMPILE_DEFINITIONS
    LOCIFW_NAME="Oric LOCI"
    LOCIFW_CODE_PAGE=850
    LOCIFW_KEYBOARD=EN_US
    LOCIFW_MIN_PHI2=500
    LOCIFW_MAX_PHI2=8000
    LOCIFW_EXFAT=1
)
if(DEFINED LOCIFW_VERSION)
    set_property(TARGET loci-firmware
        APPEND PROPERTY COMPILE_DEFINITIONS
        LOCIFW_VERSION=\"${LOCIFW_VERSION}\"
    )
endif()


configure_file(mia/locifw_version.h.in locifw_version.h)

get_target_property(LOCIFW_COMPILE_DEFINITIONS loci-firmware COMPILE_DEFINITIONS)
if(LOCIFW_COMPILE_DEFINITIONS MATCHES "LOCIFW_VERSION=\"\"")
    add_custom_command(TARGET loci-firmware
        COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_LIST_DIR}/mia/sys/sys.c
    )
endif()
