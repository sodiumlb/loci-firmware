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

# The Oric LOCI MIA (media interface adapter)

add_executable(loci_mia)
pico_add_extra_outputs(loci_mia)
pico_set_program_name(loci_mia "LOCI MIA")
pico_set_binary_type(loci_mia copy_to_ram)
pico_set_linker_script(loci_mia ${CMAKE_CURRENT_LIST_DIR}/memmap_xram_mia.ld)

target_include_directories(loci_mia PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/tinyusb/src
    mia
)

target_compile_definitions(loci_mia PRIVATE
    LFS_NO_MALLOC=1
    LFS_NAME_MAX=64
    #LFS_YES_TRACE=1
    PICO_FLASH_SIZE_BYTES=16777216
    PICO_USE_STACK_GUARDS=1
    PICO_DEBUG_MALLOC=1
    PICO_XOSC_STARTUP_DELAY_MULTIPLIER=32
)

target_compile_options(loci_mia PRIVATE
  -Wall -Wextra -Os
)

target_link_libraries(loci_mia PRIVATE
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

pico_generate_pio_header(loci_mia
    ${CMAKE_CURRENT_LIST_DIR}/mia/sys/mia.pio
)

target_sources(loci_mia PRIVATE
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
    mia/sys/cfg.c
    mia/sys/com.c
    mia/sys/cpu.c
    mia/sys/ext.c
    mia/sys/led.c
    mia/sys/lfs.c
    mia/sys/mem.c
    mia/sys/mia.c
#    mia/sys/pix.c
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
    mia/oric/map.c
    mia/oric/dsk.c
    mia/oric/tap.c
    fatfs/ff.c
    fatfs/ffunicode.c
    littlefs/lfs.c
    littlefs/lfs_util.c
)

# Project defines available to both Pi Picos.
# Please change name for hardware forks.
# For release, set version string and set code page to 0.

set_property(TARGET loci_mia
    APPEND PROPERTY COMPILE_DEFINITIONS
    RP6502_NAME="Oric LOCI"
    RP6502_VERSION=""
    RP6502_CODE_PAGE=850
    RP6502_KEYBOARD=EN_US
    RP6502_MIN_PHI2=500
    RP6502_MAX_PHI2=8000
    RP6502_EXFAT=0
)

get_target_property(RP6502_MIA_COMPILE_DEFINITIONS loci_mia COMPILE_DEFINITIONS)
if(RP6502_MIA_COMPILE_DEFINITIONS MATCHES "RP6502_VERSION=\"\"")
    add_custom_command(TARGET loci_mia
        COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_LIST_DIR}/mia/sys/sys.c
    )
endif()
