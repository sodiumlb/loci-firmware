cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/src/pico-sdk)
set(PICO_EXTRAS_PATH ${CMAKE_CURRENT_LIST_DIR}/src/pico-extras)
# set(PICO_TINYUSB_PATH ${CMAKE_CURRENT_LIST_DIR}/src/tinyusb)

include(${PICO_SDK_PATH}/pico_sdk_init.cmake)
add_subdirectory(${PICO_EXTRAS_PATH})

find_package(Git)

if(GIT_EXECUTABLE)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} describe --tags
    WORKING_DIRECTORY ..
    OUTPUT_VARIABLE LOCIFW_VERSION
    RESULT_VARIABLE ERROR_CODE
    OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()

if(LOCIFW_VERSION STREQUAL "")
  set(LOCIFW_VERSION 0.0.0)
  message(WARNING "Failed to determine version from Git tags. Using default version \"${LOCIFW_VERSION}\".")
else()
  # Remove leading 'v'
  string(REGEX REPLACE "^v" "" LOCIFW_VERSION "${LOCIFW_VERSION}")
  string(REGEX REPLACE "\-.*$" "" LOCIFW_VERSION "${LOCIFW_VERSION}")
  message("" "-- Found version from git: \"${LOCIFW_VERSION}\".")
endif()

project(LOCIFW VERSION ${LOCIFW_VERSION} LANGUAGES C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_subdirectory(src)
