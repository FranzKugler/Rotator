cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Stamp tagged builds with the same git-describe scheme as QlockThreeW32.
# The fallback in main/Version.h is used before the first release tag.
if(DEFINED ENV{ROTATOR_BUILD_VERSION_OVERRIDE} AND NOT "$ENV{ROTATOR_BUILD_VERSION_OVERRIDE}" STREQUAL "")
    set(ROTATOR_BUILD_VERSION "$ENV{ROTATOR_BUILD_VERSION_OVERRIDE}")
else()
    execute_process(
        COMMAND git describe --tags --dirty --match v[0-9]*
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE ROTATOR_GIT_VERSION
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        RESULT_VARIABLE ROTATOR_GIT_RESULT
    )
endif()
if(NOT ROTATOR_BUILD_VERSION AND ROTATOR_GIT_RESULT EQUAL 0 AND ROTATOR_GIT_VERSION MATCHES "^v?[0-9]")
    string(REGEX REPLACE "^v" "" ROTATOR_BUILD_VERSION "${ROTATOR_GIT_VERSION}")
elseif(NOT ROTATOR_BUILD_VERSION)
    set(ROTATOR_BUILD_VERSION "0.8.0")
endif()
add_compile_definitions(ROTATOR_VERSION="${ROTATOR_BUILD_VERSION}")
set(PROJECT_VER "${ROTATOR_BUILD_VERSION}")
file(WRITE "${CMAKE_BINARY_DIR}/version.txt" "${ROTATOR_BUILD_VERSION}\n")
message(STATUS "Rotator version: ${ROTATOR_BUILD_VERSION}")

# make sure we get the tinyusb headers with higher priority than the ones coming from Arduino
include_directories(BEFORE
    $ENV{IDF_PATH}/managed_components/esp_tinyusb/include
    $ENV{IDF_PATH}/managed_components/tinyusb/src
)

# get rid of compiler issues
set_source_files_properties($ENV{IDF_PATH}/components/esp_lcd/rgb/esp_lcd_panel_rgb.c PROPERTIES COMPILE_FLAGS "-O0")

# partition table layout
set(PROJECT_PARTITIONS "partitions.csv")

project(Rotator)
