cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# ESP-IDF's own embedded app-description version (PROJECT_VER - shown by
# `idf.py monitor`/esptool, baked into the esp_app_desc_t at CMake-configure
# time). This is NOT what the firmware reports over HTTP/Alpaca - that
# ROTATOR_VERSION macro is stamped fresh on every single build instead, by
# scripts/version.py via a custom target in main/CMakeLists.txt (see that
# file and main/Version.h). The problem with computing it only here, as this
# project used to: CMake only re-runs this block when it reconfigures, which
# a plain `idf.py build` after committing or tagging does not do - the
# version then quietly goes stale until something forces a reconfigure.
# PROJECT_VER has no build-time equivalent hook in ESP-IDF, so it keeps this
# configure-time-only, best-effort computation.
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
set(PROJECT_VER "${ROTATOR_BUILD_VERSION}")
message(STATUS "Rotator PROJECT_VER (app descriptor, configure-time only): ${ROTATOR_BUILD_VERSION}")

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
