cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

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
