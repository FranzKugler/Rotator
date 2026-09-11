file(GLOB_RECURSE app_sources_cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
)

file(GLOB_RECURSE app_sources_c 
    ${CMAKE_CURRENT_SOURCE_DIR}/*.c
)

file(GLOB_RECURSE alpaca_sources_cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/alpaca_server/*.cpp
)

file(GLOB_RECURSE alpaca_sources_c 
    ${CMAKE_CURRENT_SOURCE_DIR}/alpaca_server/*.c
)

idf_component_register(
    SRCS 
        ${app_sources_cpp} 
        ${app_sources_c}
        ${alpaca_sources_cpp} 
        ${alpaca_sources_c}
    INCLUDE_DIRS ""
    REQUIRES
        espressif__arduino-esp32
        nvs_flash
        esp_netif 
        esp_event
        esp_wifi
        app_update
        tinyusb
        esp_tinyusb 
        esp_http_client
        esp_http_server
        esp_https_ota
        esp-tls
        mdns
        json
        TMC2209
        FastAccelStepper
        AS5600
        ftp_server
)

# Stamps ROTATOR_VERSION fresh on every build via scripts/version.py - see
# that script's own comment and Version.h for why a one-shot CMake-configure-
# time `execute_process(git describe...)` (what this used to be, at the top
# level) is not enough. `rotator_version` has no OUTPUT of its own, so being
# in the ALL target set makes it rerun every single build; the script itself
# only rewrites the generated header when its content actually changed.
idf_build_get_property(python PYTHON)
set(rotator_version_header "${CMAKE_CURRENT_BINARY_DIR}/rotator_version_generated.h")
add_custom_target(rotator_version ALL
    COMMAND ${python} "${CMAKE_SOURCE_DIR}/scripts/version.py"
            --source-dir "${CMAKE_SOURCE_DIR}"
            --fallback-header "${CMAKE_CURRENT_SOURCE_DIR}/Version.h"
            --generated-header "${rotator_version_header}"
            --stamp-file "${CMAKE_BINARY_DIR}/version.txt"
    BYPRODUCTS "${rotator_version_header}"
    COMMENT "Stamping Rotator version"
    VERBATIM
)
add_dependencies(${COMPONENT_LIB} rotator_version)
target_include_directories(${COMPONENT_LIB} PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
target_compile_options(${COMPONENT_LIB} PRIVATE "-include" "${rotator_version_header}")

littlefs_create_partition_image(littlefs data)

