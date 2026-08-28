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

littlefs_create_partition_image(littlefs data)

