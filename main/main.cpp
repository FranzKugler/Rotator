#include <Arduino.h>
#include <string.h>
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_types.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_littlefs.h"
#include "esp_wifi.h"
#include "tinyusb.h"
#include "tinyusb_net.h"
#include "tusb.h"
#include "tusb_cdc_acm.h"
#include "tusb_console.h"
#include "esp_mac.h"
#include "esp_http_server.h"
#include "lwip/ip4_addr.h"
#include "ftp.h"
#include "usb_descconfig.h"
#include "OTAUpdate.h"
}
#include <dirent.h>
#include <stdio.h>

#include "Configuration.hpp"
#include "WebServer.h"
#include "WifiManager.h"
#include "RotatorApi.h"
#include "RotatorHW.h"

static const char *TAG = "main";
//QueueHandle_t angleUpdateQueue = nullptr;


// the port of the Alpaca Server
#define ALPACA_SERVER_PORT 80

// ----------------------------------------------------------------
// ------ Helper functions to set up the ASCOM Alpaca device ------
// ----------------------------------------------------------------

RotatorApi &get_rotator_api()
{
    static RotatorApi rotatorApi(RotatorHW::getInstance());
    return rotatorApi;
}

std::vector<AlpacaServer::Device *> &get_devices()
{
    static std::vector<AlpacaServer::Device *> devices{&get_rotator_api()};
    return devices;
}

AlpacaServer::Api &get_api()
{
    static AlpacaServer::Api api(
        get_devices(),
        "Rotator",
        "Astro Geeks Alpaca Server",
        "Astro Geeks",
        ROTATOR_VERSION,
        "Munich");
    return api;
}

extern "C" void app_main(void)
{

    initArduino();

    // 1) NVS required by WiFi/Net stack
    ESP_LOGI(TAG, "Initializing NVS...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // also read in configuration from flash disk
    auto &cfg = Configuration::getInstance();

    // 2) NetIF
    ESP_LOGI(TAG, "Initializing netif...");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 3) TinyUSB stack
    ESP_LOGI(TAG, "Installing TinyUSB driver...");
    tinyusb_config_t tusb_cfg = {
        .device_descriptor = &descriptor_device,
        .string_descriptor = descriptor_strings,
        .external_phy = false,
        .configuration_descriptor = descriptor_configuration};
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    // 4) CDC for logging
    ESP_LOGI(TAG, "Initializing CDC ACM...");
    tinyusb_config_cdcacm_t amc_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 128,
    };
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&amc_cfg));
    esp_tusb_init_console(TINYUSB_CDC_ACM_0);

    // 5) RNDIS network interface
    // Set IP from config file
    setup_usb_rndis_ip(cfg.getIPAddressInt(), cfg.getIPAddressInt(), cfg.getNetmaskInt());
    // Set mac address form config file
    setup_usb_rndis_mac(cfg.getMACAddress().data());
    // the RNDIS netif is brought up here
    setup_usb_rndis_netif();
    ESP_LOGI(TAG, "TinyUSB CDC and RNDIS initialized!");

    // 6) Wifi init
    wifi_manager_init();

    // 7) HTTP server
    httpd_config_t http_cfg = HTTPD_DEFAULT_CONFIG();
    http_cfg.server_port = ALPACA_SERVER_PORT;
    http_cfg.uri_match_fn = httpd_uri_match_wildcard;
    http_cfg.max_uri_handlers = 64;
    http_cfg.stack_size = 16384;
    httpd_handle_t server = nullptr;
    ESP_ERROR_CHECK(httpd_start(&server, &http_cfg));

    // 8) Over the Air Update registration
    register_ota_update_uri(server);

    // 9) Device and Alpaca Server
    RotatorHW::getInstance().begin();
    get_api().register_routes(server);

    // 10) Start tasks that update the GUI automatically
    xTaskCreate(angle_producer_task,  "angProd", 4096, NULL, 2, NULL);
    xTaskCreate(angle_event_broadcast, "wsBrdc", 4096, NULL, 2, NULL);

    // 11) Register all webserver handles
    register_web_handles(server);

    // 12) Alpaca discovery daemon
    alpaca_server_discovery_start(ALPACA_SERVER_PORT);

    // 13) Start ftp server
    xTaskCreate(ftp_task, "FTP", 1024 * 6, NULL, 2, NULL);

    // 14) Move to mechanical zero
    RotatorHW::getInstance().gotoMechanicalZero();


    while (1)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        //angles.angle = RotatorHW::getInstance().getPosition();
        //angles.angle = fmod(angles.angle + 1.0, 360.0);
        //angles.mechAngle = 10.0;
        //xQueueOverwrite(angleUpdateQueue, &angles);
    }
}
