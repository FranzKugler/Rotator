// WifiManager.h
#pragma once

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void wifi_manager_init();
bool wifi_manager_is_connected();
const char *wifi_manager_get_ip();
const char *wifi_manager_get_ssid();
const char *wifi_manager_get_hostname();
int wifi_manager_get_rssi();
esp_err_t wifi_manager_get_mac(uint8_t mac[6]);
esp_err_t wifi_manager_set_hostname(const char *hostname, char *stored, size_t stored_size);

#ifdef __cplusplus
}
#endif
