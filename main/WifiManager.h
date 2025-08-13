// WifiManager.h
#pragma once
#include "esp_err.h"
void wifi_manager_init();  // Mount NVS, init WiFi und non-blocking connect
bool wifi_manager_is_connected();
const char* wifi_manager_get_ip();