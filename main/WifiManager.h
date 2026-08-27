// WifiManager.h
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

void wifi_manager_init();
bool wifi_manager_is_connected();
const char *wifi_manager_get_ip();

#ifdef __cplusplus
}
#endif
