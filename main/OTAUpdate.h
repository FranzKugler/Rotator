#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

// Registers the manual-upload and GitHub release-channel update API and starts
// the periodic checker. LittleFS and NVS must already be available.
esp_err_t register_ota_update_uri(httpd_handle_t server);
