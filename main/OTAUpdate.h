#pragma once
#include "esp_http_server.h"

esp_err_t register_ota_update_uri(httpd_handle_t server);
