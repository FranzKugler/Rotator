#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

void expert_lock_init(void);
esp_err_t expert_lock_register_routes(httpd_handle_t server);
bool expert_lock_guard(httpd_req_t *req);

#ifdef __cplusplus
}
#endif
