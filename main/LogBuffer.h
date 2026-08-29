#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

// How much is kept, and what it costs: 200 x 128 = 25 KB in .bss. The one
// number to turn down if RAM ever gets tight.
#define LOG_BUFFER_LINES 200
#define LOG_BUFFER_LINE_MAX 128

// How many lines one /log response carries at most. The browser asks again
// straight away while "more" is set, so a freshly opened tab still fills in
// one go instead of a batch every two seconds.
#define LOG_BUFFER_BATCH 100

/**
 * Starts capturing. Call as the first thing in app_main(), before anything
 * has a chance to log - every ESP_LOGx call and the WiFi/USB driver's own
 * ESP-IDF logging both go through esp_log_set_vprintf(), so one hook here
 * catches the lot.
 */
void log_buffer_init(void);

/** Registers GET /log, gated by expert_lock_guard() like the OTA routes. */
esp_err_t log_buffer_register_routes(httpd_handle_t server);

#ifdef __cplusplus
}
#endif
