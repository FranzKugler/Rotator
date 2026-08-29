#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

// Most entries one listing answers with. NVS holds a few dozen on this
// rotator; this is a guard against a partition somebody filled, not a real
// limit.
#define NVS_LIST_MAX 128

// Largest value the browser is offered for editing, and the largest it may
// write back. Strings in NVS are capped at 4000 bytes by the store itself,
// so this only has to stay under what the heap will hold twice over.
#define NVS_EDIT_MAX 4096

// A string longer than this is listed as `.bin` without being read, so that
// building the tree never pulls a large blob into the heap just to look at
// its first character.
#define NVS_PEEK_MAX 2048

/** Registers the nvs routes (/nvs/list, /nvs/read, ...), gated behind the expert lock. */
esp_err_t nvs_routes_register(httpd_handle_t server);

#ifdef __cplusplus
}
#endif
