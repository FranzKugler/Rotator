#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

// Longest path LittleFS is configured for, name included - matches
// CONFIG_LITTLEFS_OBJ_NAME_LEN in sdkconfig. A longer one is refused here
// rather than silently truncated by the driver.
#define FS_PATH_MAX 63

// Most entries one directory listing answers with. The listing is per
// directory and the browser expands lazily, so this is a guard against a
// directory somebody filled up, not a limit on the tree.
#define FS_LIST_MAX 96

// Largest file the in-browser editor will load or save. The editor route
// buffers - it takes JSON, not a stream - so this is a heap limit and has to
// stay well under what the rotator has free. Anything bigger is download and
// upload only, which stream.
#define FS_EDIT_MAX 24576

/** Registers the fs routes (/fs/list, /fs/read, ...), gated behind the expert lock. */
esp_err_t file_routes_register(httpd_handle_t server);

#ifdef __cplusplus
}
#endif
