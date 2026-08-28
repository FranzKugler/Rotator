#include "OTAUpdate.h"
#include "ExpertLock.h"
#include "Version.h"
#include "WifiManager.h"

#include "cJSON.h"
#include "esp_app_desc.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "mbedtls/sha256.h"
#include "nvs.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define OTA_REPO "FranzKugler/Rotator"
#define OTA_MANIFEST_STABLE "https://github.com/" OTA_REPO "/releases/latest/download/manifest.json"
#define OTA_MANIFEST_EDGE "https://github.com/" OTA_REPO "/releases/download/edge/manifest.json"
#define OTA_NAMESPACE "ota"
#define OTA_REBOOT_DELAY_MS 1500
#define OTA_AUTO_HOUR_FROM 2
#define OTA_AUTO_HOUR_TO 5
#define OTA_HTTP_TIMEOUT_MS 20000
#define OTA_TASK_STACK 12288
#define OTA_MAX_REDIRECTS 5
#define OTA_LOCATION_BUF 1024

static const char *TAG = "OTAUpdate";
extern void ConfigurationSave(void);
extern size_t rotator_get_sketch_size(void);

typedef enum {
    OTA_IDLE,
    OTA_CHECKING,
    OTA_AVAILABLE,
    OTA_DOWNLOADING,
    OTA_FAILED,
    OTA_INSTALLED,
} ota_state_t;

static const char *STATE_NAMES[] = {
    "idle", "checking", "available", "downloading", "failed", "installed"
};

static httpd_handle_t s_server;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile ota_state_t s_state = OTA_IDLE;
static volatile int s_progress;
static char s_error[40];
static char s_error_detail[96];
static char s_fs_version[64];
static char s_available_version[64];
static char s_available_notes[192];
static char s_firmware_url[320];
static char s_firmware_sha[65];
static char s_filesystem_url[320];
static char s_filesystem_sha[65];
static size_t s_firmware_size;
static size_t s_filesystem_size;
static time_t s_last_check;
static int s_channel;
static bool s_auto_update;
static int s_check_interval;
static bool s_install_running;

static const esp_vfs_littlefs_conf_t LITTLEFS_CONF = {
    .base_path = "/lfs",
    .partition_label = "littlefs",
    .format_if_mount_failed = true,
};

static const esp_vfs_littlefs_conf_t LITTLEFS_REMOUNT_CONF = {
    .base_path = "/lfs",
    .partition_label = "littlefs",
    .format_if_mount_failed = false,
};

static bool begin_write(void)
{
    portENTER_CRITICAL(&s_lock);
    bool available = !s_install_running;
    if (available) s_install_running = true;
    portEXIT_CRITICAL(&s_lock);
    return available;
}

static void end_write(void)
{
    portENTER_CRITICAL(&s_lock);
    s_install_running = false;
    portEXIT_CRITICAL(&s_lock);
}

static bool littlefs_header_valid(const unsigned char *data, size_t length, size_t image_size)
{
    static const unsigned char prefix[] = {
        0x06, 0x00, 0x00, 0x00, 0xf0, 0x0f, 0xff, 0xf7,
        'l', 'i', 't', 't', 'l', 'e', 'f', 's'
    };
    return image_size > 0 && length >= sizeof(prefix) &&
           memcmp(data, prefix, sizeof(prefix)) == 0;
}

static void copy_string(char *target, size_t size, const char *source)
{
    snprintf(target, size, "%s", source ? source : "");
}

static void set_error(const char *code, const char *detail)
{
    portENTER_CRITICAL(&s_lock);
    copy_string(s_error, sizeof(s_error), code);
    copy_string(s_error_detail, sizeof(s_error_detail), detail);
    s_state = OTA_FAILED;
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGE(TAG, "%s: %s", code ? code : "otaError", detail ? detail : "");
}

static void clear_error(void)
{
    portENTER_CRITICAL(&s_lock);
    s_error[0] = 0;
    s_error_detail[0] = 0;
    portEXIT_CRITICAL(&s_lock);
}

static void load_config(void)
{
    nvs_handle_t nvs;
    s_channel = 0;
    s_auto_update = false;
    s_check_interval = 24;
    if (nvs_open(OTA_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) return;
    int8_t channel = 0;
    uint8_t automatic = 0;
    uint16_t interval = 24;
    nvs_get_i8(nvs, "channel", &channel);
    nvs_get_u8(nvs, "auto", &automatic);
    nvs_get_u16(nvs, "interval", &interval);
    nvs_close(nvs);
    s_channel = channel == 1 ? 1 : 0;
    s_auto_update = automatic != 0;
    s_check_interval = interval;
}

static bool save_config(void)
{
    nvs_handle_t nvs;
    esp_err_t error = nvs_open(OTA_NAMESPACE, NVS_READWRITE, &nvs);
    if (error != ESP_OK) return false;
    if (error == ESP_OK) error = nvs_set_i8(nvs, "channel", s_channel);
    if (error == ESP_OK) error = nvs_set_u8(nvs, "auto", s_auto_update ? 1 : 0);
    if (error == ESP_OK) error = nvs_set_u16(nvs, "interval", s_check_interval);
    if (error == ESP_OK) error = nvs_commit(nvs);
    nvs_close(nvs);
    return error == ESP_OK;
}

static void read_fs_version(void)
{
    FILE *file = fopen("/lfs/version.json", "r");
    if (!file) return;
    char body[256] = {0};
    fread(body, 1, sizeof(body) - 1, file);
    fclose(file);
    cJSON *root = cJSON_Parse(body);
    cJSON *version = root ? cJSON_GetObjectItem(root, "version") : NULL;
    if (cJSON_IsString(version)) copy_string(s_fs_version, sizeof(s_fs_version), version->valuestring);
    cJSON_Delete(root);
}

static int compare_versions(const char *left, const char *right)
{
    int a[3] = {0}, b[3] = {0};
    sscanf(left ? left : "", "%d.%d.%d", &a[0], &a[1], &a[2]);
    sscanf(right ? right : "", "%d.%d.%d", &b[0], &b[1], &b[2]);
    for (int i = 0; i < 3; ++i) {
        if (a[i] != b[i]) return a[i] > b[i] ? 1 : -1;
    }
    return 0;
}

static bool should_replace(const char *offered, const char *installed)
{
    if (!offered || !offered[0]) return false;
    if (s_channel == 1) return strcmp(offered, installed ? installed : "") != 0;
    return compare_versions(offered, installed) > 0;
}

static bool update_available(void)
{
    return should_replace(s_available_version, ROTATOR_VERSION) ||
           should_replace(s_available_version, s_fs_version);
}

static esp_err_t json_response(httpd_req_t *req, cJSON *root)
{
    char *text = cJSON_PrintUnformatted(root);
    if (!text) return ESP_ERR_NO_MEM;
    httpd_resp_set_type(req, "application/json");
    esp_err_t result = httpd_resp_sendstr(req, text);
    free(text);
    return result;
}

static esp_err_t send_status(httpd_req_t *req)
{
    cJSON *root = cJSON_CreateObject();
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
    const esp_app_desc_t *description = esp_app_get_description();

    cJSON_AddStringToObject(root, "firmwareVersion", ROTATOR_VERSION);
    cJSON_AddStringToObject(root, "fsVersion", s_fs_version);
    cJSON_AddStringToObject(root, "projectName", description ? description->project_name : "Rotator");
    cJSON_AddStringToObject(root, "buildDate", description ? description->date : "");
    cJSON_AddNumberToObject(root, "sketchSize", rotator_get_sketch_size());
    cJSON_AddNumberToObject(root, "freeSpace", next ? next->size : 0);
    cJSON_AddStringToObject(root, "partition", running ? running->label : "?");
    cJSON_AddNumberToObject(root, "channel", s_channel);
    cJSON_AddBoolToObject(root, "autoUpdate", s_auto_update);
    cJSON_AddNumberToObject(root, "checkInterval", s_check_interval);
    cJSON_AddStringToObject(root, "state", STATE_NAMES[s_state]);
    cJSON_AddNumberToObject(root, "progress", s_progress);
    cJSON_AddStringToObject(root, "availableVersion", s_available_version);
    cJSON_AddStringToObject(root, "availableNotes", s_available_notes);
    cJSON_AddBoolToObject(root, "updateAvailable", update_available());
    cJSON_AddStringToObject(root, "error", s_error);
    cJSON_AddStringToObject(root, "errorDetail", s_error_detail);
    time_t checked = s_last_check;

    time_t now = time(NULL);
    cJSON_AddNumberToObject(root, "lastCheck", checked && now > checked ? difftime(now, checked) : -1);
    esp_err_t result = json_response(req, root);
    cJSON_Delete(root);
    return result;
}

typedef struct {
    char *data;
    size_t length;
    size_t capacity;
} response_buffer_t;

// ESP-IDF 5.4's esp_http_client appends each hop's Location header to the
// previous one instead of replacing it, so its own automatic-redirect
// following silently builds a corrupt URL across GitHub's release-asset
// redirect chain. Redirects are
// therefore disabled and followed by hand: this buffer is app-owned, cleared
// before every request, and holds only the Location header of the response
// that was just received.
typedef struct {
    char location[OTA_LOCATION_BUF];
    response_buffer_t *body; // NULL when the caller reads the body itself
} ota_http_ctx_t;

static esp_err_t ota_http_event_handler(esp_http_client_event_t *event)
{
    ota_http_ctx_t *ctx = event->user_data;
    if (event->event_id == HTTP_EVENT_ON_HEADER && strcasecmp(event->header_key, "Location") == 0) {
        snprintf(ctx->location, sizeof(ctx->location), "%s", event->header_value ? event->header_value : "");
    } else if (event->event_id == HTTP_EVENT_ON_DATA && ctx->body && event->data && event->data_len > 0) {
        response_buffer_t *buffer = ctx->body;
        size_t needed = buffer->length + event->data_len + 1;
        if (needed > buffer->capacity) {
            size_t capacity = needed * 2;
            char *grown = realloc(buffer->data, capacity);
            if (!grown) return ESP_ERR_NO_MEM;
            buffer->data = grown;
            buffer->capacity = capacity;
        }
        memcpy(buffer->data + buffer->length, event->data, event->data_len);
        buffer->length += event->data_len;
        buffer->data[buffer->length] = 0;
    }
    return ESP_OK;
}

static bool fetch_manifest(void)
{
    if (!wifi_manager_is_connected()) {
        set_error("otaOffline", "WLAN is not connected");
        return false;
    }
    s_state = OTA_CHECKING;
    clear_error();
    response_buffer_t body = {0};
    ota_http_ctx_t ctx = {.body = &body};
    esp_http_client_config_t config = {
        .url = s_channel == 1 ? OTA_MANIFEST_EDGE : OTA_MANIFEST_STABLE,
        .event_handler = ota_http_event_handler,
        .user_data = &ctx,
        .timeout_ms = OTA_HTTP_TIMEOUT_MS,
        .buffer_size = 16384,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .user_agent = "Rotator/" ROTATOR_VERSION,
        .disable_auto_redirect = true,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t error = client ? ESP_OK : ESP_ERR_NO_MEM;
    int status = 0;
    for (int redirects = 0; client && error == ESP_OK; ++redirects) {
        ctx.location[0] = 0;
        body.length = 0;
        if (body.data) body.data[0] = 0;
        error = esp_http_client_perform(client);
        status = esp_http_client_get_status_code(client);
        if (error != ESP_OK || status < 300 || status >= 400 || !ctx.location[0]) break;
        if (redirects >= OTA_MAX_REDIRECTS) { error = ESP_FAIL; break; }
        esp_http_client_set_url(client, ctx.location);
    }
    if (client) esp_http_client_cleanup(client);
    if (error != ESP_OK || status != 200) {
        char detail[64];
        snprintf(detail, sizeof(detail), "HTTP %d / %s", status, esp_err_to_name(error));
        free(body.data);
        set_error("otaManifestHttp", detail);
        return false;
    }

    cJSON *root = cJSON_Parse(body.data ? body.data : "");
    free(body.data);
    if (!root) {
        set_error("otaManifestParse", "Invalid JSON");
        return false;
    }
    cJSON *firmware = cJSON_GetObjectItem(root, "firmware");
    cJSON *filesystem = cJSON_GetObjectItem(root, "filesystem");
    cJSON *version = cJSON_GetObjectItem(root, "version");
    cJSON *notes = cJSON_GetObjectItem(root, "notes");
    cJSON *fw_url = cJSON_GetObjectItem(firmware, "url");
    cJSON *fw_sha = cJSON_GetObjectItem(firmware, "sha256");
    cJSON *fw_size = cJSON_GetObjectItem(firmware, "size");
    cJSON *fs_url = cJSON_GetObjectItem(filesystem, "url");
    cJSON *fs_sha = cJSON_GetObjectItem(filesystem, "sha256");
    cJSON *fs_size = cJSON_GetObjectItem(filesystem, "size");
    bool valid = cJSON_IsString(version) && cJSON_IsObject(firmware) && cJSON_IsObject(filesystem) &&
                 cJSON_IsString(fw_url) && cJSON_IsString(fw_sha) && cJSON_IsNumber(fw_size) &&
                 cJSON_IsString(fs_url) && cJSON_IsString(fs_sha) && cJSON_IsNumber(fs_size);
    if (valid) {
        valid = strlen(fw_sha->valuestring) == 64 && strlen(fs_sha->valuestring) == 64 &&
                strncmp(fw_url->valuestring, "https://github.com/FranzKugler/Rotator/releases/download/", 57) == 0 &&
                strncmp(fs_url->valuestring, "https://github.com/FranzKugler/Rotator/releases/download/", 57) == 0;
    }
    if (!valid) {
        cJSON_Delete(root);
        set_error("otaManifestParse", "Missing manifest fields");
        return false;
    }
    portENTER_CRITICAL(&s_lock);
    copy_string(s_available_version, sizeof(s_available_version), version->valuestring);
    copy_string(s_available_notes, sizeof(s_available_notes), cJSON_IsString(notes) ? notes->valuestring : "");
    copy_string(s_firmware_url, sizeof(s_firmware_url), fw_url->valuestring);
    copy_string(s_firmware_sha, sizeof(s_firmware_sha), fw_sha->valuestring);
    copy_string(s_filesystem_url, sizeof(s_filesystem_url), fs_url->valuestring);
    copy_string(s_filesystem_sha, sizeof(s_filesystem_sha), fs_sha->valuestring);
    s_firmware_size = fw_size->valuedouble;
    s_filesystem_size = fs_size->valuedouble;
    s_last_check = time(NULL);
    s_state = update_available() ? OTA_AVAILABLE : OTA_IDLE;
    portEXIT_CRITICAL(&s_lock);
    cJSON_Delete(root);
    return true;
}

static esp_err_t check_handler(httpd_req_t *req)
{
    if (s_install_running) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"error\":\"otaBusy\"}");
    }
    fetch_manifest();
    return send_status(req);
}

static esp_err_t config_handler(httpd_req_t *req)
{
    if (!expert_lock_guard(req)) return ESP_FAIL;
    if (s_install_running) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"error\":\"otaBusy\"}");
    }
    if (req->content_len <= 0 || req->content_len > 512) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON body");
        return ESP_FAIL;
    }
    char body[513] = {0};
    int received = httpd_req_recv(req, body, req->content_len);
    if (received <= 0) return ESP_FAIL;
    cJSON *root = cJSON_Parse(body);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    cJSON *channel = cJSON_GetObjectItem(root, "channel");
    cJSON *automatic = cJSON_GetObjectItem(root, "autoUpdate");
    cJSON *interval = cJSON_GetObjectItem(root, "checkInterval");
    if (cJSON_IsNumber(channel)) {
        s_channel = channel->valueint == 1 ? 1 : 0;
        s_available_version[0] = 0;
        s_available_notes[0] = 0;
        s_last_check = 0;
        s_state = OTA_IDLE;
    }
    if (cJSON_IsBool(automatic)) s_auto_update = cJSON_IsTrue(automatic);
    if (cJSON_IsNumber(interval)) {
        int hours = interval->valueint;
        s_check_interval = hours >= 0 && hours <= 168 ? hours : 24;
    }
    cJSON_Delete(root);
    if (!save_config()) {
        set_error("otaConfigSave", "Could not persist update settings");
        httpd_resp_set_status(req, "500 Internal Server Error");
    }
    return send_status(req);
}

static void digest_to_hex(const unsigned char digest[32], char output[65])
{
    static const char digits[] = "0123456789abcdef";
    for (int index = 0; index < 32; ++index) {
        output[index * 2] = digits[digest[index] >> 4];
        output[index * 2 + 1] = digits[digest[index] & 15];
    }
    output[64] = 0;
}

static esp_err_t stream_to_partition(const char *url, const char *expected_sha, size_t expected_size,
                                     const esp_partition_t *partition, bool firmware)
{
    if (!partition || expected_size == 0 || expected_size > partition->size) return ESP_ERR_INVALID_SIZE;
    ota_http_ctx_t ctx = {0};
    esp_http_client_config_t config = {
        .url = url,
        .event_handler = ota_http_event_handler,
        .user_data = &ctx,
        .timeout_ms = OTA_HTTP_TIMEOUT_MS,
        .buffer_size = 16384,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .user_agent = "Rotator/" ROTATOR_VERSION,
        .disable_auto_redirect = true,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client || esp_http_client_open(client, 0) != ESP_OK) {
        if (client) esp_http_client_cleanup(client);
        return ESP_FAIL;
    }
    esp_http_client_fetch_headers(client);
    for (int redirects = 0; redirects < OTA_MAX_REDIRECTS; ++redirects) {
        int status = esp_http_client_get_status_code(client);
        if (status < 300 || status >= 400 || !ctx.location[0]) break;
        char location[OTA_LOCATION_BUF];
        snprintf(location, sizeof(location), "%s", ctx.location);
        ctx.location[0] = 0;
        esp_http_client_set_url(client, location);
        esp_http_client_close(client);
        if (esp_http_client_open(client, 0) != ESP_OK) break;
        esp_http_client_fetch_headers(client);
    }
    if (esp_http_client_get_status_code(client) != 200) {
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    esp_ota_handle_t ota_handle = 0;
    bool fs_unmounted = false;
    esp_err_t error;
    if (firmware) {
        error = esp_ota_begin(partition, expected_size, &ota_handle);
    } else {
        error = esp_vfs_littlefs_unregister(LITTLEFS_CONF.partition_label);
        if (error != ESP_OK) {
            esp_http_client_cleanup(client);
            return error;
        }
        fs_unmounted = true;
        error = esp_partition_erase_range(partition, 0, partition->size);
    }
    if (error != ESP_OK) {
        esp_http_client_cleanup(client);
        if (fs_unmounted) esp_vfs_littlefs_register(&LITTLEFS_REMOUNT_CONF);
        return error;
    }

    mbedtls_sha256_context sha;
    mbedtls_sha256_init(&sha);
    mbedtls_sha256_starts(&sha, 0);
    unsigned char buffer[2048];
    size_t written = 0;
    while (written < expected_size) {
        int read = esp_http_client_read(client, (char *)buffer,
                                       expected_size - written < sizeof(buffer) ? expected_size - written : sizeof(buffer));
        if (read <= 0) { error = ESP_FAIL; break; }
        mbedtls_sha256_update(&sha, buffer, read);
        error = firmware ? esp_ota_write(ota_handle, buffer, read)
                         : esp_partition_write(partition, written, buffer, read);
        if (error != ESP_OK) break;
        written += read;
        s_progress = (int)((written * 100) / expected_size);
    }
    unsigned char digest[32];
    mbedtls_sha256_finish(&sha, digest);
    mbedtls_sha256_free(&sha);
    esp_http_client_cleanup(client);

    char actual_sha[65];
    digest_to_hex(digest, actual_sha);
    if (error == ESP_OK && written != expected_size) error = ESP_ERR_INVALID_SIZE;
    if (error == ESP_OK && strcasecmp(actual_sha, expected_sha) != 0) error = ESP_ERR_INVALID_CRC;
    if (firmware) {
        if (error == ESP_OK) error = esp_ota_end(ota_handle);
        else esp_ota_abort(ota_handle);
    } else if (fs_unmounted) {
        esp_err_t mount_error = esp_vfs_littlefs_register(&LITTLEFS_REMOUNT_CONF);
        if (error == ESP_OK) error = mount_error;
        // The raw image replaces config.json too. Recreate it from the live
        // configuration even when the download failed after the erase.
        if (mount_error == ESP_OK) ConfigurationSave();
    }
    return error;
}

static void reboot_task(void *unused)
{
    vTaskDelay(pdMS_TO_TICKS(OTA_REBOOT_DELAY_MS));
    esp_restart();
}

static void install_task(void *unused)
{
    esp_err_t error = ESP_OK;
    bool firmware_written = false;
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
    const esp_partition_t *filesystem = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_LITTLEFS, LITTLEFS_CONF.partition_label);

    if (should_replace(s_available_version, ROTATOR_VERSION)) {
        if (!next) error = ESP_ERR_NOT_FOUND;
        else {
            error = stream_to_partition(s_firmware_url, s_firmware_sha, s_firmware_size, next, true);
            firmware_written = error == ESP_OK;
        }
    }
    if (error == ESP_OK && should_replace(s_available_version, s_fs_version)) {
        if (!filesystem) error = ESP_ERR_NOT_FOUND;
        else error = stream_to_partition(s_filesystem_url, s_filesystem_sha, s_filesystem_size, filesystem, false);
    }
    // Only activate the new firmware after every requested image has passed its
    // size/hash check and the filesystem is mounted again.
    if (error == ESP_OK && firmware_written) error = esp_ota_set_boot_partition(next);
    portENTER_CRITICAL(&s_lock);
    s_install_running = false;
    if (error == ESP_OK) {
        s_progress = 100;
        s_state = OTA_INSTALLED;
    }
    portEXIT_CRITICAL(&s_lock);
    if (error == ESP_OK) xTaskCreate(reboot_task, "otaReboot", 2048, NULL, 1, NULL);
    else set_error("otaInstall", esp_err_to_name(error));
    vTaskDelete(NULL);
}

static esp_err_t install_handler(httpd_req_t *req)
{
    if (!expert_lock_guard(req)) return ESP_FAIL;
    if (!update_available()) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"error\":\"otaNoUpdate\"}");
    }
    bool busy = !begin_write();
    if (!busy) {
        s_state = OTA_DOWNLOADING;
        s_progress = 0;
    }
    if (busy) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"error\":\"otaBusy\"}");
    }
    if (xTaskCreatePinnedToCore(install_task, "otaInstall", OTA_TASK_STACK, NULL, 2, NULL, 0) != pdPASS) {
        end_write();
        set_error("otaTask", "Could not start update task");
    }
    return send_status(req);
}

static esp_err_t upload_handler(httpd_req_t *req)
{
    if (!expert_lock_guard(req)) return ESP_FAIL;
    if (!begin_write()) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"error\":\"otaBusy\"}");
    }
    if (req->content_len <= 0) {
        end_write();
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No image");
        return ESP_FAIL;
    }
    unsigned char buffer[2048];
    int first = httpd_req_recv(req, (char *)buffer, sizeof(buffer));
    if (first <= 0) { end_write(); return ESP_FAIL; }
    bool firmware = buffer[0] == 0xE9;
    const esp_partition_t *partition = firmware
        ? esp_ota_get_next_update_partition(NULL)
        : esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_LITTLEFS,
                                   LITTLEFS_CONF.partition_label);
    if (!partition) { end_write(); return ESP_ERR_NOT_FOUND; }
    if (req->content_len > partition->size) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Image is too large");
        end_write();
        return ESP_ERR_INVALID_SIZE;
    }
    if (!firmware && (req->content_len != partition->size ||
                      !littlefs_header_valid(buffer, first, req->content_len))) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Not a Rotator LittleFS image");
        end_write();
        return ESP_ERR_INVALID_ARG;
    }

    esp_ota_handle_t handle = 0;
    esp_err_t error;
    if (firmware) error = esp_ota_begin(partition, req->content_len, &handle);
    else {
        error = esp_vfs_littlefs_unregister(LITTLEFS_CONF.partition_label);
        if (error == ESP_OK) error = esp_partition_erase_range(partition, 0, partition->size);
    }
    size_t written = 0;
    int chunk = first;
    while (error == ESP_OK && chunk > 0) {
        error = firmware ? esp_ota_write(handle, buffer, chunk)
                         : esp_partition_write(partition, written, buffer, chunk);
        written += chunk;
        if (written >= req->content_len) break;
        chunk = httpd_req_recv(req, (char *)buffer,
                               req->content_len - written < sizeof(buffer) ? req->content_len - written : sizeof(buffer));
        if (chunk <= 0) error = ESP_FAIL;
    }
    if (firmware) {
        if (error == ESP_OK) error = esp_ota_end(handle); else esp_ota_abort(handle);
        if (error == ESP_OK) error = esp_ota_set_boot_partition(partition);
    } else {
        esp_err_t mounted = esp_vfs_littlefs_register(&LITTLEFS_REMOUNT_CONF);
        if (error == ESP_OK) error = mounted;
        if (mounted == ESP_OK) ConfigurationSave();
    }
    if (error != ESP_OK) {
        end_write();
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(error));
        return error;
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, firmware ? "{\"kind\":\"firmware\",\"reboot\":true}"
                                      : "{\"kind\":\"filesystem\",\"reboot\":true}");
    end_write();
    xTaskCreate(reboot_task, "otaReboot", 2048, NULL, 1, NULL);
    return ESP_OK;
}

static void periodic_task(void *unused)
{
    int elapsed_minutes = 0;
    int first_check_minutes = 2;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(60000));
        if (!wifi_manager_is_connected() || s_install_running) continue;

        if (s_check_interval > 0) {
            int due_minutes = first_check_minutes > 0 ? first_check_minutes : s_check_interval * 60;
            if (++elapsed_minutes >= due_minutes) {
                elapsed_minutes = 0;
                first_check_minutes = 0;
                fetch_manifest();
            }
        }

        // A pending offer is considered every minute, independently of when
        // the interval check happened, so it cannot miss the night window.
        if (!s_auto_update || !update_available()) continue;
        time_t now = time(NULL);
        struct tm local = {0};
        localtime_r(&now, &local);
        if (now > 1700000000 && local.tm_hour >= OTA_AUTO_HOUR_FROM && local.tm_hour < OTA_AUTO_HOUR_TO) {
            if (!begin_write()) continue;
            s_state = OTA_DOWNLOADING;
            s_progress = 0;
            if (xTaskCreatePinnedToCore(install_task, "otaInstall", OTA_TASK_STACK,
                                        NULL, 2, NULL, 0) != pdPASS) {
                end_write();
                set_error("otaTask", "Could not start automatic update task");
            }
        }
    }
}

static esp_err_t register_uri(const char *uri, httpd_method_t method, esp_err_t (*handler)(httpd_req_t *))
{
    httpd_uri_t route = {.uri = uri, .method = method, .handler = handler, .user_ctx = NULL};
    return httpd_register_uri_handler(s_server, &route);
}

esp_err_t register_ota_update_uri(httpd_handle_t server)
{
    s_server = server;
    load_config();
    read_fs_version();
    ESP_ERROR_CHECK(register_uri("/ota/status", HTTP_GET, send_status));
    ESP_ERROR_CHECK(register_uri("/ota/check", HTTP_GET, check_handler));
    ESP_ERROR_CHECK(register_uri("/ota/install", HTTP_POST, install_handler));
    ESP_ERROR_CHECK(register_uri("/ota/config", HTTP_POST, config_handler));
    ESP_ERROR_CHECK(register_uri("/ota/upload", HTTP_POST, upload_handler));
    xTaskCreate(periodic_task, "otaPoll", 4096, NULL, 1, NULL);
    return ESP_OK;
}
