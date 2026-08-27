#include "ExpertLock.h"

#include "cJSON.h"
#include "esp_random.h"
#include "esp_system.h"
#include "mbedtls/sha256.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define EXPERT_NAMESPACE "expert"
#define SALT_BYTES 16
#define HASH_BYTES 32
#define MIN_PASSWORD 6
#define MAX_FAILURES 5
#define LOCKOUT_MS (5 * 60 * 1000UL)
#define GRACE_MS (5 * 60 * 1000UL)

static bool s_enrolled;
static bool s_unlocked;
static bool s_power_on;
static uint32_t s_started;
static uint8_t s_failures;
static uint32_t s_lockout_until;

static void digest(const uint8_t salt[SALT_BYTES], const char *password, uint8_t out[HASH_BYTES])
{
    mbedtls_sha256_context context;
    mbedtls_sha256_init(&context);
    mbedtls_sha256_starts(&context, 0);
    mbedtls_sha256_update(&context, salt, SALT_BYTES);
    mbedtls_sha256_update(&context, (const uint8_t *)password, strlen(password));
    mbedtls_sha256_finish(&context, out);
    mbedtls_sha256_free(&context);
}

static bool same_digest(const uint8_t *left, const uint8_t *right)
{
    uint8_t difference = 0;
    for (size_t index = 0; index < HASH_BYTES; ++index) difference |= left[index] ^ right[index];
    return difference == 0;
}

static bool grace_open(void)
{
    return s_power_on && (xTaskGetTickCount() * portTICK_PERIOD_MS - s_started) < GRACE_MS;
}

static bool locked_out(void)
{
    if (!s_lockout_until) return false;
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if ((int32_t)(now - s_lockout_until) >= 0) {
        s_lockout_until = 0;
        s_failures = 0;
        return false;
    }
    return true;
}

static void store_unlocked(bool value)
{
    nvs_handle_t nvs;
    if (nvs_open(EXPERT_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) return;
    nvs_set_u8(nvs, "on", value ? 1 : 0);
    nvs_commit(nvs);
    nvs_close(nvs);
}

static esp_err_t send_json(httpd_req_t *req, cJSON *root)
{
    char *text = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    esp_err_t result = text ? httpd_resp_sendstr(req, text) : ESP_ERR_NO_MEM;
    free(text);
    return result;
}

static esp_err_t status_handler(httpd_req_t *req)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "enrolled", s_enrolled);
    cJSON_AddBoolToObject(root, "unlocked", s_unlocked);
    cJSON_AddBoolToObject(root, "lockedOut", locked_out());
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t grace = grace_open() ? (GRACE_MS - (now - s_started)) / 1000 : 0;
    cJSON_AddNumberToObject(root, "grace", grace);
    esp_err_t result = send_json(req, root);
    cJSON_Delete(root);
    return result;
}

static char *read_password(httpd_req_t *req)
{
    if (req->content_len <= 0 || req->content_len > 256) return NULL;
    char body[257] = {0};
    int received = httpd_req_recv(req, body, req->content_len);
    if (received <= 0) return NULL;
    cJSON *root = cJSON_Parse(body);
    cJSON *password = root ? cJSON_GetObjectItem(root, "password") : NULL;
    char *copy = cJSON_IsString(password) ? strdup(password->valuestring) : NULL;
    cJSON_Delete(root);
    return copy;
}

static esp_err_t enroll_handler(httpd_req_t *req)
{
    char *password = read_password(req);
    if (s_enrolled || !password || strlen(password) < MIN_PASSWORD) {
        free(password);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Password must contain at least 6 characters");
        return ESP_FAIL;
    }
    uint8_t salt[SALT_BYTES], hash[HASH_BYTES];
    esp_fill_random(salt, sizeof(salt));
    digest(salt, password, hash);
    free(password);
    nvs_handle_t nvs;
    if (nvs_open(EXPERT_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) return ESP_FAIL;
    nvs_set_blob(nvs, "salt", salt, sizeof(salt));
    nvs_set_blob(nvs, "hash", hash, sizeof(hash));
    nvs_set_u8(nvs, "on", 1);
    nvs_commit(nvs);
    nvs_close(nvs);
    s_enrolled = true;
    s_unlocked = true;
    return status_handler(req);
}

static esp_err_t unlock_handler(httpd_req_t *req)
{
    if (!s_enrolled || locked_out()) {
        httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Expert mode locked");
        return ESP_FAIL;
    }
    char *password = read_password(req);
    uint8_t salt[SALT_BYTES], stored[HASH_BYTES], offered[HASH_BYTES];
    size_t salt_length = sizeof(salt), hash_length = sizeof(stored);
    nvs_handle_t nvs;
    bool read = password && nvs_open(EXPERT_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK;
    if (read) {
        read = nvs_get_blob(nvs, "salt", salt, &salt_length) == ESP_OK &&
               nvs_get_blob(nvs, "hash", stored, &hash_length) == ESP_OK;
        nvs_close(nvs);
    }
    if (read) digest(salt, password, offered);
    free(password);
    if (!read || !same_digest(stored, offered)) {
        if (++s_failures >= MAX_FAILURES) {
            s_lockout_until = xTaskGetTickCount() * portTICK_PERIOD_MS + LOCKOUT_MS;
            if (!s_lockout_until) s_lockout_until = 1;
        }
        httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Wrong password");
        return ESP_FAIL;
    }
    s_failures = 0;
    s_lockout_until = 0;
    s_unlocked = true;
    store_unlocked(true);
    return status_handler(req);
}

static esp_err_t lock_handler(httpd_req_t *req)
{
    s_unlocked = false;
    store_unlocked(false);
    return status_handler(req);
}

static esp_err_t reset_handler(httpd_req_t *req)
{
    if (!grace_open()) {
        httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Reset window closed");
        return ESP_FAIL;
    }
    nvs_handle_t nvs;
    if (nvs_open(EXPERT_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_erase_all(nvs);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    s_enrolled = false;
    s_unlocked = false;
    return status_handler(req);
}

void expert_lock_init(void)
{
    s_started = xTaskGetTickCount() * portTICK_PERIOD_MS;
    s_power_on = esp_reset_reason() == ESP_RST_POWERON;
    nvs_handle_t nvs;
    if (nvs_open(EXPERT_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) return;
    size_t salt = 0, hash = 0;
    s_enrolled = nvs_get_blob(nvs, "salt", NULL, &salt) == ESP_OK && salt == SALT_BYTES &&
                 nvs_get_blob(nvs, "hash", NULL, &hash) == ESP_OK && hash == HASH_BYTES;
    uint8_t unlocked = 0;
    nvs_get_u8(nvs, "on", &unlocked);
    nvs_close(nvs);
    s_unlocked = s_enrolled && unlocked == 1;
}

bool expert_lock_guard(httpd_req_t *req)
{
    if (s_unlocked) return true;
    httpd_resp_set_status(req, "403 Forbidden");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"error\":\"expertLocked\"}");
    return false;
}

static esp_err_t add_route(httpd_handle_t server, const char *uri, httpd_method_t method,
                           esp_err_t (*handler)(httpd_req_t *))
{
    httpd_uri_t route = {.uri = uri, .method = method, .handler = handler, .user_ctx = NULL};
    return httpd_register_uri_handler(server, &route);
}

esp_err_t expert_lock_register_routes(httpd_handle_t server)
{
    ESP_ERROR_CHECK(add_route(server, "/expert", HTTP_GET, status_handler));
    ESP_ERROR_CHECK(add_route(server, "/expert/enroll", HTTP_POST, enroll_handler));
    ESP_ERROR_CHECK(add_route(server, "/expert/unlock", HTTP_POST, unlock_handler));
    ESP_ERROR_CHECK(add_route(server, "/expert/lock", HTTP_POST, lock_handler));
    ESP_ERROR_CHECK(add_route(server, "/expert/reset", HTTP_POST, reset_handler));
    return ESP_OK;
}
