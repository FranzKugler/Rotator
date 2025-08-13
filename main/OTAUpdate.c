#include "OTAUpdate.h"
#include "esp_ota_ops.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "esp_littlefs.h"
#include "esp_system.h"
#include "esp_log.h"
#include <string.h>

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

static const char *TAG = "OTAUpdate";
extern void ConfigurationSave();

// LittleFS partition config
static const esp_vfs_littlefs_conf_t littlefs_conf = {
    .base_path = "/lfs",
    .partition_label = "littlefs",
    .format_if_mount_failed = true};

esp_err_t ota_update_raw_post_handler(httpd_req_t *req)
{
    esp_ota_handle_t update_handle = 0;
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);

    if (!update_partition)
    {
        ESP_LOGE(TAG, "No OTA partition found");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA partition not found");
        return ESP_FAIL;
    }

    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return err;
    }

    uint8_t buf[1024];
    int remaining = req->content_len;
    while (remaining > 0)
    {
        int to_read = (remaining < sizeof(buf)) ? remaining : sizeof(buf);
        int recv_len = httpd_req_recv(req, (char *)buf, to_read);
        if (recv_len <= 0)
        {
            ESP_LOGE(TAG, "Error receiving file");
            esp_ota_end(update_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Error receiving file");
            return ESP_FAIL;
        }
        err = esp_ota_write(update_handle, buf, recv_len);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_ota_write failed (%s)", esp_err_to_name(err));
            esp_ota_end(update_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA write failed");
            return err;
        }
        remaining -= recv_len;
    }

    if ((err = esp_ota_end(update_handle)) != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_end failed (%s)", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA end failed");
        return err;
    }

    if ((err = esp_ota_set_boot_partition(update_partition)) != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA set boot partition failed");
        return err;
    }

    httpd_resp_sendstr(req, "OTA update successful, rebooting...");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    esp_restart();

    return ESP_OK;
}

// HTTP POST handler for raw LittleFS image upload
static esp_err_t ota_update_raw_fs_handler(httpd_req_t *req)
{
    esp_err_t err;
    // Find LittleFS partition by label
    const esp_partition_t *fs_part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA,
        ESP_PARTITION_SUBTYPE_DATA_LITTLEFS,
        littlefs_conf.partition_label);
    if (!fs_part) {
        ESP_LOGE(TAG, "Partition not found");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Partition not found");
        return ESP_FAIL;
    }

    // Unmount VFS to allow raw write
    esp_vfs_littlefs_unregister(littlefs_conf.partition_label);

    // Erase entire LittleFS partition
    err = esp_partition_erase_range(fs_part, 0, fs_part->size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to erase partition: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Erase failed");
        return err;
    }

    // Read request body and write to partition
    char buf[1024];
    int received = 0;
    size_t remaining = req->content_len;
    while (remaining > 0) {
        int to_read = MIN(remaining, sizeof(buf));
        int ret = httpd_req_recv(req, buf, to_read);
        if (ret <= 0) {
            ESP_LOGE(TAG, "Failed to receive file data");
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive failed");
            return ESP_FAIL;
        }
        err = esp_partition_write(fs_part, received, buf, ret);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write partition: %s", esp_err_to_name(err));
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Write failed");
            return err;
        }
        remaining -= ret;
        received += ret;
    }

    ESP_LOGI(TAG, "Received and wrote %d bytes to LittleFS partition", received);

    // Re-register and mount LittleFS
    err = esp_vfs_littlefs_register(&littlefs_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to re-mount LittleFS: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Mount failed");
        return err;
    }

    // config.json is probably lost, so write it again from configuration store
    ConfigurationSave();

    // Send response
    httpd_resp_sendstr(req, "FS Update Success");

    // Optionally restart
    esp_restart();
    return ESP_OK;
}

esp_err_t register_ota_update_uri(httpd_handle_t server)
{

    httpd_uri_t ota_raw_post_uri = {
        .uri = "/update_raw",
        .method = HTTP_POST,
        .handler = ota_update_raw_post_handler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &ota_raw_post_uri);

    httpd_uri_t fs_uota_raw_post_fs = {
        .uri = "/update_fs_raw",
        .method = HTTP_POST,
        .handler = ota_update_raw_fs_handler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &fs_uota_raw_post_fs);

    return ESP_OK;
}
