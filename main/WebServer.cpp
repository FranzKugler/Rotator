#include "esp_log.h"
#include "cJSON.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_wifi.h"
#include "esp_http_server.h"

#include <string>
#include "Configuration.hpp"
#include "RotatorHW.h"
#include "WebServer.h"
#include "WifiManager.h"

static const char *TAG = "webserver";

struct WsSession
{
    httpd_handle_t hd;
    int fd;
};
static std::vector<WsSession> ws_sessions;
static AngleUpdateData latest{0, 0};

static esp_err_t static_get_handler(httpd_req_t *req)
{
    const char *base_path = "/lfs";
    const char *uri = req->uri;
    char path[1024]; // Erhöht, um Pfade sicher aufzunehmen

    // Wenn nur “/” angefragt, index.html ausliefern
    if (strcmp(uri, "/") == 0)
    {
        snprintf(path, sizeof(path), "%s/index.html", base_path);
    }
    else
    {
        // Alle anderen URIs direkt unter /lfs abbilden
        snprintf(path, sizeof(path), "%s%s", base_path, uri);
    }

    int fd = open(path, O_RDONLY);
    if (fd < 0)
    {
        ESP_LOGE(TAG, "File not found: %s", path);
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
        return ESP_FAIL;
    }

    // MIME-Type anhand der Dateiendung setzen
    if (strstr(path, ".html"))
    {
        httpd_resp_set_type(req, "text/html");
    }
    else if (strstr(path, ".css"))
    {
        httpd_resp_set_type(req, "text/css");
    }
    else if (strstr(path, ".js"))
    {
        httpd_resp_set_type(req, "application/javascript");
    }
    else
    {
        httpd_resp_set_type(req, "application/octet-stream");
    }

    // Datei in 1-KB-Chunks senden
    char buffer[1024];
    ssize_t read_bytes;
    while ((read_bytes = read(fd, buffer, sizeof(buffer))) > 0)
    {
        httpd_resp_send_chunk(req, buffer, read_bytes);
    }
    close(fd);

    // Signalisiere Ende
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

void angle_producer_task(void *)
{
    while (true)
    {
        latest.angle = RotatorHW::getInstance().getPosition();
        latest.mechAngle = RotatorHW::getInstance().getMechanicalPosition();
        latest.direction = RotatorHW::getInstance().getDirection();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void angle_event_broadcast(void *)
{
    while (true) {
        // JSON bauen
        char buf[64];
        int len = snprintf(buf, sizeof(buf),
            "{\"angle\":%.2f,\"mechAngle\":%.2f,\"direction\":\"%s\"}",
            latest.angle, latest.mechAngle, latest.direction ? "cw" : "ccw");

        // Frame komplett initialisieren
        httpd_ws_frame_t pkt;
        memset(&pkt, 0, sizeof(httpd_ws_frame_t));     // final=0, fragmented=0
        pkt.payload = (uint8_t*)buf;
        pkt.len     = len;
        pkt.type    = HTTPD_WS_TYPE_TEXT;

        //ESP_LOGI("evt broadcast", "%s", buf);
        // an alle Sessions senden
        for (auto it = ws_sessions.begin(); it != ws_sessions.end(); ) {
            esp_err_t err = httpd_ws_send_frame_async(it->hd, it->fd, &pkt);
            if (err != ESP_OK) {
                it = ws_sessions.erase(it);
            } else {
                ++it;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static esp_err_t angle_event_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        // Das GET-Upgrade übernimmt der Server automatisch
        int fd = httpd_req_to_sockfd(req);
        ws_sessions.push_back({ req->handle, fd });
        return ESP_OK;
    }
    // hier könntest Du eingehende Messages verarbeiten...
    return ESP_OK;
}

/*
static esp_err_t angle_event_handler(httpd_req_t *req)
{
    // Header einmal setzen
    httpd_resp_set_type(req, "text/event-stream");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Connection", "keep-alive");

    AngleUpdateData data;
    // so lange der Client verbunden ist…
    while (httpd_req_to_sockfd(req) >= 0)
    {
        // hier schläft der HTTPD-Task, bis ein Producer ein neues Paar
        // in die Queue schreibt – keine vTaskDelay(), kein Loop-Fressen!
        if (xQueueReceive(angleUpdateQueue, &data, portMAX_DELAY) != pdTRUE)
        {
            break; // Fehler oder keine Queue mehr
        }

        // und dann in einem Rutsch senden
        char buf[64];
        int len = snprintf(buf, sizeof(buf),
                           "data: {\"angle\":%.1f,\"mechAngle\":%.1f}\n\n",
                           data.angle, data.mechAngle);
        if (httpd_resp_send_chunk(req, buf, len) != ESP_OK)
        {
            break; // Client hat die Verbindung geschlossen
        }
    }
    // Ende des Streams
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}
*/

// GET /api/network/config
static esp_err_t get_network_config(httpd_req_t *req)
{
    auto &cfg = Configuration::getInstance();
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "ip", cfg.getIPAddressString().c_str());
    cJSON_AddStringToObject(root, "netmask", cfg.getNetmaskString().c_str());

    // MAC als Hex-String z. B. "01:23:45:67:89:AB"
    auto mac = cfg.getMACAddress();
    char macbuf[18];
    snprintf(macbuf, sizeof(macbuf), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    cJSON_AddStringToObject(root, "mac", macbuf);

    const char *json = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json);
    cJSON_free((void *)json);
    cJSON_Delete(root);
    return ESP_OK;
}

// POST helper: read body and parse JSON
static bool parse_json_body(httpd_req_t *req, cJSON **root_out)
{
    int len = req->content_len;
    std::string body;
    body.resize(len);
    int ret = httpd_req_recv(req, &body[0], len);
    if (ret <= 0)
        return false;
    *root_out = cJSON_ParseWithLength(body.c_str(), len);
    return *root_out != nullptr;
}

// POST /api/network/set/ip
static esp_err_t set_ip_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Set IP called.");
    cJSON *root = nullptr;
    if (!parse_json_body(req, &root))
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        ESP_LOGI(TAG, "Invalid JSON");
        return ESP_FAIL;
    }
    cJSON *ip_item = cJSON_GetObjectItem(root, "ip");
    if (!cJSON_IsString(ip_item))
    {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing 'ip'");
        ESP_LOGI(TAG, "Missing IP");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Set new IP to %s", ip_item->valuestring);
    Configuration::getInstance().setIPAddress(ip_item->valuestring);
    cJSON_Delete(root);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// POST /api/network/set/netmask
static esp_err_t set_netmask_handler(httpd_req_t *req)
{
    cJSON *root = nullptr;
    if (!parse_json_body(req, &root))
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    cJSON *nm_item = cJSON_GetObjectItem(root, "netmask");
    if (!cJSON_IsString(nm_item))
    {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing 'netmask'");
        return ESP_FAIL;
    }
    Configuration::getInstance().setNetmask(nm_item->valuestring);
    cJSON_Delete(root);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// POST /api/network/set/mac
static esp_err_t set_mac_handler(httpd_req_t *req)
{
    cJSON *root = nullptr;
    if (!parse_json_body(req, &root))
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    cJSON *mac_item = cJSON_GetObjectItem(root, "mac");
    if (!cJSON_IsString(mac_item))
    {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing 'mac'");
        return ESP_FAIL;
    }
    // Parse "AA:BB:CC:DD:EE:FF"
    std::array<uint8_t, 6> mac{};
    int vals[6];
    if (sscanf(mac_item->valuestring, "%02x:%02x:%02x:%02x:%02x:%02x",
               &vals[0], &vals[1], &vals[2], &vals[3], &vals[4], &vals[5]) == 6)
    {
        for (int i = 0; i < 6; ++i)
            mac[i] = static_cast<uint8_t>(vals[i]);
        Configuration::getInstance().setMACAddress(mac);
        cJSON_Delete(root);
        httpd_resp_sendstr(req, "OK");
        return ESP_OK;
    }
    else
    {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid MAC format");
        return ESP_FAIL;
    }
}

static esp_err_t calibration_zero_stream(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/event-stream");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Connection", "keep-alive");

    char buf[64];
    // Helper, um SSE-Events zu senden und kurz zu yielden
    auto send_event = [&](const char *evt, const char *data)
    {
        int len = snprintf(buf, sizeof(buf),
                           "event: %s\ndata: %s\n\n", evt, data);
        httpd_resp_send_chunk(req, buf, len);
        vTaskDelay(pdMS_TO_TICKS(10));
    };

    // Long-running zero measurement mit Progress-Callback
    int zero = RotatorHW::getInstance().measureMechanicalZero(
        [&](int pct)
        {
            char d[8];
            snprintf(d, sizeof(d), "%d", pct);
            send_event("progress", d);
        });
    char dv[8];
    snprintf(dv, sizeof(dv), "%d", zero);
    send_event("complete_zero", dv);

    httpd_resp_send_chunk(req, nullptr, 0);
    return ESP_OK;
}

static esp_err_t calibration_angle_stream(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/event-stream");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Connection", "keep-alive");

    char buf[64];
    auto send_event = [&](const char *evt, const char *data)
    {
        int len = snprintf(buf, sizeof(buf),
                           "event: %s\ndata: %s\n\n", evt, data);
        httpd_resp_send_chunk(req, buf, len);
        vTaskDelay(pdMS_TO_TICKS(10));
    };

    RotatorHW::getInstance().calibrateAngleSensor(
        [&](int pct)
        {
            char d[8];
            snprintf(d, sizeof(d), "%d", pct);
            send_event("progress", d);
        });
    send_event("complete_angle", "0");

    httpd_resp_send_chunk(req, nullptr, 0);
    return ESP_OK;
}

// WiFi Server
static esp_err_t wifi_scan_handler(httpd_req_t *req)
{
    // blockierend scannen
    wifi_scan_config_t scfg = {};
    esp_wifi_scan_start(&scfg, true);
    uint16_t n = 0;
    esp_wifi_scan_get_ap_num(&n);
    wifi_ap_record_t *recs = (wifi_ap_record_t *)malloc(n * sizeof(*recs));
    esp_wifi_scan_get_ap_records(&n, recs);

    cJSON *arr = cJSON_CreateArray();
    for (int i = 0; i < n; ++i)
    {
        cJSON *o = cJSON_CreateObject();
        cJSON_AddStringToObject(o, "ssid", (char *)recs[i].ssid);
        cJSON_AddNumberToObject(o, "rssi", recs[i].rssi);
        cJSON_AddNumberToObject(o, "authmode", recs[i].authmode);
        cJSON_AddItemToArray(arr, o);
    }
    free(recs);
    cJSON *root = cJSON_CreateObject();
    cJSON_AddItemToObject(root, "aps", arr);
    const char *out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    httpd_resp_set_type(req, "application/json");
    // ESP_LOGI("wifi scan", "%s", out);
    httpd_resp_sendstr(req, out);
    free((void *)out);
    return ESP_OK;
}

static esp_err_t wifi_connect_handler(httpd_req_t *req)
{
    // parse JSON body
    char buf[256] = {};
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0)
        return ESP_FAIL;
    cJSON *o = cJSON_Parse(buf);
    const char *ssid = cJSON_GetObjectItem(o, "ssid")->valuestring;
    const char *pwd = cJSON_GetObjectItem(o, "password")->valuestring;

    // store in NVS
    nvs_handle_t h;
    if (nvs_open("wifi", NVS_READWRITE, &h) == ESP_OK)
    {
        nvs_set_str(h, "ssid", ssid);
        nvs_set_str(h, "password", pwd);
        nvs_commit(h);
        nvs_close(h);
    }
    // configure and connect
    wifi_config_t wc = {};
    strcpy((char *)wc.sta.ssid, ssid);
    strcpy((char *)wc.sta.password, pwd);
    esp_wifi_set_config(WIFI_IF_STA, &wc);
    esp_wifi_connect();

    // answer with status
    char out[128];
    if (wifi_manager_is_connected())
    {
        snprintf(out, sizeof(out),
                 "{\"connected\":true,\"ip\":\"%s\"}",
                 wifi_manager_get_ip());
    }
    else
    {
        snprintf(out, sizeof(out),
                 "{\"connected\":false}");
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, out);
    cJSON_Delete(o);
    return ESP_OK;
}

// registration
void register_web_handles(httpd_handle_t server)
{
    httpd_uri_t angle_uri = {
        .uri = "/api/info/events",
        .method = HTTP_GET,
        .handler = angle_event_handler,
        .user_ctx = NULL,
        .is_websocket = true};
    httpd_register_uri_handler(server, &angle_uri);

    httpd_uri_t get_uri = {
        .uri = "/api/network/config",
        .method = HTTP_GET,
        .handler = get_network_config,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &get_uri);

    httpd_uri_t ip_uri = {
        .uri = "/api/network/set/ip",
        .method = HTTP_POST,
        .handler = set_ip_handler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &ip_uri);

    httpd_uri_t nm_uri = {
        .uri = "/api/network/set/netmask",
        .method = HTTP_POST,
        .handler = set_netmask_handler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &nm_uri);

    httpd_uri_t mac_uri = {
        .uri = "/api/network/set/mac",
        .method = HTTP_POST,
        .handler = set_mac_handler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &mac_uri);

    httpd_uri_t s1 = {
        .uri = "/api/wifi/scan",
        .method = HTTP_GET,
        .handler = wifi_scan_handler};
    httpd_register_uri_handler(server, &s1);

    httpd_uri_t s2 = {
        .uri = "/api/wifi/connect",
        .method = HTTP_POST,
        .handler = wifi_connect_handler};
    httpd_register_uri_handler(server, &s2);

    // SSE endpoints
    httpd_uri_t zero_sse = {
        .uri = "/api/calibration/zero/stream",
        .method = HTTP_GET,
        .handler = calibration_zero_stream,
        .user_ctx = nullptr};
    httpd_register_uri_handler(server, &zero_sse);

    httpd_uri_t angle_sse = {
        .uri = "/api/calibration/angle/stream",
        .method = HTTP_GET,
        .handler = calibration_angle_stream,
        .user_ctx = nullptr};
    httpd_register_uri_handler(server, &angle_sse);

    // root „/“ gets /lfs/index.html from littlefs
    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = static_get_handler,
        .user_ctx = NULL};
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &root_uri));

    // Wildcard „/*“ all other files in littlefs
    httpd_uri_t wildcard_uri = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = static_get_handler,
        .user_ctx = NULL};
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &wildcard_uri));
}
