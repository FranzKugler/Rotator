#include "esp_log.h"
#include "cJSON.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "esp_system.h"

#include <string>
#include "Configuration.hpp"
#include "ExpertLock.h"
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
        // Heartbeat for diagnosing main/RotatorHW.cpp's calibrateAngleSensor()
        // hang (memory/rotator_angle_cal_hang.md) - this task runs
        // concurrently with that sweep and is the other suspected party in
        // CALIBRATION_FINDINGS.md's "still-unresolved cross-task reliability
        // issue". If this line stops appearing in /log at the same moment
        // the calibration markers do, the freeze is somewhere shared
        // (mutex/ISR/hardware); if this keeps ticking while calibration's
        // markers stop, the freeze is specific to that sweep's own call
        // path. Deliberately at INFO - DEBUG is compiled out.
        ESP_LOGI("angProd", "tick");
        auto &rotator = RotatorHW::getInstance();
        latest.angle = rotator.getPosition();
        latest.mechAngle = rotator.getMechanicalPosition();
        latest.direction = rotator.getDirection();
        auto snapshot = rotator.getSensorSnapshot();
        latest.rawSensor = snapshot.rawSensor;
        latest.correctedSensor = snapshot.correctedSensor;
        latest.stepPosition = snapshot.stepPosition;
        latest.hall = snapshot.hall;
        ESP_LOGI("angProd", "tock stepPos=%ld hall=%d", (long)latest.stepPosition, latest.hall);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void angle_event_broadcast(void *)
{
    while (true) {
        // JSON bauen
        char buf[192];
        int len = snprintf(buf, sizeof(buf),
            "{\"angle\":%.2f,\"mechAngle\":%.2f,\"direction\":\"%s\","
            "\"rawSensor\":%u,\"correctedSensor\":%.2f,\"stepPosition\":%ld,\"hall\":%s}",
            latest.angle, latest.mechAngle, latest.direction ? "cw" : "ccw",
            latest.rawSensor, latest.correctedSensor, (long)latest.stepPosition,
            latest.hall ? "true" : "false");

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
    // httpd_req_recv() is not guaranteed to return the whole body in one
    // call - it only ever did for this project's small JSON bodies (a few
    // hundred bytes) until /api/calibration/fullstep-table's ~4KB payload
    // (400 numbers) exposed it: a partial recv left the rest of `body`
    // uninitialized while parsing still used the full `len`, so every such
    // upload failed to parse. Loop until the declared content_len is fully
    // read (or a real error/close occurs).
    int received = 0;
    while (received < len)
    {
        int ret = httpd_req_recv(req, &body[received], len - received);
        if (ret <= 0)
            return false;
        received += ret;
    }
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

/**
 * GET/POST /api/calibration/nominal-direction - {"clockwise": <bool>}
 *
 * The Calibration tab's "Nominal Direction" setting - see
 * RotatorHW::getNominalClockwise()/putNominalClockwise() and
 * Configuration.hpp's ConfigData::nominalClockwise. Ordinary application
 * configuration, not raw hardware access - not expert-gated, like the
 * network settings above.
 */
static esp_err_t get_nominal_direction_handler(httpd_req_t *req)
{
    bool clockwise = RotatorHW::getInstance().getNominalClockwise();
    char buf[32];
    int len = snprintf(buf, sizeof(buf), "{\"clockwise\":%s}", clockwise ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, len);
    return ESP_OK;
}

static esp_err_t set_nominal_direction_handler(httpd_req_t *req)
{
    cJSON *root = nullptr;
    if (!parse_json_body(req, &root))
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    cJSON *cwItem = cJSON_GetObjectItem(root, "clockwise");
    if (!cJSON_IsBool(cwItem))
    {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing 'clockwise'");
        return ESP_FAIL;
    }
    RotatorHW::getInstance().putNominalClockwise(cJSON_IsTrue(cwItem));
    cJSON_Delete(root);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

/**
 * GET/POST /api/calibration/coefficients - {"C0": <number>, "A": [KMAX+1
 * numbers], "B": [KMAX+1 numbers]}
 *
 * Reads/overwrites the AS5600 correction coefficients directly - see
 * RotatorHW::setAngleCalCoefficients(). Meant for the camera-referenced
 * offline calibration pipeline (scripts/camera_angle_analyze.py's
 * fit_fourier_against_reference() + scripts/upload_angle_calibration.py),
 * which fits against an independent reference instead of the on-device
 * calibrateAngleSensor() sweep's only available self-referential phase
 * basis. Expert-gated: this bypasses the normal on-device calibration flow
 * entirely and silently trusts whatever numbers arrive, unlike that flow's
 * own internally-consistent fit.
 */
static esp_err_t get_coefficients_handler(httpd_req_t *req)
{
    if (!expert_lock_guard(req)) return ESP_FAIL;

    auto &cfg = Configuration::getInstance();
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "C0", cfg.getC0());
    cJSON *arrA = cJSON_CreateArray();
    cJSON *arrB = cJSON_CreateArray();
    for (int k = 0; k <= KMAX; k++)
    {
        cJSON_AddItemToArray(arrA, cJSON_CreateNumber(cfg.getA(k)));
        cJSON_AddItemToArray(arrB, cJSON_CreateNumber(cfg.getB(k)));
    }
    cJSON_AddItemToObject(root, "A", arrA);
    cJSON_AddItemToObject(root, "B", arrB);
    const char *json = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json);
    cJSON_free((void *)json);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t set_coefficients_handler(httpd_req_t *req)
{
    if (!expert_lock_guard(req)) return ESP_FAIL;

    cJSON *root = nullptr;
    if (!parse_json_body(req, &root))
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    cJSON *c0Item = cJSON_GetObjectItem(root, "C0");
    cJSON *arrA = cJSON_GetObjectItem(root, "A");
    cJSON *arrB = cJSON_GetObjectItem(root, "B");
    if (!cJSON_IsNumber(c0Item) || !cJSON_IsArray(arrA) || !cJSON_IsArray(arrB) ||
        cJSON_GetArraySize(arrA) != KMAX + 1 || cJSON_GetArraySize(arrB) != KMAX + 1)
    {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Expected 'C0' (number) and 'A'/'B' (arrays of KMAX+1 numbers each)");
        return ESP_FAIL;
    }
    double a[KMAX + 1], b[KMAX + 1];
    for (int k = 0; k <= KMAX; k++)
    {
        cJSON *ai = cJSON_GetArrayItem(arrA, k);
        cJSON *bi = cJSON_GetArrayItem(arrB, k);
        if (!cJSON_IsNumber(ai) || !cJSON_IsNumber(bi))
        {
            cJSON_Delete(root);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "'A'/'B' must contain only numbers");
            return ESP_FAIL;
        }
        a[k] = ai->valuedouble;
        b[k] = bi->valuedouble;
    }
    double c0 = c0Item->valuedouble;
    cJSON_Delete(root);

    RotatorHW::getInstance().setAngleCalCoefficients(c0, a, b);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

/**
 * GET/POST /api/calibration/fullstep-table - {"table": [N_STEPS numbers]}
 *
 * Reads/overwrites the full-step-resolution residual correction layered on
 * top of C0/A/B - see RotatorHW::setFullStepTable(). Meant for
 * scripts/camera_fullstep_table.py's output, uploaded the same way as
 * /api/calibration/coefficients (same expert gate, same rationale: this
 * bypasses the normal on-device calibration flow and trusts whatever
 * numbers arrive).
 */
static esp_err_t get_fullstep_table_handler(httpd_req_t *req)
{
    if (!expert_lock_guard(req)) return ESP_FAIL;

    float table[N_STEPS];
    RotatorHW::getInstance().getFullStepTable(table);
    cJSON *root = cJSON_CreateObject();
    cJSON *arr = cJSON_CreateArray();
    for (int i = 0; i < N_STEPS; i++)
        cJSON_AddItemToArray(arr, cJSON_CreateNumber(table[i]));
    cJSON_AddItemToObject(root, "table", arr);
    const char *json = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json);
    cJSON_free((void *)json);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t set_fullstep_table_handler(httpd_req_t *req)
{
    if (!expert_lock_guard(req)) return ESP_FAIL;

    cJSON *root = nullptr;
    if (!parse_json_body(req, &root))
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    cJSON *arr = cJSON_GetObjectItem(root, "table");
    if (!cJSON_IsArray(arr) || cJSON_GetArraySize(arr) != N_STEPS)
    {
        cJSON_Delete(root);
        char msg[64];
        snprintf(msg, sizeof(msg), "Expected 'table' as an array of %d numbers", N_STEPS);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, msg);
        return ESP_FAIL;
    }
    float table[N_STEPS];
    for (int i = 0; i < N_STEPS; i++)
    {
        cJSON *item = cJSON_GetArrayItem(arr, i);
        if (!cJSON_IsNumber(item))
        {
            cJSON_Delete(root);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "'table' must contain only numbers");
            return ESP_FAIL;
        }
        table[i] = (float)item->valuedouble;
    }
    cJSON_Delete(root);

    RotatorHW::getInstance().setFullStepTable(table);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

/**
 * POST /api/position/goto - {"position": <degrees>}
 *
 * Manual goto for the Position tab's live-position card - drives the
 * rotator to an absolute Alpaca Position via RotatorHW::putAbsolutePosition(),
 * the same semantics (and cable-wrap motion limit) as Alpaca's own
 * MoveAbsolute. Ordinary operation, not expert-gated. Synchronous like every
 * other motion handler in this file: the response only arrives once the
 * move and its closed-loop refinement have finished.
 */
static esp_err_t position_goto_handler(httpd_req_t *req)
{
    cJSON *root = nullptr;
    if (!parse_json_body(req, &root))
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    cJSON *posItem = cJSON_GetObjectItem(root, "position");
    if (!cJSON_IsNumber(posItem))
    {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing 'position'");
        return ESP_FAIL;
    }
    double position = posItem->valuedouble;
    cJSON_Delete(root);

    bool ok = RotatorHW::getInstance().putAbsolutePosition(position);
    char buf[32];
    int len = snprintf(buf, sizeof(buf), "{\"ok\":%s}", ok ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, len);
    return ESP_OK;
}

static esp_err_t calibration_zero_stream(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/event-stream");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Connection", "keep-alive");

    char buf[64];
    // Helper, um SSE-Events zu senden und kurz zu yielden. Once the client is
    // gone, httpd_resp_send_chunk() on the dead socket doesn't just fail
    // quickly - each attempt blocks for a real send timeout (several hundred
    // ms, live-measured), so a lost browser tab can turn a ~30s calibration
    // into many minutes of the httpd worker retrying sends nobody reads. The
    // underlying measurement must keep running regardless (its NVS result
    // should still be good even if nobody's watching), so this only stops
    // trying to *send* once the socket is confirmed dead - it never aborts
    // the calibration itself.
    bool clientGone = false;
    auto send_event = [&](const char *evt, const char *data)
    {
        if (clientGone)
            return;
        int len = snprintf(buf, sizeof(buf),
                           "event: %s\ndata: %s\n\n", evt, data);
        if (httpd_resp_send_chunk(req, buf, len) != ESP_OK)
        {
            ESP_LOGW(TAG, "calibration/zero/stream: client gone, continuing without further progress events");
            clientGone = true;
            return;
        }
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
    // Persists to NVS (see RotatorHW::setZeroPosSensorValue()) so this
    // calibration survives a reboot without a firmware rebuild - and takes
    // effect immediately in memory, so gotoMechanicalZero() uses it on the
    // very next boot or on-demand /api/debug/goto-mechanical-zero call.
    RotatorHW::getInstance().setZeroPosSensorValue(zero);
    char dv[8];
    snprintf(dv, sizeof(dv), "%d", zero);
    send_event("complete_zero", dv);

    if (!clientGone)
        httpd_resp_send_chunk(req, nullptr, 0);
    return ESP_OK;
}

static esp_err_t calibration_angle_stream(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/event-stream");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Connection", "keep-alive");

    char buf[256];
    // See calibration_zero_stream()'s send_event() - same client-gone/send-
    // timeout hazard, same fix: stop sending once the socket is dead, but
    // let the (much longer) angle sweep keep running and persist its result.
    bool clientGone = false;
    auto send_event = [&](const char *evt, const char *data)
    {
        if (clientGone)
            return;
        int len = snprintf(buf, sizeof(buf),
                           "event: %s\ndata: %s\n\n", evt, data);
        if (httpd_resp_send_chunk(req, buf, len) != ESP_OK)
        {
            ESP_LOGW(TAG, "calibration/angle/stream: client gone, continuing without further progress events");
            clientGone = true;
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    };

    RotatorHW::CalibrationResult result = RotatorHW::getInstance().calibrateAngleSensor(
        [&](int pct)
        {
            char d[8];
            snprintf(d, sizeof(d), "%d", pct);
            send_event("progress", d);
        });
    char resultJson[128];
    snprintf(resultJson, sizeof(resultJson),
             "{\"residualBeforeDeg\":%.4f,\"residualAfterDeg\":%.4f,\"peakAfterDeg\":%.4f}",
             result.residualBeforeDeg, result.residualAfterDeg, result.peakAfterDeg);
    send_event("complete_angle", resultJson);

    if (!clientGone)
        httpd_resp_send_chunk(req, nullptr, 0);
    return ESP_OK;
}

/**
 * POST /api/debug/jog - {"microsteps": <signed int, default 0>, "samples": <int, default 1>}
 *
 * For offline calibration/filter development against an external script
 * (e.g. Python) instead of a firmware rebuild+flash+wait cycle per
 * iteration: jogs the motor by a raw, signed microstep count - bypassing
 * every degree/offset conversion the normal Alpaca motion API applies - and
 * returns a sensor/position snapshot. "samples" controls the AS5600 read
 * averaging depth: 1 (default) is a fast single sample, higher trades speed
 * for precision via the same averaging calibration uses.
 *
 * Expert-gated like the rest of this file's raw hardware access: unlike the
 * Alpaca API, there is no sanity checking here beyond a generous jog-distance
 * clamp, so this is meant for a developer driving it deliberately, not for
 * routine use.
 */
static esp_err_t debug_jog_handler(httpd_req_t *req)
{
    if (!expert_lock_guard(req)) return ESP_FAIL;

    cJSON *root = nullptr;
    if (!parse_json_body(req, &root))
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    cJSON *msItem = cJSON_GetObjectItem(root, "microsteps");
    cJSON *samplesItem = cJSON_GetObjectItem(root, "samples");
    double microstepsRaw = cJSON_IsNumber(msItem) ? msItem->valuedouble : 0;
    int samples = cJSON_IsNumber(samplesItem) ? (int)samplesItem->valuedouble : 1;
    cJSON_Delete(root);

    // About two full motor revolutions (400 fullsteps x 256 microsteps each,
    // per RotatorHW.cpp's FULLSTEPS_PER_ROTATION/MICROSTEPS - not visible
    // here, they are file-local to RotatorHW.cpp) - generous for a single
    // calibration step, but bounded so a mistaken huge value can't block
    // this HTTP server task for long or spin the motor unexpectedly far.
    constexpr double JOG_LIMIT = 2.0 * 400 * 256;
    if (microstepsRaw < -JOG_LIMIT || microstepsRaw > JOG_LIMIT)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "microsteps out of range");
        return ESP_FAIL;
    }

    auto &rotator = RotatorHW::getInstance();
    rotator.jogMicrosteps((int32_t)microstepsRaw);
    double raw = rotator.measureRawAngle(samples);
    auto snapshot = rotator.getSensorSnapshot();

    char buf[160];
    int len = snprintf(buf, sizeof(buf),
        "{\"rawSensor\":%.3f,\"correctedSensor\":%.3f,\"stepPosition\":%ld,\"hall\":%s}",
        raw, rotator.correctSensorReading(raw), (long)snapshot.stepPosition,
        snapshot.hall ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, len);
    return ESP_OK;
}

/**
 * POST /api/debug/align-fullstep
 *
 * A stepper only has FULLSTEPS_PER_ROTATION true mechanical equilibrium
 * positions - any within-full-step analysis over /api/debug/jog's raw
 * microsteps needs a stepPosition value known to actually sit on one of
 * them, not just assumed to. This drives the motor to a real full-step
 * position (main/RotatorHW.cpp's RotatorHW::alignToFullStep()) and returns
 * it as that reference. Expert-gated like the rest of this file's raw
 * hardware access.
 */
static esp_err_t debug_align_fullstep_handler(httpd_req_t *req)
{
    if (!expert_lock_guard(req)) return ESP_FAIL;

    int32_t pos = RotatorHW::getInstance().alignToFullStep();
    char buf[48];
    int len = snprintf(buf, sizeof(buf), "{\"stepPosition\":%ld}", (long)pos);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, len);
    return ESP_OK;
}

/**
 * GET /api/debug/sensor-diagnostics
 *
 * AS5600 magnet/airgap health straight from the chip's own AGC loop - see
 * RotatorHW::getSensorDiagnostics(). Read-only, no motion, expert-gated like
 * the rest of this file's raw hardware access.
 */
static esp_err_t debug_sensor_diagnostics_handler(httpd_req_t *req)
{
    if (!expert_lock_guard(req)) return ESP_FAIL;

    auto diag = RotatorHW::getInstance().getSensorDiagnostics();
    char buf[192];
    int len = snprintf(buf, sizeof(buf),
        "{\"agc\":%u,\"magnitude\":%u,\"magnetDetected\":%s,\"magnetTooStrong\":%s,\"magnetTooWeak\":%s}",
        diag.agc, diag.magnitude,
        diag.magnetDetected ? "true" : "false",
        diag.magnetTooStrong ? "true" : "false",
        diag.magnetTooWeak ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, len);
    return ESP_OK;
}

/**
 * POST /api/debug/goto-mechanical-zero
 *
 * Runs RotatorHW::gotoMechanicalZero() - the same homing routine main.cpp
 * runs once at boot - on demand, so its repeatability can be measured
 * without a reboot per attempt (see scripts/homing_repeatability.py).
 * Synchronous like every other motion handler in this file: the response
 * only arrives once homing (Hall search + the routine's own ~10s settle
 * delay + microstep edge search) has finished, so callers need a generous
 * timeout. Expert-gated and real motion, like the rest of this file's raw
 * hardware access.
 */
static esp_err_t debug_goto_mechanical_zero_handler(httpd_req_t *req)
{
    if (!expert_lock_guard(req)) return ESP_FAIL;

    RotatorHW::getInstance().gotoMechanicalZero();
    auto snapshot = RotatorHW::getInstance().getSensorSnapshot();
    char buf[128];
    int len = snprintf(buf, sizeof(buf),
        "{\"stepPosition\":%ld,\"correctedSensor\":%.3f,\"hall\":%s}",
        (long)snapshot.stepPosition, snapshot.correctedSensor,
        snapshot.hall ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, len);
    return ESP_OK;
}

/**
 * POST /api/debug/stress-fullstep-i2c - {"steps": <int, default 2000>,
 * "sampleCount": <int, default 4>}
 *
 * Deliberately aggressive reproduction attempt for the still-open
 * calibrateAngleSensor() hang (memory/rotator_angle_cal_hang.md) - see
 * RotatorHW::stressFullStepI2C(). SSE progress, same client-gone/send-
 * timeout fix as the calibration streams, since this is meant to run
 * unattended for a while. Expert-gated and real motion, like the rest of
 * this file's raw hardware access.
 */
static esp_err_t debug_stress_fullstep_i2c_handler(httpd_req_t *req)
{
    if (!expert_lock_guard(req)) return ESP_FAIL;

    cJSON *root = nullptr;
    if (!parse_json_body(req, &root))
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    cJSON *stepsItem = cJSON_GetObjectItem(root, "steps");
    cJSON *sampleCountItem = cJSON_GetObjectItem(root, "sampleCount");
    int32_t steps = cJSON_IsNumber(stepsItem) ? (int32_t)stepsItem->valuedouble : 2000;
    int sampleCount = cJSON_IsNumber(sampleCountItem) ? (int)sampleCountItem->valuedouble : 4;
    cJSON_Delete(root);
    if (steps < 1 || steps > 20000 || sampleCount < 1 || sampleCount > 64)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "steps or sampleCount out of range");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "text/event-stream");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Connection", "keep-alive");

    char buf[64];
    bool clientGone = false;
    auto send_event = [&](const char *evt, const char *data)
    {
        if (clientGone)
            return;
        int len = snprintf(buf, sizeof(buf), "event: %s\ndata: %s\n\n", evt, data);
        if (httpd_resp_send_chunk(req, buf, len) != ESP_OK)
        {
            ESP_LOGW(TAG, "debug/stress-fullstep-i2c: client gone, continuing without further progress events");
            clientGone = true;
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    };

    int32_t completed = RotatorHW::getInstance().stressFullStepI2C(
        steps, sampleCount,
        [&](int pct)
        {
            char d[8];
            snprintf(d, sizeof(d), "%d", pct);
            send_event("progress", d);
        });
    char resultJson[64];
    snprintf(resultJson, sizeof(resultJson), "{\"completed\":%ld,\"steps\":%ld}", (long)completed, (long)steps);
    send_event("complete_stress", resultJson);

    if (!clientGone)
        httpd_resp_send_chunk(req, nullptr, 0);
    return ESP_OK;
}

// WiFi Server
static esp_err_t wifi_status_handler(httpd_req_t *req)
{
    cJSON *root = cJSON_CreateObject();
    uint8_t mac[6] = {};
    char macbuf[18] = "—";
    if (wifi_manager_get_mac(mac) == ESP_OK) {
        snprintf(macbuf, sizeof(macbuf), "%02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    cJSON_AddBoolToObject(root, "connected", wifi_manager_is_connected());
    cJSON_AddStringToObject(root, "ssid", wifi_manager_get_ssid());
    cJSON_AddStringToObject(root, "ip", wifi_manager_get_ip());
    cJSON_AddNumberToObject(root, "rssi", wifi_manager_get_rssi());
    cJSON_AddStringToObject(root, "hostname", wifi_manager_get_hostname());
    cJSON_AddStringToObject(root, "mac", macbuf);
    const char *out = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    esp_err_t result = httpd_resp_sendstr(req, out);
    cJSON_free((void *)out);
    cJSON_Delete(root);
    return result;
}

static void restart_after_hostname(void *)
{
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
}

static esp_err_t wifi_hostname_handler(httpd_req_t *req)
{
    cJSON *root = nullptr;
    if (!parse_json_body(req, &root)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    cJSON *item = cJSON_GetObjectItem(root, "hostname");
    char stored[33] = {};
    esp_err_t error = cJSON_IsString(item)
        ? wifi_manager_set_hostname(item->valuestring, stored, sizeof(stored))
        : ESP_ERR_INVALID_ARG;
    cJSON_Delete(root);
    if (error != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, esp_err_to_name(error));
        return error;
    }
    char response[96];
    snprintf(response, sizeof(response), "{\"hostname\":\"%s\",\"restarting\":true}", stored);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, response);
    xTaskCreate(restart_after_hostname, "hostnameRestart", 2048, nullptr, 1, nullptr);
    return ESP_OK;
}

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

/**
 * Restarts the rotator on request.
 *
 * The Storage tab's NVS panel is this route's one caller, and the answer to
 * its own warning: an edit made straight into NVS only takes effect where
 * the firmware re-reads that value at boot, so anyone who wants to be sure
 * asks for a restart here rather than waiting for the next power cycle.
 */
static esp_err_t restart_handler(httpd_req_t *req)
{
    if (!expert_lock_guard(req)) return ESP_FAIL;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"restarting\":true}");
    xTaskCreate(restart_after_hostname, "restart", 2048, NULL, 1, NULL);
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

    httpd_uri_t nominal_direction_get = {
        .uri = "/api/calibration/nominal-direction",
        .method = HTTP_GET,
        .handler = get_nominal_direction_handler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &nominal_direction_get);

    httpd_uri_t nominal_direction_set = {
        .uri = "/api/calibration/nominal-direction",
        .method = HTTP_POST,
        .handler = set_nominal_direction_handler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &nominal_direction_set);

    httpd_uri_t position_goto = {
        .uri = "/api/position/goto",
        .method = HTTP_POST,
        .handler = position_goto_handler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &position_goto);

    httpd_uri_t coefficients_get = {
        .uri = "/api/calibration/coefficients",
        .method = HTTP_GET,
        .handler = get_coefficients_handler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &coefficients_get);

    httpd_uri_t coefficients_set = {
        .uri = "/api/calibration/coefficients",
        .method = HTTP_POST,
        .handler = set_coefficients_handler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &coefficients_set);

    httpd_uri_t fullstep_table_get = {
        .uri = "/api/calibration/fullstep-table",
        .method = HTTP_GET,
        .handler = get_fullstep_table_handler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &fullstep_table_get);

    httpd_uri_t fullstep_table_set = {
        .uri = "/api/calibration/fullstep-table",
        .method = HTTP_POST,
        .handler = set_fullstep_table_handler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &fullstep_table_set);

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

    httpd_uri_t wifi_status = {
        .uri = "/api/wifi/status",
        .method = HTTP_GET,
        .handler = wifi_status_handler};
    httpd_register_uri_handler(server, &wifi_status);

    httpd_uri_t wifi_hostname = {
        .uri = "/api/wifi/hostname",
        .method = HTTP_POST,
        .handler = wifi_hostname_handler};
    httpd_register_uri_handler(server, &wifi_hostname);

    httpd_uri_t restart = {
        .uri = "/restart",
        .method = HTTP_POST,
        .handler = restart_handler};
    httpd_register_uri_handler(server, &restart);

    httpd_uri_t debug_jog = {
        .uri = "/api/debug/jog",
        .method = HTTP_POST,
        .handler = debug_jog_handler};
    httpd_register_uri_handler(server, &debug_jog);

    httpd_uri_t debug_align = {
        .uri = "/api/debug/align-fullstep",
        .method = HTTP_POST,
        .handler = debug_align_fullstep_handler};
    httpd_register_uri_handler(server, &debug_align);

    httpd_uri_t debug_goto_zero = {
        .uri = "/api/debug/goto-mechanical-zero",
        .method = HTTP_POST,
        .handler = debug_goto_mechanical_zero_handler};
    httpd_register_uri_handler(server, &debug_goto_zero);

    httpd_uri_t debug_sensor_diagnostics = {
        .uri = "/api/debug/sensor-diagnostics",
        .method = HTTP_GET,
        .handler = debug_sensor_diagnostics_handler};
    httpd_register_uri_handler(server, &debug_sensor_diagnostics);

    httpd_uri_t debug_stress_fullstep_i2c = {
        .uri = "/api/debug/stress-fullstep-i2c",
        .method = HTTP_POST,
        .handler = debug_stress_fullstep_i2c_handler};
    httpd_register_uri_handler(server, &debug_stress_fullstep_i2c);

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
