#include "CameraAngle.h"

#include "cJSON.h"
#include "esp_http_client.h"
#include "esp_log.h"

#include "lwip/netdb.h"
#include "lwip/sockets.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "camangle";

// Generous for a body that is a number or a small JSON object, and small
// enough that a misconfigured URL pointing at a web page cannot fill memory.
#define CAMERA_ANGLE_MAX_BODY 512

typedef struct
{
    char buffer[CAMERA_ANGLE_MAX_BODY + 1];
    int length;
} body_t;

static esp_err_t on_event(esp_http_client_event_t *event)
{
    if (event->event_id != HTTP_EVENT_ON_DATA)
        return ESP_OK;
    body_t *body = (body_t *)event->user_data;
    if (!body)
        return ESP_OK;
    int room = CAMERA_ANGLE_MAX_BODY - body->length;
    int take = event->data_len < room ? event->data_len : room;
    if (take > 0)
    {
        memcpy(body->buffer + body->length, event->data, take);
        body->length += take;
        body->buffer[body->length] = '\0';
    }
    return ESP_OK;
}

/**
 * Can we open a TCP connection to this URL's host within `timeoutMs`?
 *
 * esp_http_client's own timeout governs the transaction, not the connect,
 * and this build has CONFIG_LWIP_TCP_SYNMAXRTX=12 - twelve SYN
 * retransmissions with exponential backoff. Against an address that
 * silently drops packets, which is what a misconfigured angle source
 * usually is, a single connect then takes minutes rather than the fifteen
 * seconds asked for. Live on 2026-09-19 that turned a 164-position
 * calibration into hours during which the device answered nothing at all,
 * because ESP-IDF's httpd serves every socket from the one task this
 * handler is running in.
 *
 * So the connect is bounded here instead: non-blocking, select, done. Cheap
 * when the source is there, decisive when it is not.
 */
static bool host_reachable(const char *url, int timeoutMs)
{
    const char *rest = strstr(url, "://");
    rest = rest ? rest + 3 : url;
    char host[128];
    size_t n = strcspn(rest, ":/");
    if (n == 0 || n >= sizeof(host))
        return false;
    memcpy(host, rest, n);
    host[n] = '\0';
    char port[8] = "80";
    if (rest[n] == ':')
    {
        size_t m = strcspn(rest + n + 1, "/");
        if (m > 0 && m < sizeof(port))
        {
            memcpy(port, rest + n + 1, m);
            port[m] = '\0';
        }
    }

    struct addrinfo hints = {.ai_family = AF_INET, .ai_socktype = SOCK_STREAM};
    struct addrinfo *info = NULL;
    if (getaddrinfo(host, port, &hints, &info) != 0 || !info)
    {
        ESP_LOGW(TAG, "Cannot resolve %s", host);
        return false;
    }

    int fd = socket(info->ai_family, info->ai_socktype, info->ai_protocol);
    if (fd < 0)
    {
        freeaddrinfo(info);
        return false;
    }
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    bool ok = false;
    int result = connect(fd, info->ai_addr, info->ai_addrlen);
    if (result == 0)
    {
        ok = true;
    }
    else if (errno == EINPROGRESS)
    {
        fd_set writable;
        FD_ZERO(&writable);
        FD_SET(fd, &writable);
        struct timeval tv = {.tv_sec = timeoutMs / 1000, .tv_usec = (timeoutMs % 1000) * 1000};
        if (select(fd + 1, NULL, &writable, NULL, &tv) > 0)
        {
            int error = 0;
            socklen_t length = sizeof(error);
            ok = getsockopt(fd, SOL_SOCKET, SO_ERROR, &error, &length) == 0 && error == 0;
        }
    }
    close(fd);
    freeaddrinfo(info);
    if (!ok)
        ESP_LOGW(TAG, "No TCP connection to %s:%s within %d ms", host, port, timeoutMs);
    return ok;
}

bool camera_angle_read(const char *url, double *angleDeg, int timeoutMs)
{
    if (!url || !*url || !angleDeg)
        return false;

    // Bounded connect first - see host_reachable(). Three seconds is
    // generous for anything on the same network and short enough that five
    // failures in a row cost fifteen seconds rather than most of a day.
    if (!host_reachable(url, 3000))
        return false;

    body_t body = {.length = 0};
    body.buffer[0] = '\0';

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = timeoutMs,
        .event_handler = on_event,
        .user_data = &body,
        .disable_auto_redirect = false,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client)
        return false;

    esp_err_t error = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (error != ESP_OK)
    {
        ESP_LOGW(TAG, "GET %s failed: %s", url, esp_err_to_name(error));
        return false;
    }
    if (status != 200)
    {
        ESP_LOGW(TAG, "GET %s returned HTTP %d", url, status);
        return false;
    }
    if (body.length == 0)
    {
        ESP_LOGW(TAG, "GET %s returned an empty body", url);
        return false;
    }

    // JSON object with "angleDeg" first, then a bare number. strtod's
    // end-pointer check is what keeps a stray HTML page from parsing as 0.
    cJSON *root = cJSON_Parse(body.buffer);
    if (root)
    {
        cJSON *item = cJSON_GetObjectItem(root, "angleDeg");
        bool ok = cJSON_IsNumber(item);
        if (ok)
            *angleDeg = item->valuedouble;
        cJSON_Delete(root);
        if (ok)
            return true;
    }

    char *end = NULL;
    double value = strtod(body.buffer, &end);
    if (end == body.buffer)
    {
        ESP_LOGW(TAG, "GET %s returned something that is not an angle: %.40s", url, body.buffer);
        return false;
    }
    *angleDeg = value;
    return true;
}
