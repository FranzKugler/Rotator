/**
 * LogBuffer
 * The rotator's own log, kept in RAM so the web UI's Debug tab can show it.
 *
 * The point of this is what happens *before* anyone is watching. The USB
 * serial console needs a cable plugged in and a terminal already open, and
 * the interesting lines are the ones from the first two seconds after a
 * restart, which nobody is ever in time for. A ring in RAM holds them until
 * the Debug tab is opened, which may be hours later.
 *
 * Everything in this firmware logs through the ESP_LOGx macros, and those -
 * this project's own calls as much as the WiFi/USB driver's - all go through
 * esp_log_set_vprintf(), so hooking that one function catches every source
 * at once. Nothing needs to be duplicated at each call site.
 *
 * What it cannot hold is anything from before log_buffer_init() runs: the
 * ROM bootloader, the second stage and the partition table have all had
 * their say before app_main() does.
 */
#include "LogBuffer.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ExpertLock.h"

#define LEVEL_VERBOSE 1
#define LEVEL_DEBUG 2
#define LEVEL_INFO 3
#define LEVEL_WARN 4
#define LEVEL_ERROR 5

typedef struct
{
    uint32_t ms;
    uint8_t level;
    char text[LOG_BUFFER_LINE_MAX];
} log_slot_t;

// In .bss on purpose: a ring that needed setting up first would start one
// step too late for the lines this exists to keep.
static log_slot_t s_slots[LOG_BUFFER_LINES];

// Counts up forever and never wraps in any life this rotator will have.
static volatile uint32_t s_seq_next = 0;

// Held for the length of a memcpy and nothing else, on the write side only -
// see store_line() for why the read side does not need it too.
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// What ESP-IDF printed before we took its vprintf over. Still called, so the
// USB serial console keeps seeing exactly what it always has.
static vprintf_like_t s_chained_vprintf = NULL;

static uint32_t oldest_seq(void)
{
    uint32_t next = s_seq_next;
    return next > LOG_BUFFER_LINES ? next - LOG_BUFFER_LINES : 0;
}

/**
 * Stores one line. Safe to call from either core.
 *
 * The read side (the /log handler) does not take s_mux at all: holding a
 * spinlock across a whole HTTP response would stall whichever core is
 * logging for as long as the response takes to build. Reading unlocked risks
 * a torn line if a write lands on the exact slot being read at the exact
 * moment - text is always null-terminated first and stays memory-safe - and
 * that is a better trade for a debug view than blocking the rotator over it.
 */
static void store_line(uint8_t level, const char *text, size_t length)
{
    if (length == 0) return;
    if (length > LOG_BUFFER_LINE_MAX - 1) length = LOG_BUFFER_LINE_MAX - 1;

    portENTER_CRITICAL_SAFE(&s_mux);
    log_slot_t *slot = &s_slots[s_seq_next % LOG_BUFFER_LINES];
    slot->ms = (uint32_t)(esp_timer_get_time() / 1000);
    slot->level = level;
    slot->text[length] = '\0';
    memcpy(slot->text, text, length);
    s_seq_next++;
    portEXIT_CRITICAL_SAFE(&s_mux);
}

/** ESP-IDF's level letter, kept as a field rather than parsed again in the browser. */
static uint8_t level_from_letter(char letter)
{
    switch (letter)
    {
        case 'E': return LEVEL_ERROR;
        case 'W': return LEVEL_WARN;
        case 'I': return LEVEL_INFO;
        case 'D': return LEVEL_DEBUG;
        case 'V': return LEVEL_VERBOSE;
        default:  return LEVEL_DEBUG;
    }
}

/**
 * Takes one ESP-IDF log line apart: strips the colour escapes it wraps every
 * line in, and reads the level off the letter that starts it.
 *
 * The format is `I (1234) wifi: ...`, optionally inside `\033[0;32m...
 * \033[0m`. Both halves are worth removing - the escapes would arrive in the
 * browser as visible rubbish, and the level is better as a field.
 */
static void store_idf_line(char *text)
{
    size_t out = 0;
    for (char *in = text; *in; in++)
    {
        if (*in == '\033')
        {
            while (*in && *in != 'm') in++;
            if (!*in) break;
            continue;
        }
        if (*in == '\r' || *in == '\n') continue;
        text[out++] = *in;
    }
    text[out] = '\0';
    if (out == 0) return;

    uint8_t level = LEVEL_DEBUG;
    size_t from = 0;
    if (out > 2 && text[1] == ' ' && text[2] == '(')
    {
        level = level_from_letter(text[0]);
        from = 2;
    }
    store_line(level, text + from, out - from);
}

/**
 * Stands in for ESP-IDF's vprintf. Formats once into a stack buffer, hands
 * it on unchanged, and keeps a copy.
 *
 * IDF calls this per format string, not per line, and a single call can
 * carry several lines or none - so the copy is split on newlines rather than
 * stored as it arrives.
 */
static int capture_vprintf(const char *format, va_list args)
{
    char buffer[LOG_BUFFER_LINE_MAX + 32];

    va_list forward;
    va_copy(forward, args);
    int written = vsnprintf(buffer, sizeof(buffer), format, args);
    int result = s_chained_vprintf ? s_chained_vprintf(format, forward) : written;
    va_end(forward);

    if (written > 0)
    {
        char *start = buffer;
        for (char *at = buffer; *at; at++)
        {
            if (*at != '\n') continue;
            *at = '\0';
            store_idf_line(start);
            start = at + 1;
        }
        // A trailing fragment with no newline yet, stored as its own line
        // rather than held back: IDF finishes its lines in one call.
        if (*start) store_idf_line(start);
    }

    return result;
}

void log_buffer_init(void)
{
    s_chained_vprintf = esp_log_set_vprintf(capture_vprintf);
}

/** Reset reasons, short enough for a table cell and specific enough to matter. */
static const char *reset_reason_name(void)
{
    switch (esp_reset_reason())
    {
        case ESP_RST_POWERON:   return "power-on";
        case ESP_RST_EXT:       return "external";
        case ESP_RST_SW:        return "software";
        case ESP_RST_PANIC:     return "panic";
        case ESP_RST_INT_WDT:   return "watchdog-int";
        case ESP_RST_TASK_WDT:  return "watchdog-task";
        case ESP_RST_WDT:       return "watchdog";
        case ESP_RST_BROWNOUT:  return "brownout";
        case ESP_RST_SDIO:      return "sdio";
        case ESP_RST_DEEPSLEEP: return "deep-sleep";
        case ESP_RST_USB:       return "usb";
        case ESP_RST_JTAG:      return "jtag";
        case ESP_RST_EFUSE:     return "efuse";
        case ESP_RST_PWR_GLITCH: return "power-glitch";
        case ESP_RST_CPU_LOCKUP: return "cpu-lockup";
        default:                return "unknown";
    }
}

/**
 * The rotator's log, and enough of its state to know what it was doing.
 *
 * Polled by the Debug tab with the sequence number it last saw, so the usual
 * answer carries nothing but the handful of lines that have appeared since.
 * A freshly opened tab asks with since=0 and gets the oldest batch first;
 * "more" then says whether another round is worth it.
 *
 * "oldest" says which line the ring still starts at, so the browser can tell
 * "nothing new" from "the ring wrapped and you missed 300 lines" instead of
 * silently showing a gap.
 */
static esp_err_t log_get_handler(httpd_req_t *req)
{
    if (!expert_lock_guard(req)) return ESP_FAIL;

    uint32_t since = 0;
    char query[32];
    if (httpd_req_get_url_query_len(req) > 0 &&
        httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK)
    {
        char value[16];
        if (httpd_query_key_value(query, "since", value, sizeof(value)) == ESP_OK)
        {
            since = (uint32_t)strtoul(value, NULL, 10);
        }
    }

    uint32_t next = s_seq_next;
    uint32_t oldest = oldest_seq();

    // Asking for something that has already scrolled out gives what is
    // left; asking for something that has not happened yet means the
    // rotator restarted under the browser, and the fix for both is the
    // same: start from what is actually still held.
    uint32_t from = since;
    if (from < oldest) from = oldest;
    if (from > next) from = oldest;

    cJSON *lines = cJSON_CreateArray();
    uint32_t seq = from;
    for (; seq < next && (seq - from) < LOG_BUFFER_BATCH; seq++)
    {
        const log_slot_t *slot = &s_slots[seq % LOG_BUFFER_LINES];
        cJSON *line = cJSON_CreateObject();
        cJSON_AddNumberToObject(line, "s", seq);
        cJSON_AddNumberToObject(line, "t", slot->ms);
        cJSON_AddNumberToObject(line, "l", slot->level);
        cJSON_AddStringToObject(line, "m", slot->text);
        cJSON_AddItemToArray(lines, line);
    }

    // What the caller should ask for next time. With nothing sent that is
    // where it already stood, clamped to the ring, so a restart hands back
    // a smaller number than was asked for instead of one that never comes.
    uint32_t served = since;
    if (seq > from)          served = seq;
    else if (served < oldest) served = oldest;
    else if (served > next)   served = next;

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "oldest", oldest);
    cJSON_AddNumberToObject(root, "seq", served);
    cJSON_AddBoolToObject(root, "more", served < next);
    cJSON_AddNumberToObject(root, "uptime", (double)(esp_timer_get_time() / 1000));
    cJSON_AddNumberToObject(root, "heap", esp_get_free_heap_size());
    cJSON_AddNumberToObject(root, "heapMin", esp_get_minimum_free_heap_size());
    cJSON_AddNumberToObject(root, "heapBlock", heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
    cJSON_AddStringToObject(root, "reset", reset_reason_name());
    cJSON_AddItemToObject(root, "lines", lines);

    char *text = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    esp_err_t result = text ? httpd_resp_sendstr(req, text) : ESP_ERR_NO_MEM;
    free(text);
    cJSON_Delete(root);
    return result;
}

esp_err_t log_buffer_register_routes(httpd_handle_t server)
{
    httpd_uri_t route = {.uri = "/log", .method = HTTP_GET, .handler = log_get_handler, .user_ctx = NULL};
    return httpd_register_uri_handler(server, &route);
}
