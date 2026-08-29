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
 * their say before app_main() does. It also cannot hold a panic's own
 * text - that goes straight to the UART through ESP-IDF's own ROM print
 * path, never through esp_log_set_vprintf() - but it can hold the tail of
 * ordinary logging that led up to one; see recover_rtc_tail() below.
 */
#include "LogBuffer.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_attr.h"
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

// A panic's own text (the exception cause, registers, backtrace) never
// reaches here - ESP-IDF's panic handler writes that straight to the UART
// with its own ROM print path, bypassing esp_log_set_vprintf entirely, and
// there is no safe way to hook that from application code. What *can* be
// recovered is the tail of ordinary logging that led up to it, which is
// usually enough to place the trigger even without the crash's own text.
//
// RTC slow memory survives a software reset or panic - only a true power
// cycle clears it - so a small mirror of the most recent lines kept there
// outlives exactly the resets that matter here. Nothing else on this
// project uses RTC memory (no ULP, no deep sleep), so this is the only
// tenant of it.
#define RTC_TAIL_LINES 32
#define RTC_TAIL_MAGIC 0x726f7461u // "rota"

typedef struct
{
    uint32_t magic;
    uint32_t count; // valid slots, saturating at RTC_TAIL_LINES
    uint32_t next;  // slot the next line will land in
    struct
    {
        uint32_t ms;
        uint8_t level;
        char text[LOG_BUFFER_LINE_MAX];
    } lines[RTC_TAIL_LINES];
} rtc_tail_t;

static RTC_NOINIT_ATTR rtc_tail_t s_rtc_tail;

/**
 * Writes into the RAM ring only - see store_line() for the RTC-mirrored
 * version. Clamps defensively rather than trusting every caller to have
 * done it already: recover_rtc_tail() prepends a "[before restart] " marker
 * before calling this, which can push a full-length stored line past what
 * this ring's LOG_BUFFER_LINE_MAX-sized slots hold.
 */
static void ring_store(uint8_t level, uint32_t ms, const char *text, size_t length)
{
    if (length > LOG_BUFFER_LINE_MAX - 1) length = LOG_BUFFER_LINE_MAX - 1;

    portENTER_CRITICAL_SAFE(&s_mux);
    log_slot_t *slot = &s_slots[s_seq_next % LOG_BUFFER_LINES];
    slot->ms = ms;
    slot->level = level;
    slot->text[length] = '\0';
    memcpy(slot->text, text, length);
    s_seq_next++;
    portEXIT_CRITICAL_SAFE(&s_mux);
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

    uint32_t ms = (uint32_t)(esp_timer_get_time() / 1000);
    ring_store(level, ms, text, length);

    // Uncontended outside of this call - only ever touched from here and
    // from the one-shot recovery in log_buffer_init(), which runs before
    // logging starts. No lock needed for the same reason store_line()'s
    // ring write does not need one on the read side.
    rtc_tail_t *tail = &s_rtc_tail;
    if (tail->magic != RTC_TAIL_MAGIC || tail->count > RTC_TAIL_LINES || tail->next >= RTC_TAIL_LINES)
    {
        tail->magic = RTC_TAIL_MAGIC;
        tail->count = 0;
        tail->next = 0;
    }
    tail->lines[tail->next].ms = ms;
    tail->lines[tail->next].level = level;
    memcpy(tail->lines[tail->next].text, text, length);
    tail->lines[tail->next].text[length] = '\0';
    tail->next = (tail->next + 1) % RTC_TAIL_LINES;
    if (tail->count < RTC_TAIL_LINES) tail->count++;
}

/**
 * Replays whatever the RTC tail holds from before this boot into the fresh
 * ring, oldest first, each line marked so it reads as history rather than
 * as something happening now. Called once, before esp_log_set_vprintf() is
 * hooked, so nothing can be logged - and so mirrored into the tail - while
 * this reads it.
 *
 * A garbage magic value means either the first boot ever or a true power
 * cycle, both of which mean there is nothing to recover; silently doing
 * nothing is the correct answer for both.
 */
static void recover_rtc_tail(void)
{
    rtc_tail_t *tail = &s_rtc_tail;
    if (tail->magic != RTC_TAIL_MAGIC || tail->count == 0 ||
        tail->count > RTC_TAIL_LINES || tail->next >= RTC_TAIL_LINES) return;

    bool wrapped = tail->count == RTC_TAIL_LINES;
    size_t start = wrapped ? tail->next : 0;
    for (size_t i = 0; i < tail->count; i++)
    {
        size_t idx = (start + i) % RTC_TAIL_LINES;
        char marked[LOG_BUFFER_LINE_MAX + 20];
        int written = snprintf(marked, sizeof(marked), "[before restart] %s", tail->lines[idx].text);
        size_t length = written < 0 ? 0 : (size_t)written;
        if (length >= sizeof(marked)) length = sizeof(marked) - 1;
        ring_store(tail->lines[idx].level, tail->lines[idx].ms, marked, length);
    }
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
    recover_rtc_tail();
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
