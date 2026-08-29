/**
 * NvsRoutes
 * NVS, shown as if it were a filesystem: namespaces as folders, keys as
 * files.
 *
 * It is not one, and the pretence is deliberate rather than sloppy. NVS is a
 * flat key-value store - a namespace, a key, a typed value - with no tree,
 * no paths and no file names. But this project's own records are JSON
 * strings, one per key, and once that is true a namespace reads exactly
 * like a folder of `.json` files. Giving the two stores the same shape
 * means one explorer, one set of gestures, and one place to look when
 * something has gone missing.
 *
 * Where the pretence stops is written down here so it is not discovered by
 * surprise:
 *
 *  - **The tree is two levels deep and cannot be deeper.** There are no
 *    sub-namespaces. A folder inside a folder is not refused, it is
 *    impossible.
 *  - **There is nothing to upload and no folder to create.** A namespace
 *    comes into existence when a key is written into it and vanishes with
 *    the last one, so both would be writing a key by another name.
 *  - **The extension is a reading, not a fact.** A string that starts with
 *    `{` or `[` is offered as `.json` because that is what every record this
 *    project writes looks like, another string as `.txt`, everything else -
 *    integers, blobs - as `.bin`. Nothing in NVS says so.
 *  - **Sizes are in entries, not bytes.** NVS accounts for itself in 32-byte
 *    entries and reports them through nvs_get_stats(), so that is what the
 *    fullness bar shows.
 *
 * Two values are deliberately not readable: the expert password hash and
 * salt, and the WiFi password. Everything else here is as open as the
 * unlock that reached it - but a hash carried away in those thirty seconds
 * is crackable offline forever, and the WiFi password is exactly the kind
 * of thing "probably used elsewhere too" was written about. The keys are
 * still listed, because a tree that hides entries is a tree that lies; only
 * the read is refused.
 */
#include "NvsRoutes.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "ExpertLock.h"

static const char *TAG = "nvsroutes";

// The default NVS partition, which is the only one this build has.
#define NVS_PARTITION "nvs"

static bool is_protected(const char *ns, const char *key)
{
    if (strcmp(ns, "expert") == 0 && (strcmp(key, "hash") == 0 || strcmp(key, "salt") == 0))
        return true;
    // The rotator's own WiFi credentials, written in WebServer.cpp's
    // /api/wifi/connect handler - not a value expert mode should be able to
    // carry off any more than the password that unlocked it.
    if (strcmp(ns, "wifi") == 0 && strcmp(key, "password") == 0)
        return true;
    return false;
}

static esp_err_t send_error(httpd_req_t *req, int code, const char *what)
{
    httpd_resp_set_status(req, code == 404 ? "404 Not Found" : code == 400 ? "400 Bad Request"
                              : code == 403 ? "403 Forbidden" : code == 413 ? "413 Payload Too Large"
                              : "500 Internal Server Error");
    httpd_resp_set_type(req, "application/json");
    char body[64];
    snprintf(body, sizeof(body), "{\"error\":\"%s\"}", what);
    httpd_resp_sendstr(req, body);
    return ESP_FAIL;
}

/** The type as the browser sees it, and as this file talks about it. */
static const char *type_name(nvs_type_t type)
{
    switch (type)
    {
        case NVS_TYPE_U8:   return "u8";
        case NVS_TYPE_I8:   return "i8";
        case NVS_TYPE_U16:  return "u16";
        case NVS_TYPE_I16:  return "i16";
        case NVS_TYPE_U32:  return "u32";
        case NVS_TYPE_I32:  return "i32";
        case NVS_TYPE_U64:  return "u64";
        case NVS_TYPE_I64:  return "i64";
        case NVS_TYPE_STR:  return "str";
        case NVS_TYPE_BLOB: return "blob";
        default:            return "?";
    }
}

static bool is_integer(nvs_type_t type)
{
    switch (type)
    {
        case NVS_TYPE_U8: case NVS_TYPE_I8:
        case NVS_TYPE_U16: case NVS_TYPE_I16:
        case NVS_TYPE_U32: case NVS_TYPE_I32:
        case NVS_TYPE_U64: case NVS_TYPE_I64: return true;
        default: return false;
    }
}

/**
 * One value as text, whatever it is underneath.
 *
 * Strings come out as they are, integers as decimal, and a blob is refused -
 * it has no text form, which is what makes it a download rather than an
 * edit. `type` is filled in either way, so a caller that only wanted to
 * know the shape gets it. Returns a malloc'd string in `*out` on success -
 * the caller frees it.
 */
static bool read_value(const char *ns, const char *key, char **out,
                       nvs_type_t *type, size_t *length, const char **error)
{
    nvs_handle_t handle;
    if (nvs_open_from_partition(NVS_PARTITION, ns, NVS_READONLY, &handle) != ESP_OK)
    {
        *error = "nvsNamespace";
        return false;
    }

    size_t size = 0;
    bool ok = false;
    *out = NULL;

    if (nvs_get_str(handle, key, NULL, &size) == ESP_OK)
    {
        *type = NVS_TYPE_STR;
        *length = size > 0 ? size - 1 : 0; // NVS counts the terminator
        if (*length > NVS_EDIT_MAX) { *error = "nvsTooBig"; }
        else
        {
            char *buffer = malloc(size);
            if (!buffer) { *error = "nvsMemory"; }
            else if (nvs_get_str(handle, key, buffer, &size) == ESP_OK) { *out = buffer; ok = true; }
            else { free(buffer); *error = "nvsRead"; }
        }
    }
    else if (nvs_get_blob(handle, key, NULL, &size) == ESP_OK)
    {
        *type = NVS_TYPE_BLOB;
        *length = size;
        *error = "nvsBinary";
    }
    else
    {
        // The integer widths, in the order that costs least to get wrong: a
        // narrower read of a wider value fails, so asking narrow first and
        // widening finds the one the store actually holds.
        int64_t value = 0;
        uint64_t unsigned_value = 0;
        uint8_t u8; int8_t i8; uint16_t u16; int16_t i16;
        uint32_t u32; int32_t i32; uint64_t u64; int64_t i64;

        if      (nvs_get_u8(handle, key, &u8)   == ESP_OK) { *type = NVS_TYPE_U8;  unsigned_value = u8;  ok = true; }
        else if (nvs_get_i8(handle, key, &i8)   == ESP_OK) { *type = NVS_TYPE_I8;  value = i8;  ok = true; }
        else if (nvs_get_u16(handle, key, &u16) == ESP_OK) { *type = NVS_TYPE_U16; unsigned_value = u16; ok = true; }
        else if (nvs_get_i16(handle, key, &i16) == ESP_OK) { *type = NVS_TYPE_I16; value = i16; ok = true; }
        else if (nvs_get_u32(handle, key, &u32) == ESP_OK) { *type = NVS_TYPE_U32; unsigned_value = u32; ok = true; }
        else if (nvs_get_i32(handle, key, &i32) == ESP_OK) { *type = NVS_TYPE_I32; value = i32; ok = true; }
        else if (nvs_get_u64(handle, key, &u64) == ESP_OK) { *type = NVS_TYPE_U64; unsigned_value = u64; ok = true; }
        else if (nvs_get_i64(handle, key, &i64) == ESP_OK) { *type = NVS_TYPE_I64; value = i64; ok = true; }
        else *error = "nvsNotFound";

        if (ok)
        {
            char text[24];
            if (*type == NVS_TYPE_U8 || *type == NVS_TYPE_U16 || *type == NVS_TYPE_U32 || *type == NVS_TYPE_U64)
                snprintf(text, sizeof(text), "%llu", (unsigned long long)unsigned_value);
            else
                snprintf(text, sizeof(text), "%lld", (long long)value);
            *out = strdup(text);
            *length = strlen(*out);
        }
    }

    nvs_close(handle);
    return ok;
}

/**
 * The suffix this module is willing to put on a value.
 *
 * An opinion, not a fact - nothing in NVS records a file name. A string
 * that begins with a brace or a bracket is offered as `.json` because that
 * is what every record this project writes looks like; anything else is
 * `.txt` or, having no text form at all, `.bin`.
 */
static const char *suffix_for(nvs_type_t type, const char *text, size_t length)
{
    if (type == NVS_TYPE_BLOB) return "bin";
    if (is_integer(type)) return "txt";
    if (length > NVS_PEEK_MAX) return "bin"; // not read, so not judged

    for (const char *c = text; *c; c++)
    {
        if (*c == ' ' || *c == '\t' || *c == '\r' || *c == '\n') continue;
        return (*c == '{' || *c == '[') ? "json" : "txt";
    }
    return "txt";
}

/**
 * Everything in NVS, in one answer.
 *
 * One request rather than one per namespace, unlike the LittleFS tree:
 * there are a few dozen entries on this rotator and the iterator walks the
 * whole partition anyway, so splitting it would mean walking it once per
 * folder. The browser groups by namespace itself.
 */
static esp_err_t send_list(httpd_req_t *req)
{
    if (!expert_lock_guard(req)) return ESP_FAIL;

    cJSON *root = cJSON_CreateObject();
    cJSON *entries = cJSON_CreateArray();
    cJSON_AddItemToObject(root, "entries", entries);

    nvs_iterator_t iterator = NULL;
    esp_err_t found = nvs_entry_find(NVS_PARTITION, NULL, NVS_TYPE_ANY, &iterator);
    int count = 0;

    while (found == ESP_OK && iterator != NULL)
    {
        if (count++ >= NVS_LIST_MAX) { cJSON_AddBoolToObject(root, "truncated", true); break; }

        nvs_entry_info_t info;
        nvs_entry_info(iterator, &info);

        cJSON *item = cJSON_CreateObject();
        cJSON_AddStringToObject(item, "ns", info.namespace_name);
        cJSON_AddStringToObject(item, "key", info.key);
        cJSON_AddStringToObject(item, "type", type_name(info.type));

        if (is_protected(info.namespace_name, info.key))
        {
            // Listed, and that is all. See the note at the top of the file.
            cJSON_AddStringToObject(item, "suffix", "bin");
            cJSON_AddBoolToObject(item, "protected", true);
            cJSON_AddNumberToObject(item, "size", 0);
        }
        else
        {
            char *text = NULL;
            nvs_type_t type = info.type;
            size_t length = 0;
            const char *error = NULL;
            bool ok = read_value(info.namespace_name, info.key, &text, &type, &length, &error);

            cJSON_AddNumberToObject(item, "size", (double)length);
            cJSON_AddStringToObject(item, "suffix", suffix_for(type, ok ? text : "", length));
            // Only a string short enough to hold twice can be edited in place.
            cJSON_AddBoolToObject(item, "edit", ok && type != NVS_TYPE_BLOB && length <= NVS_EDIT_MAX);
            free(text);
        }

        cJSON_AddItemToArray(entries, item);
        found = nvs_entry_next(&iterator);
    }
    nvs_release_iterator(iterator);

    // NVS accounts for itself in 32-byte entries, so that is what the
    // fullness bar is given. Inventing a byte count would be worse than an
    // odd unit.
    nvs_stats_t stats;
    if (nvs_get_stats(NVS_PARTITION, &stats) == ESP_OK)
    {
        cJSON_AddNumberToObject(root, "used", (double)stats.used_entries);
        cJSON_AddNumberToObject(root, "total", (double)stats.total_entries);
        cJSON_AddNumberToObject(root, "namespaces", (double)stats.namespace_count);
    }
    cJSON_AddNumberToObject(root, "editMax", NVS_EDIT_MAX);

    char *text = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    esp_err_t result = text ? httpd_resp_sendstr(req, text) : ESP_ERR_NO_MEM;
    free(text);
    cJSON_Delete(root);
    return result;
}

/**
 * Decodes a query-string value in place. See FileRoutes.c's url_decode() for
 * why this is needed at all - esp_http_server never does it on its own, and
 * a namespace or key containing anything encodeURIComponent() would escape
 * arrives here still escaped otherwise.
 */
static void url_decode(char *s)
{
    char *out = s;
    while (*s)
    {
        if (s[0] == '%' && isxdigit((unsigned char)s[1]) && isxdigit((unsigned char)s[2]))
        {
            char hex[3] = {s[1], s[2], 0};
            *out++ = (char)strtol(hex, NULL, 16);
            s += 3;
        }
        else if (*s == '+')
        {
            *out++ = ' ';
            s++;
        }
        else
        {
            *out++ = *s++;
        }
    }
    *out = '\0';
}

static bool query_arg(httpd_req_t *req, const char *name, char *out, size_t out_size)
{
    char query[128];
    out[0] = '\0';
    if (httpd_req_get_url_query_len(req) == 0 ||
        httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) return false;
    if (httpd_query_key_value(query, name, out, out_size) != ESP_OK || out[0] == '\0') return false;
    url_decode(out);
    return true;
}

/**
 * A blob, as the bytes it is.
 *
 * The one entry kind with no text form, so it is always a download and
 * never an edit - which is exactly what `.bin` in the tree is telling the
 * reader. Held whole in the heap rather than streamed: NVS has no partial
 * read, so there is nothing to stream from.
 */
static esp_err_t send_blob(httpd_req_t *req, const char *ns, const char *key, size_t length)
{
    if (length > NVS_EDIT_MAX) return send_error(req, 413, "nvsTooBig");

    nvs_handle_t handle;
    if (nvs_open_from_partition(NVS_PARTITION, ns, NVS_READONLY, &handle) != ESP_OK)
        return send_error(req, 404, "nvsNamespace");

    uint8_t *buffer = malloc(length ? length : 1);
    if (!buffer) { nvs_close(handle); return send_error(req, 500, "nvsMemory"); }

    size_t size = length;
    esp_err_t result = nvs_get_blob(handle, key, buffer, &size);
    nvs_close(handle);

    if (result != ESP_OK) { free(buffer); return send_error(req, 500, "nvsRead"); }

    char header[80];
    snprintf(header, sizeof(header), "attachment; filename=\"%s.bin\"", key);
    httpd_resp_set_hdr(req, "Content-Disposition", header);
    httpd_resp_set_type(req, "application/octet-stream");
    esp_err_t sent = httpd_resp_send(req, (const char *)buffer, size);
    free(buffer);
    return sent;
}

/** One value, as text where it has one and as bytes where it does not. */
static esp_err_t send_read(httpd_req_t *req)
{
    if (!expert_lock_guard(req)) return ESP_FAIL;

    char ns[NVS_KEY_NAME_MAX_SIZE], key[NVS_KEY_NAME_MAX_SIZE];
    if (!query_arg(req, "ns", ns, sizeof(ns)) || !query_arg(req, "key", key, sizeof(key)))
        return send_error(req, 400, "nvsPath");

    if (is_protected(ns, key)) return send_error(req, 403, "nvsProtected");

    char *text = NULL;
    nvs_type_t type = NVS_TYPE_ANY;
    size_t length = 0;
    const char *error = NULL;

    if (!read_value(ns, key, &text, &type, &length, &error))
    {
        // A blob has no text form, so it is offered as a download instead of
        // being refused outright - which is what `.bin` in the tree means.
        if (error && strcmp(error, "nvsBinary") == 0) return send_blob(req, ns, key, length);
        return send_error(req, 404, error ? error : "nvsNotFound");
    }

    char download[8];
    if (query_arg(req, "download", download, sizeof(download)))
    {
        char header[80];
        snprintf(header, sizeof(header), "attachment; filename=\"%s.%s\"", key, suffix_for(type, text, length));
        httpd_resp_set_hdr(req, "Content-Disposition", header);
        httpd_resp_set_type(req, "application/octet-stream");
    }
    else
    {
        httpd_resp_set_type(req, strcmp(suffix_for(type, text, length), "json") == 0 ? "application/json" : "text/plain");
    }
    esp_err_t result = httpd_resp_sendstr(req, text);
    free(text);
    return result;
}

static bool read_json_body(httpd_req_t *req, cJSON **root_out)
{
    if (req->content_len <= 0 || req->content_len > NVS_EDIT_MAX + 512) return false;
    char *body = malloc(req->content_len + 1);
    if (!body) return false;
    int received = httpd_req_recv(req, body, req->content_len);
    if (received <= 0) { free(body); return false; }
    body[received] = '\0';
    *root_out = cJSON_Parse(body);
    free(body);
    return *root_out != NULL;
}

static const char *json_string(cJSON *root, const char *field)
{
    cJSON *item = cJSON_GetObjectItem(root, field);
    return cJSON_IsString(item) ? item->valuestring : "";
}

/** Writes a value back, keeping the type the store already has for the key. */
static esp_err_t save_value(httpd_req_t *req)
{
    if (!expert_lock_guard(req)) return ESP_FAIL;

    cJSON *root = NULL;
    if (!read_json_body(req, &root)) return send_error(req, 400, "nvsBody");

    const char *ns = json_string(root, "ns");
    const char *key = json_string(root, "key");
    const char *content = json_string(root, "content");
    size_t content_len = strlen(content);

    if (ns[0] == '\0' || key[0] == '\0') { cJSON_Delete(root); return send_error(req, 400, "nvsPath"); }
    if (is_protected(ns, key)) { cJSON_Delete(root); return send_error(req, 403, "nvsProtected"); }
    if (content_len > NVS_EDIT_MAX) { cJSON_Delete(root); return send_error(req, 413, "nvsTooBig"); }

    // What is there now decides what goes back. Writing a string over an
    // integer would change the type of a key the firmware then reads with
    // nvs_get_u8 and finds missing - a setting that silently reverts to its
    // default, which is the worst way for this to go wrong.
    char *before = NULL;
    nvs_type_t type = NVS_TYPE_ANY;
    size_t length = 0;
    const char *error = NULL;
    if (!read_value(ns, key, &before, &type, &length, &error))
    {
        // A blob was found and has no text form, which is a different answer
        // from "there is no such key" and deserves a different status.
        const char *code = error ? error : "nvsNotFound";
        int status = strcmp(code, "nvsNotFound") == 0 ? 404 : 400;
        cJSON_Delete(root);
        return send_error(req, status, code);
    }
    free(before);

    nvs_handle_t handle;
    if (nvs_open_from_partition(NVS_PARTITION, ns, NVS_READWRITE, &handle) != ESP_OK)
    {
        cJSON_Delete(root);
        return send_error(req, 500, "nvsNamespace");
    }

    esp_err_t result = ESP_FAIL;
    if (type == NVS_TYPE_STR)
    {
        result = nvs_set_str(handle, key, content);
    }
    else if (is_integer(type))
    {
        char *end = NULL;
        long long value = strtoll(content, &end, 10);
        while (end && (*end == ' ' || *end == '\r' || *end == '\n')) end++;
        if (!end || *end != '\0')
        {
            nvs_close(handle);
            cJSON_Delete(root);
            return send_error(req, 400, "nvsNotANumber");
        }
        switch (type)
        {
            case NVS_TYPE_U8:  result = nvs_set_u8(handle, key, (uint8_t)value); break;
            case NVS_TYPE_I8:  result = nvs_set_i8(handle, key, (int8_t)value); break;
            case NVS_TYPE_U16: result = nvs_set_u16(handle, key, (uint16_t)value); break;
            case NVS_TYPE_I16: result = nvs_set_i16(handle, key, (int16_t)value); break;
            case NVS_TYPE_U32: result = nvs_set_u32(handle, key, (uint32_t)value); break;
            case NVS_TYPE_I32: result = nvs_set_i32(handle, key, (int32_t)value); break;
            case NVS_TYPE_U64: result = nvs_set_u64(handle, key, (uint64_t)value); break;
            case NVS_TYPE_I64: result = nvs_set_i64(handle, key, (int64_t)value); break;
            default: break;
        }
    }
    else
    {
        nvs_close(handle);
        cJSON_Delete(root);
        return send_error(req, 400, "nvsBinary");
    }

    if (result == ESP_OK) result = nvs_commit(handle);
    nvs_close(handle);

    if (result != ESP_OK) { cJSON_Delete(root); return send_error(req, 500, "nvsWrite"); }

    ESP_LOGW(TAG, "%s/%s rewritten, %u bytes", ns, key, (unsigned)content_len);

    cJSON *answer = cJSON_CreateObject();
    cJSON_AddStringToObject(answer, "ns", ns);
    cJSON_AddStringToObject(answer, "key", key);
    cJSON_AddNumberToObject(answer, "size", (double)content_len);
    // Some of this project's own settings are cached in RAM and only written
    // back to NVS on their own save path, so an edit made here may need a
    // restart to be picked up cleanly - the browser says so.
    cJSON_AddBoolToObject(answer, "cached", true);
    cJSON_Delete(root);

    char *text = cJSON_PrintUnformatted(answer);
    httpd_resp_set_type(req, "application/json");
    esp_err_t sent = text ? httpd_resp_sendstr(req, text) : ESP_ERR_NO_MEM;
    free(text);
    cJSON_Delete(answer);
    return sent;
}

/** Erases one key. The namespace goes with the last key in it. */
static esp_err_t remove_key(httpd_req_t *req)
{
    if (!expert_lock_guard(req)) return ESP_FAIL;

    cJSON *root = NULL;
    if (!read_json_body(req, &root)) return send_error(req, 400, "nvsBody");

    const char *ns = json_string(root, "ns");
    const char *key = json_string(root, "key");
    if (ns[0] == '\0' || key[0] == '\0') { cJSON_Delete(root); return send_error(req, 400, "nvsPath"); }

    nvs_handle_t handle;
    if (nvs_open_from_partition(NVS_PARTITION, ns, NVS_READWRITE, &handle) != ESP_OK)
    {
        cJSON_Delete(root);
        return send_error(req, 404, "nvsNamespace");
    }

    esp_err_t result = nvs_erase_key(handle, key);
    if (result == ESP_OK) result = nvs_commit(handle);
    nvs_close(handle);

    if (result != ESP_OK) { cJSON_Delete(root); return send_error(req, 500, "nvsDelete"); }

    ESP_LOGW(TAG, "%s/%s erased", ns, key);

    cJSON *answer = cJSON_CreateObject();
    cJSON_AddStringToObject(answer, "ns", ns);
    cJSON_AddStringToObject(answer, "key", key);
    cJSON_Delete(root);

    char *text = cJSON_PrintUnformatted(answer);
    httpd_resp_set_type(req, "application/json");
    esp_err_t sent = text ? httpd_resp_sendstr(req, text) : ESP_ERR_NO_MEM;
    free(text);
    cJSON_Delete(answer);
    return sent;
}

static esp_err_t add_route(httpd_handle_t server, const char *uri, httpd_method_t method,
                           esp_err_t (*handler)(httpd_req_t *))
{
    httpd_uri_t route = {.uri = uri, .method = method, .handler = handler, .user_ctx = NULL};
    return httpd_register_uri_handler(server, &route);
}

esp_err_t nvs_routes_register(httpd_handle_t server)
{
    ESP_ERROR_CHECK(add_route(server, "/nvs/list", HTTP_GET, send_list));
    ESP_ERROR_CHECK(add_route(server, "/nvs/read", HTTP_GET, send_read));
    ESP_ERROR_CHECK(add_route(server, "/nvs/save", HTTP_POST, save_value));
    ESP_ERROR_CHECK(add_route(server, "/nvs/delete", HTTP_POST, remove_key));
    return ESP_OK;
}
