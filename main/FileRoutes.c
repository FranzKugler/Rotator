/**
 * FileRoutes
 * The rotator's filesystem, opened up to the Storage tab: a tree, a
 * download, an upload and a small editor.
 *
 * **This is LittleFS, not NVS.** The two are easy to confuse because both
 * survive a reboot and both are asked about in the same breath, but NVS is a
 * key-value store - see NvsRoutes.c - with no tree, no paths and nothing
 * that can be downloaded as a file. What has files is the LittleFS
 * partition, and that is what this serves: the web UI itself and the
 * rotator's own config.json.
 *
 * Which is also the warning. The partition this hands out write access to
 * is the one the page doing the asking is served from. Deleting index.html
 * leaves a rotator that still answers every API endpoint and shows nothing
 * in a browser, and the way back is a filesystem OTA upload. That is not a
 * reason to forbid it - somebody who opens a file explorer wants to change
 * files - but it is a reason for the confirmation in the browser.
 *
 * Everything here is behind expert mode, for the same reason /log is: this
 * is strictly the more powerful of the two.
 */
#include "FileRoutes.h"

#include <ctype.h>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "cJSON.h"
#include "esp_littlefs.h"
#include "esp_log.h"

#include "ExpertLock.h"

static const char *TAG = "fileroutes";
#define FS_BASE "/lfs"
#define FS_LABEL "littlefs"

// ------ paths ------

/**
 * Decodes a query-string value in place.
 *
 * httpd_query_key_value() hands back the raw bytes between `=` and the next
 * `&`, untouched - esp_http_server does no percent-decoding of its own. Every
 * path this project's own browser code sends goes through encodeURIComponent
 * first, which always escapes a slash as %2F, so a literal path arrives here
 * still escaped and safe_path() rejects it outright: not a `/`, not absolute,
 * error. Decoding in place is safe because the result is never longer than
 * the input. Done before safe_path() runs, not after - so a decoded ".."
 * still meets the dot-dot check, rather than sailing through as the harmless
 * looking literal string "%2e%2e".
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

/**
 * Reduces what arrived to a path this rotator will act on, or refuses it.
 *
 * The endpoints are reachable without the UI, so this is the only thing
 * standing between a typo and a write somewhere unintended. It is not a
 * sandbox - the whole volume is fair game by design - it is a check that
 * what was asked for is a path at all: absolute, no climbing with dot-dot,
 * no backslashes, no control characters, and short enough for LittleFS to
 * hold without truncating it into a different file.
 */
static bool safe_path(const char *raw, char *out, size_t out_size)
{
    size_t raw_len = strlen(raw);
    if (raw_len == 0 || raw[0] != '/') return false;
    if (raw_len > FS_PATH_MAX || raw_len + 1 > out_size) return false;

    size_t o = 0;
    for (size_t i = 0; i < raw_len; i++)
    {
        char c = raw[i];
        if (c == '\\' || (unsigned char)c < 0x20) return false;
        // Two slashes in a row are a typo, not a directory; collapse them
        // rather than creating a nameless one.
        if (c == '/' && o > 0 && out[o - 1] == '/') continue;
        out[o++] = c;
    }
    out[o] = '\0';

    // A trailing slash would make "/assets" and "/assets/" two different
    // strings for one directory.
    while (o > 1 && out[o - 1] == '/') out[--o] = '\0';
    if (o == 0) { out[0] = '/'; out[1] = '\0'; o = 1; }

    // Segment by segment, so a name that merely starts with dots is fine.
    size_t at = 1;
    while (at <= o)
    {
        char *slash = strchr(out + at, '/');
        size_t end = slash ? (size_t)(slash - out) : o;
        size_t len = end - at;
        if ((len == 1 && out[at] == '.') || (len == 2 && out[at] == '.' && out[at + 1] == '.'))
            return false;
        if (!slash) break;
        at = end + 1;
    }

    return true;
}

/** The path this VFS mount actually needs, given one already checked by safe_path(). */
static void full_path(const char *rel, char *out, size_t out_size)
{
    snprintf(out, out_size, "%s%s", FS_BASE, rel);
}

static esp_err_t send_error(httpd_req_t *req, httpd_err_code_t status_hint, int code, const char *what)
{
    char status[16];
    snprintf(status, sizeof(status), "%d", code);
    httpd_resp_set_status(req, code == 404 ? "404 Not Found" : code == 400 ? "400 Bad Request"
                              : code == 409 ? "409 Conflict" : code == 413 ? "413 Payload Too Large"
                              : code == 500 ? "500 Internal Server Error" : "507 Insufficient Storage");
    httpd_resp_set_type(req, "application/json");
    char body[64];
    snprintf(body, sizeof(body), "{\"error\":\"%s\"}", what);
    httpd_resp_sendstr(req, body);
    (void)status_hint;
    return ESP_FAIL;
}

/** The path of a request query argument, already checked. False means answered. */
static bool arg_path(httpd_req_t *req, char *out, size_t out_size)
{
    char query[FS_PATH_MAX + 32];
    char raw[FS_PATH_MAX + 8] = "/";
    if (httpd_req_get_url_query_len(req) > 0 &&
        httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK)
    {
        httpd_query_key_value(query, "path", raw, sizeof(raw));
    }
    url_decode(raw);
    if (safe_path(raw, out, out_size)) return true;
    send_error(req, HTTPD_400_BAD_REQUEST, 400, "fsPath");
    return false;
}

/** The path out of a JSON request body, checked and never the root. False: answered. */
static bool body_path(httpd_req_t *req, cJSON *root, char *out, size_t out_size)
{
    cJSON *item = cJSON_GetObjectItem(root, "path");
    const char *raw = cJSON_IsString(item) ? item->valuestring : "";
    if (!safe_path(raw, out, out_size) || strcmp(out, "/") == 0)
    {
        send_error(req, HTTPD_400_BAD_REQUEST, 400, "fsPath");
        return false;
    }
    return true;
}

static bool read_json_body(httpd_req_t *req, cJSON **root_out)
{
    if (req->content_len <= 0 || req->content_len > FS_EDIT_MAX + 512) return false;
    char *body = malloc(req->content_len + 1);
    if (!body) return false;
    int received = httpd_req_recv(req, body, req->content_len);
    if (received <= 0) { free(body); return false; }
    body[received] = '\0';
    *root_out = cJSON_Parse(body);
    free(body);
    return *root_out != NULL;
}

/** Content type from the extension. Unknown means "download it". */
static const char *mime_for(const char *path)
{
    size_t len = strlen(path);
    #define ENDS(ext) (len > strlen(ext) && strcmp(path + len - strlen(ext), ext) == 0)
    if (ENDS(".html") || ENDS(".htm")) return "text/html";
    if (ENDS(".css"))  return "text/css";
    if (ENDS(".js"))   return "application/javascript";
    if (ENDS(".json")) return "application/json";
    if (ENDS(".svg"))  return "image/svg+xml";
    if (ENDS(".png"))  return "image/png";
    if (ENDS(".ico"))  return "image/x-icon";
    if (ENDS(".txt"))  return "text/plain";
    if (ENDS(".gz"))   return "application/gzip";
    #undef ENDS
    return "application/octet-stream";
}

/**
 * Whether the browser should offer to edit this file.
 *
 * A guess, and it only decides which buttons are drawn - the editor route
 * checks the size for itself. By extension rather than by content: sniffing
 * a file means reading it, and this answer is wanted for every entry in a
 * listing.
 */
static bool editable(const char *name, size_t size)
{
    if (size > FS_EDIT_MAX) return false;
    static const char *exts[] = {".json", ".txt", ".css", ".html", ".htm", ".js", ".csv", ".md"};
    for (size_t i = 0; i < sizeof(exts) / sizeof(exts[0]); i++)
    {
        size_t elen = strlen(exts[i]), nlen = strlen(name);
        if (nlen > elen && strcmp(name + nlen - elen, exts[i]) == 0) return true;
    }
    return strchr(name, '.') == NULL; // no extension at all is usually a note
}

// ------ reading ------

/**
 * One directory, and how full the volume is.
 *
 * Per directory rather than the whole tree in one answer: the browser
 * expands a branch when it is opened, so a directory somebody filled with a
 * thousand files costs one slow response instead of making every response
 * slow.
 */
static esp_err_t send_list(httpd_req_t *req)
{
    if (!expert_lock_guard(req)) return ESP_FAIL;

    char path[FS_PATH_MAX + 1];
    if (!arg_path(req, path, sizeof(path))) return ESP_FAIL;

    char full[FS_PATH_MAX + 8];
    full_path(path, full, sizeof(full));

    struct stat root_stat;
    if (stat(full, &root_stat) != 0) return send_error(req, HTTPD_404_NOT_FOUND, 404, "fsNotFound");
    if (!S_ISDIR(root_stat.st_mode)) return send_error(req, HTTPD_400_BAD_REQUEST, 400, "fsNotDir");

    DIR *dir = opendir(full);
    if (!dir) return send_error(req, HTTPD_404_NOT_FOUND, 404, "fsNotFound");

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "path", path);
    size_t total = 0, used = 0;
    esp_littlefs_info(FS_LABEL, &total, &used);
    cJSON_AddNumberToObject(root, "total", (double)total);
    cJSON_AddNumberToObject(root, "used", (double)used);
    cJSON_AddNumberToObject(root, "editMax", FS_EDIT_MAX);
    cJSON *entries = cJSON_CreateArray();
    cJSON_AddItemToObject(root, "entries", entries);

    struct dirent *entry;
    int count = 0;
    while ((entry = readdir(dir)) != NULL)
    {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) continue;
        if (count++ >= FS_LIST_MAX) { cJSON_AddBoolToObject(root, "truncated", true); break; }

        // Sized for dirent's own d_name, not FS_PATH_MAX: the name came from
        // the filesystem, not through safe_path(), so nothing has bounded it
        // to that length yet - only LittleFS's own CONFIG_LITTLEFS_OBJ_NAME_LEN
        // does, and the compiler does not know that.
        char child[FS_PATH_MAX + 8 + sizeof(entry->d_name)];
        snprintf(child, sizeof(child), "%s/%s", full, entry->d_name);
        struct stat st;
        bool is_dir = stat(child, &st) == 0 && S_ISDIR(st.st_mode);
        size_t size = is_dir ? 0 : (size_t)st.st_size;

        cJSON *item = cJSON_CreateObject();
        cJSON_AddStringToObject(item, "name", entry->d_name);
        cJSON_AddBoolToObject(item, "dir", is_dir);
        cJSON_AddNumberToObject(item, "size", (double)size);
        if (!is_dir) cJSON_AddBoolToObject(item, "edit", editable(entry->d_name, size));
        cJSON_AddItemToArray(entries, item);
    }
    closedir(dir);

    char *text = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    esp_err_t result = text ? httpd_resp_sendstr(req, text) : ESP_ERR_NO_MEM;
    free(text);
    cJSON_Delete(root);
    return result;
}

/**
 * A file, as it is.
 *
 * Streamed, so downloading the whole web UI does not need the heap to hold
 * it. download=1 asks for the attachment header, which is the difference
 * between the browser saving index.html and the browser rendering it in
 * place.
 */
static esp_err_t send_read(httpd_req_t *req)
{
    if (!expert_lock_guard(req)) return ESP_FAIL;

    char path[FS_PATH_MAX + 1];
    if (!arg_path(req, path, sizeof(path))) return ESP_FAIL;

    char full[FS_PATH_MAX + 8];
    full_path(path, full, sizeof(full));

    struct stat st;
    if (stat(full, &st) != 0) return send_error(req, HTTPD_404_NOT_FOUND, 404, "fsNotFound");
    if (S_ISDIR(st.st_mode)) return send_error(req, HTTPD_400_BAD_REQUEST, 400, "fsIsDir");

    FILE *file = fopen(full, "rb");
    if (!file) return send_error(req, HTTPD_404_NOT_FOUND, 404, "fsNotFound");

    char query[16];
    bool download = httpd_req_get_url_query_len(req) > 0 &&
        httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        strstr(query, "download") != NULL;

    if (download)
    {
        const char *slash = strrchr(path, '/');
        const char *name = slash ? slash + 1 : path;
        char header[FS_PATH_MAX + 24];
        snprintf(header, sizeof(header), "attachment; filename=\"%s\"", name);
        httpd_resp_set_hdr(req, "Content-Disposition", header);
        httpd_resp_set_type(req, "application/octet-stream");
    }
    else
    {
        httpd_resp_set_type(req, mime_for(path));
    }

    char buffer[1024];
    size_t read_bytes;
    while ((read_bytes = fread(buffer, 1, sizeof(buffer), file)) > 0)
    {
        if (httpd_resp_send_chunk(req, buffer, read_bytes) != ESP_OK)
        {
            fclose(file);
            return ESP_FAIL;
        }
    }
    fclose(file);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

// ------ writing ------

/**
 * Streams the raw request body straight into a `.part` file, then renames it
 * into place. The rename is the moment the new file exists; until then the
 * old one is untouched, which is what makes replacing index.html survivable.
 *
 * Raw body rather than multipart: esp_http_server has no multipart parser,
 * and /ota/upload already established this project's own pattern for
 * streaming an upload straight to storage without holding it in the heap.
 */
static esp_err_t upload_handler(httpd_req_t *req)
{
    if (!expert_lock_guard(req)) return ESP_FAIL;

    char path[FS_PATH_MAX + 1];
    if (!arg_path(req, path, sizeof(path)) || strcmp(path, "/") == 0)
        return send_error(req, HTTPD_400_BAD_REQUEST, 400, "fsPath");
    if (req->content_len <= 0) return send_error(req, HTTPD_400_BAD_REQUEST, 400, "fsBody");

    char target[FS_PATH_MAX + 8], part[FS_PATH_MAX + 16];
    full_path(path, target, sizeof(target));
    snprintf(part, sizeof(part), "%s.part", target);
    if (strlen(part) >= sizeof(part) - 1) return send_error(req, HTTPD_400_BAD_REQUEST, 400, "fsPath");

    FILE *file = fopen(part, "wb");
    if (!file) return send_error(req, HTTPD_500_INTERNAL_SERVER_ERROR, 500, "fsOpen");

    ESP_LOGI(TAG, "receiving %s", path);
    char buffer[1024];
    size_t written = 0;
    int remaining = req->content_len;
    bool ok = true;
    while (remaining > 0)
    {
        int chunk = httpd_req_recv(req, buffer, remaining < (int)sizeof(buffer) ? remaining : sizeof(buffer));
        if (chunk <= 0) { ok = false; break; }
        if (fwrite(buffer, 1, chunk, file) != (size_t)chunk) { ok = false; break; }
        written += chunk;
        remaining -= chunk;
    }
    fclose(file);

    if (!ok) { unlink(part); return send_error(req, HTTPD_500_INTERNAL_SERVER_ERROR, 500, "fsWrite"); }

    unlink(target);
    if (rename(part, target) != 0) { unlink(part); return send_error(req, HTTPD_500_INTERNAL_SERVER_ERROR, 500, "fsRename"); }

    ESP_LOGI(TAG, "wrote %s, %u bytes", path, (unsigned)written);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "path", path);
    cJSON_AddNumberToObject(root, "size", (double)written);
    size_t total = 0, used = 0;
    esp_littlefs_info(FS_LABEL, &total, &used);
    cJSON_AddNumberToObject(root, "total", (double)total);
    cJSON_AddNumberToObject(root, "used", (double)used);
    char *text = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    esp_err_t result = text ? httpd_resp_sendstr(req, text) : ESP_ERR_NO_MEM;
    free(text);
    cJSON_Delete(root);
    return result;
}

/**
 * What the in-browser editor saves: {path, content}.
 *
 * Buffered rather than streamed, which is the whole reason FS_EDIT_MAX
 * exists - the body is in the request buffer, the parsed string is a second
 * copy, and both are on the heap an OTA update wants. The same part-file
 * dance as the upload, for the same reason.
 */
static esp_err_t save_text(httpd_req_t *req)
{
    if (!expert_lock_guard(req)) return ESP_FAIL;

    cJSON *root = NULL;
    if (!read_json_body(req, &root)) return send_error(req, HTTPD_400_BAD_REQUEST, 400, "fsBody");

    char path[FS_PATH_MAX + 1];
    bool valid = body_path(req, root, path, sizeof(path));
    cJSON *content_item = valid ? cJSON_GetObjectItem(root, "content") : NULL;
    const char *content = cJSON_IsString(content_item) ? content_item->valuestring : "";
    size_t content_len = strlen(content);

    if (!valid) { cJSON_Delete(root); return ESP_FAIL; }
    if (content_len > FS_EDIT_MAX) { cJSON_Delete(root); return send_error(req, HTTPD_400_BAD_REQUEST, 413, "fsTooBig"); }

    char target[FS_PATH_MAX + 8], part[FS_PATH_MAX + 16];
    full_path(path, target, sizeof(target));
    snprintf(part, sizeof(part), "%s.part", target);
    if (strlen(part) >= sizeof(part) - 1) { cJSON_Delete(root); return send_error(req, HTTPD_400_BAD_REQUEST, 400, "fsPath"); }

    FILE *file = fopen(part, "wb");
    if (!file) { cJSON_Delete(root); return send_error(req, HTTPD_500_INTERNAL_SERVER_ERROR, 500, "fsOpen"); }
    size_t written = fwrite(content, 1, content_len, file);
    fclose(file);

    if (written != content_len)
    {
        unlink(part);
        cJSON_Delete(root);
        return send_error(req, HTTPD_400_BAD_REQUEST, 507, "fsWrite");
    }

    unlink(target);
    if (rename(part, target) != 0)
    {
        unlink(part);
        cJSON_Delete(root);
        return send_error(req, HTTPD_500_INTERNAL_SERVER_ERROR, 500, "fsRename");
    }
    cJSON_Delete(root);

    ESP_LOGI(TAG, "saved %s, %u bytes", path, (unsigned)written);

    cJSON *answer = cJSON_CreateObject();
    cJSON_AddStringToObject(answer, "path", path);
    cJSON_AddNumberToObject(answer, "size", (double)written);
    size_t total = 0, used = 0;
    esp_littlefs_info(FS_LABEL, &total, &used);
    cJSON_AddNumberToObject(answer, "total", (double)total);
    cJSON_AddNumberToObject(answer, "used", (double)used);
    char *text = cJSON_PrintUnformatted(answer);
    httpd_resp_set_type(req, "application/json");
    esp_err_t result = text ? httpd_resp_sendstr(req, text) : ESP_ERR_NO_MEM;
    free(text);
    cJSON_Delete(answer);
    return result;
}

/**
 * Deletes one file, or one empty directory.
 *
 * Not recursive, on purpose. A file explorer that empties a directory tree
 * on one click is how the web UI gets deleted by somebody who meant to tidy
 * up, and LittleFS gives no way back.
 */
static esp_err_t remove_entry(httpd_req_t *req)
{
    if (!expert_lock_guard(req)) return ESP_FAIL;

    cJSON *root = NULL;
    if (!read_json_body(req, &root)) return send_error(req, HTTPD_400_BAD_REQUEST, 400, "fsBody");

    char path[FS_PATH_MAX + 1];
    if (!body_path(req, root, path, sizeof(path))) { cJSON_Delete(root); return ESP_FAIL; }
    cJSON_Delete(root);

    char full[FS_PATH_MAX + 8];
    full_path(path, full, sizeof(full));

    struct stat st;
    if (stat(full, &st) != 0) return send_error(req, HTTPD_404_NOT_FOUND, 404, "fsNotFound");
    bool is_dir = S_ISDIR(st.st_mode);

    if (is_dir)
    {
        DIR *dir = opendir(full);
        if (dir)
        {
            struct dirent *entry;
            bool has_child = false;
            while ((entry = readdir(dir)) != NULL)
            {
                if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) continue;
                has_child = true;
                break;
            }
            closedir(dir);
            if (has_child) return send_error(req, HTTPD_400_BAD_REQUEST, 409, "fsNotEmpty");
        }
    }

    bool gone = (is_dir ? rmdir(full) : unlink(full)) == 0;
    if (!gone) return send_error(req, HTTPD_500_INTERNAL_SERVER_ERROR, 500, "fsDelete");

    ESP_LOGW(TAG, "deleted %s", path);

    cJSON *answer = cJSON_CreateObject();
    cJSON_AddStringToObject(answer, "path", path);
    size_t total = 0, used = 0;
    esp_littlefs_info(FS_LABEL, &total, &used);
    cJSON_AddNumberToObject(answer, "total", (double)total);
    cJSON_AddNumberToObject(answer, "used", (double)used);
    char *text = cJSON_PrintUnformatted(answer);
    httpd_resp_set_type(req, "application/json");
    esp_err_t result = text ? httpd_resp_sendstr(req, text) : ESP_ERR_NO_MEM;
    free(text);
    cJSON_Delete(answer);
    return result;
}

/** Creates one directory. Its parent has to exist; LittleFS does not do -p. */
static esp_err_t make_dir(httpd_req_t *req)
{
    if (!expert_lock_guard(req)) return ESP_FAIL;

    cJSON *root = NULL;
    if (!read_json_body(req, &root)) return send_error(req, HTTPD_400_BAD_REQUEST, 400, "fsBody");

    char path[FS_PATH_MAX + 1];
    if (!body_path(req, root, path, sizeof(path))) { cJSON_Delete(root); return ESP_FAIL; }
    cJSON_Delete(root);

    char full[FS_PATH_MAX + 8];
    full_path(path, full, sizeof(full));

    struct stat st;
    if (stat(full, &st) == 0) return send_error(req, HTTPD_400_BAD_REQUEST, 409, "fsExists");
    if (mkdir(full, 0755) != 0) return send_error(req, HTTPD_500_INTERNAL_SERVER_ERROR, 500, "fsMkdir");

    cJSON *answer = cJSON_CreateObject();
    cJSON_AddStringToObject(answer, "path", path);
    char *text = cJSON_PrintUnformatted(answer);
    httpd_resp_set_type(req, "application/json");
    esp_err_t result = text ? httpd_resp_sendstr(req, text) : ESP_ERR_NO_MEM;
    free(text);
    cJSON_Delete(answer);
    return result;
}

static esp_err_t add_route(httpd_handle_t server, const char *uri, httpd_method_t method,
                           esp_err_t (*handler)(httpd_req_t *))
{
    httpd_uri_t route = {.uri = uri, .method = method, .handler = handler, .user_ctx = NULL};
    return httpd_register_uri_handler(server, &route);
}

esp_err_t file_routes_register(httpd_handle_t server)
{
    ESP_ERROR_CHECK(add_route(server, "/fs/list", HTTP_GET, send_list));
    ESP_ERROR_CHECK(add_route(server, "/fs/read", HTTP_GET, send_read));
    ESP_ERROR_CHECK(add_route(server, "/fs/upload", HTTP_POST, upload_handler));
    ESP_ERROR_CHECK(add_route(server, "/fs/save", HTTP_POST, save_text));
    ESP_ERROR_CHECK(add_route(server, "/fs/delete", HTTP_POST, remove_entry));
    ESP_ERROR_CHECK(add_route(server, "/fs/mkdir", HTTP_POST, make_dir));
    return ESP_OK;
}
