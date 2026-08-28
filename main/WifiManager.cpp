// wifi_manager.cpp
#include "WifiManager.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_event.h"
#include "esp_log.h"
#include <ESPmDNS.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

static const char *TAG = "wifi_mgr";
static bool s_connected = false;
static char s_ipstr[16] = "";
static bool s_sntp_started = false;
static char s_ssid[33] = "";
static char s_hostname[33] = "AG2998-Rotator";

static void load_hostname()
{
  nvs_handle_t h;
  if (nvs_open("wifi", NVS_READONLY, &h) != ESP_OK) return;
  size_t length = sizeof(s_hostname);
  if (nvs_get_str(h, "hostname", s_hostname, &length) != ESP_OK) {
    snprintf(s_hostname, sizeof(s_hostname), "%s", "AG2998-Rotator");
  }
  nvs_close(h);
}


static void on_wifi_event(void* arg, esp_event_base_t base, int32_t id, void* data) {
  if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
    s_connected = false;
    esp_wifi_connect();
    ESP_LOGW(TAG, "WLAN disconnected, reconnecting...");
  }
}

static void on_ip_event(void* arg, esp_event_base_t base, int32_t id, void* data) {
  if (id == IP_EVENT_STA_GOT_IP) {
    esp_netif_ip_info_t info = {};
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &info);
    snprintf(s_ipstr, sizeof(s_ipstr), IPSTR, IP2STR(&info.ip));
    s_connected = true;
    if (!s_sntp_started) {
      // Rotator's automatic-update night window follows local Munich time.
      setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
      tzset();
      esp_sntp_config_t sntp = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
      if (esp_netif_sntp_init(&sntp) == ESP_OK) s_sntp_started = true;
    }
    ESP_LOGI(TAG, "Got IP: %s", s_ipstr);
  }
}

void wifi_manager_init() {
  // 1) NVS für Credentials
  nvs_flash_init();
  // 2) TCP/IP-Stack
  esp_netif_init();
  esp_event_loop_create_default();
  esp_netif_t *station = esp_netif_create_default_wifi_sta();
  load_hostname();
  esp_netif_set_hostname(station, s_hostname);
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &on_wifi_event, NULL);
  esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &on_ip_event, NULL);
  // 3) NVRAM lesen
  nvs_handle_t h;
  if (nvs_open("wifi", NVS_READONLY, &h) == ESP_OK) {
    size_t ss, sp;
    char ssid[33], password[65];
    if (nvs_get_str(h, "ssid", nullptr, &ss) == ESP_OK &&
        nvs_get_str(h, "ssid", ssid, &ss) == ESP_OK &&
        nvs_get_str(h, "password", nullptr, &sp) == ESP_OK &&
        nvs_get_str(h, "password", password, &sp) == ESP_OK)
    {
      wifi_config_t wc = {};
      strcpy((char*)wc.sta.ssid, ssid);
      snprintf(s_ssid, sizeof(s_ssid), "%s", ssid);
      strcpy((char*)wc.sta.password, password);
      esp_wifi_set_mode(WIFI_MODE_STA);
      esp_wifi_set_config(WIFI_IF_STA, &wc);
      esp_wifi_start();
      esp_wifi_set_max_tx_power(34);
      esp_wifi_connect();  // non-blocking
      if (!MDNS.begin(s_hostname)) ESP_LOGW(TAG, "mDNS startup failed");
      else MDNS.addService("http", "tcp", 80);
    }
    nvs_close(h);
  } else {
    ESP_LOGW(TAG, "No stored WLAN-Credentials");
  }
}

bool wifi_manager_is_connected()
{
  return s_connected;
}

const char *wifi_manager_get_ip()
{
  return s_ipstr;
}

const char *wifi_manager_get_ssid()
{
  wifi_config_t config = {};
  if (esp_wifi_get_config(WIFI_IF_STA, &config) == ESP_OK) {
    snprintf(s_ssid, sizeof(s_ssid), "%s", (const char *)config.sta.ssid);
  }
  return s_ssid;
}
const char *wifi_manager_get_hostname() { return s_hostname; }

int wifi_manager_get_rssi()
{
  wifi_ap_record_t record = {};
  return s_connected && esp_wifi_sta_get_ap_info(&record) == ESP_OK ? record.rssi : 0;
}

esp_err_t wifi_manager_get_mac(uint8_t mac[6])
{
  return esp_wifi_get_mac(WIFI_IF_STA, mac);
}

esp_err_t wifi_manager_set_hostname(const char *hostname, char *stored, size_t stored_size)
{
  if (!hostname || !stored || stored_size == 0) return ESP_ERR_INVALID_ARG;
  char cleaned[33] = {};
  size_t out = 0;
  for (size_t i = 0; hostname[i] && out < sizeof(cleaned) - 1; ++i) {
    char ch = hostname[i];
    bool valid = (ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z') ||
                 (ch >= '0' && ch <= '9') || ch == '-';
    if (valid) cleaned[out++] = ch;
  }
  while (out > 0 && cleaned[out - 1] == '-') cleaned[--out] = 0;
  size_t first = 0;
  while (cleaned[first] == '-') ++first;
  if (first) memmove(cleaned, cleaned + first, strlen(cleaned + first) + 1);
  if (!cleaned[0]) return ESP_ERR_INVALID_ARG;

  nvs_handle_t h = 0;
  esp_err_t error = nvs_open("wifi", NVS_READWRITE, &h);
  if (error == ESP_OK) error = nvs_set_str(h, "hostname", cleaned);
  if (error == ESP_OK) error = nvs_commit(h);
  if (error == ESP_OK) snprintf(s_hostname, sizeof(s_hostname), "%s", cleaned);
  if (h) nvs_close(h);
  if (error == ESP_OK) snprintf(stored, stored_size, "%s", cleaned);
  return error;
}