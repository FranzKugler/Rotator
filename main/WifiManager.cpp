// wifi_manager.cpp
#include "WifiManager.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "wifi_mgr";
static bool s_connected = false;
static char s_ipstr[16] = "";


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
    ESP_LOGI(TAG, "Got IP: %s", s_ipstr);
  }
}

void wifi_manager_init() {
  // 1) NVS für Credentials
  nvs_flash_init();
  // 2) TCP/IP-Stack
  esp_netif_init();
  esp_event_loop_create_default();
  esp_netif_create_default_wifi_sta();
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
      strcpy((char*)wc.sta.password, password);
      esp_wifi_set_mode(WIFI_MODE_STA);
      esp_wifi_set_config(WIFI_IF_STA, &wc);
      esp_wifi_start();
      esp_wifi_set_max_tx_power(34);
      esp_wifi_connect();  // non-blocking
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