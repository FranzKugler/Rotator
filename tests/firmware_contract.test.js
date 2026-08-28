import { readFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';

const webServer = readFileSync(new URL('../main/WebServer.cpp', import.meta.url), 'utf8');
const wifiManager = readFileSync(new URL('../main/WifiManager.cpp', import.meta.url), 'utf8');
const ota = readFileSync(new URL('../main/OTAUpdate.c', import.meta.url), 'utf8');

describe('WLAN firmware contract', () => {
  it('provides current WLAN state and hostname endpoints', () => {
    expect(webServer).toContain('"/api/wifi/status"');
    expect(webServer).toContain('"/api/wifi/hostname"');
    for (const field of ['ssid', 'ip', 'rssi', 'hostname', 'mac']) {
      expect(webServer).toContain(`root, "${field}"`);
    }
  });

  it('persists and applies a DNS-safe hostname', () => {
    expect(wifiManager).toContain('wifi_manager_set_hostname');
    expect(wifiManager).toContain('nvs_set_str');
    expect(wifiManager).toMatch(/esp_netif_set_hostname|MDNS\.begin/);
  });
});

describe('OTA firmware contract', () => {
  it('follows every GitHub redirect with a fresh app-owned Location value', () => {
    expect(ota).toContain('HTTP_EVENT_ON_HEADER');
    expect(ota).toContain('ctx->location');
    expect(ota).toContain('.disable_auto_redirect = true');
    expect(ota).toContain('esp_http_client_set_url(client, ctx.location)');
    expect(ota).toContain('esp_http_client_set_url(client, location)');
    expect(ota).not.toContain('esp_http_client_set_redirection');
  });

  it('reports configuration persistence errors instead of claiming success', () => {
    expect(ota).toMatch(/static bool save_config/);
    expect(ota).toContain('otaConfigSave');
  });
});
