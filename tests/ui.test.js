import { readFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';

const app = readFileSync(new URL('../web/src/App.svelte', import.meta.url), 'utf8');
const main = readFileSync(new URL('../web/src/main.js', import.meta.url), 'utf8');
const css = readFileSync(new URL('../web/src/app.css', import.meta.url), 'utf8');
const firmware = readFileSync(new URL('../web/src/sections/Firmware.svelte', import.meta.url), 'utf8');
const index = readFileSync(new URL('../web/index.html', import.meta.url), 'utf8');

describe('QlockThreeW32-compatible shell', () => {
  it('uses the requested product title everywhere in the title bar', () => {
    expect(app).toContain('<title>AG2998 Camera Rotator</title>');
    expect(app).toContain('<h1>AG2998 Camera Rotator</h1>');
    expect(index).toContain('<title>AG2998 Camera Rotator</title>');
    expect(`${app}\n${index}`).not.toMatch(/AG2998 Rotator(?! firmware)/);
  });

  it('loads the shared QlockThreeW32 look and uses its card vocabulary', () => {
    expect(main).toContain("import './app.css';");
    expect(css).toContain('--surface: #ffffff');
    expect(css).toContain('.card > h2');
    expect(app).toContain('class="card"');
    expect(firmware).toContain('class="card"');
  });

  it('does not expose copied QlockThreeW32 wording', () => {
    expect(`${app}\n${firmware}`).not.toMatch(/QlockThreeW32|word clock|clock firmware/i);
  });

  it('matches the Qlock WLAN cards including status and device name', () => {
    expect(app).toContain('<h2>Connection</h2>');
    expect(app).toContain('<h2>Device name</h2>');
    for (const label of ['Network', 'Address', 'Signal', 'Hostname', 'MAC']) {
      expect(app).toContain(`>${label}<`);
    }
    expect(app).toContain("'/api/wifi/status'");
    expect(app).toContain("'/api/wifi/hostname'");
  });

  it('lays out the RNDIS network controls as equal stacked rows', () => {
    expect(app).toContain('class="network-fields"');
    expect(app).toMatch(/\.network-fields[\s\S]*grid-template-columns:\s*1fr/);
    expect(app).toMatch(/\.network-fields input[\s\S]*width:\s*100%/);
  });
});
