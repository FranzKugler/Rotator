import { readFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';

const webServer = readFileSync(new URL('../main/WebServer.cpp', import.meta.url), 'utf8');
const wifiManager = readFileSync(new URL('../main/WifiManager.cpp', import.meta.url), 'utf8');
const ota = readFileSync(new URL('../main/OTAUpdate.c', import.meta.url), 'utf8');
const rotatorHW = readFileSync(new URL('../main/RotatorHW.cpp', import.meta.url), 'utf8');

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
    // ctx.location is cleared right after being copied out, so passing it to
    // set_url directly (rather than the local copy) would race the next
    // redirect's Location header against the request that is about to use it.
    expect(ota).not.toContain('esp_http_client_set_url(client, ctx.location)');
    expect(ota).toContain('esp_http_client_set_url(client, location)');
    expect(ota).not.toContain('esp_http_client_set_redirection');
  });

  it('reports configuration persistence errors instead of claiming success', () => {
    expect(ota).toMatch(/static bool save_config/);
    expect(ota).toContain('otaConfigSave');
  });
});

describe('Sensor calibration firmware contract', () => {
  it('persists a finished calibration run to flash', () => {
    // Regression test: calibrateAngleSensor() used to call setA/setB/setC0
    // with save=false and never call save() itself, so a completed
    // calibration was silently lost on the next restart.
    const body = rotatorHW.slice(rotatorHW.indexOf('RotatorHW::calibrateAngleSensor('));
    const setters = body.slice(0, body.indexOf('\n}'));
    expect(setters).toMatch(/cfg\.setA\(k, A\[k\], false\)/);
    expect(setters).toContain('cfg.save();');
  });

  it('reports before/after residual quality from a calibration run', () => {
    expect(rotatorHW).toContain('CalibrationResult RotatorHW::calibrateAngleSensor(');
    expect(rotatorHW).toContain('computeResidual(avgRaw)');
    expect(webServer).toContain('\\"residualBeforeDeg\\"');
    expect(webServer).toContain('\\"residualAfterDeg\\"');
    expect(webServer).toContain('\\"peakAfterDeg\\"');
  });

  it('fits the harmonics against the actually measured angle, not the ideal step angle', () => {
    // Regression test: correctSensorReading() evaluates cos(k*theta)/sin(k*theta)
    // at the raw sensor reading it is given. The fit used to correlate against
    // an angle accumulated from the step index instead (phi[k] += delta_phi[k],
    // starting at 0), which only matches when a calibration run happens to
    // start at raw sensor value 0 - any other start position phase-rotates the
    // fitted harmonics relative to how they are later applied. A live test run
    // that started away from raw 0 measured a ~50 degree spurious residual
    // from exactly this mismatch.
    expect(rotatorHW).not.toContain('phi[k]');
    expect(rotatorHW).not.toContain('delta_phi');
    const step = rotatorHW.slice(rotatorHW.indexOf('void RotatorHW::calibrateAngleSensorStep('));
    const body = step.slice(0, step.indexOf('\n}'));
    expect(body).toMatch(/theta\s*=\s*2\.0\s*\*\s*M_PI\s*\*\s*sensor_raw\s*\/\s*4096\.0/);
    expect(body).toContain('cos(k * theta)');
    expect(body).toContain('sin(k * theta)');
  });

  it('steps in true full-step mode and never attempts to rescale the position counter afterwards', () => {
    // Regression test, the other direction from most of the ones above: two
    // different live-tested "fixes" for the fact that full-step mode leaves
    // FastAccelStepper's position counter undercounting by up to 256x
    // (switching back to 256 microsteps and rescaling via one large
    // setCurrentPosition() jump, and moving MICROSTEPS pulses at the normal
    // resolution instead of ever entering full-step mode) both made live
    // results *worse* than the original forwardStep()-in-full-step-mode
    // behaviour, not better - one corrupted the counter into a nonsense
    // multi-million-step reading, the other produced measurably worse
    // calibration fits even at a single repeat. Whatever is actually wrong is
    // a deeper cross-task FastAccelStepper reliability issue that neither
    // attempt fixed, so don't reintroduce either of them here without first
    // resolving that separately and re-validating on the bench. The
    // consequence of leaving this alone: the position counter is left
    // undercounted after every calibration run, so gotoMechanicalZero() must
    // be re-run before trusting absolute position commands.
    const body = rotatorHW.slice(rotatorHW.indexOf('RotatorHW::calibrateAngleSensor('));
    const fn = body.slice(0, body.indexOf('\n}'));
    const liveCode = fn.split('\n').filter((line) => !line.trim().startsWith('//')).join('\n');
    expect(liveCode).toContain('setMicrostepsPerStep(1)');
    expect(liveCode).toContain('forwardStep()');
    expect(liveCode).toContain('setMicrostepsPerStep(256)');
    expect(liveCode).not.toContain('setStepPositionSafe');
    expect(liveCode).not.toContain('MOVE_WAIT');
  });

  it('serializes access to the stepper position counter across tasks', () => {
    // Regression test: FastAccelStepper's ESP32 backend pairs a PCNT hardware
    // register with a software overflow-extension word. setCurrentPosition()
    // has to update both, and angle_producer_task reads the position every
    // 100ms from a different task - a read caught mid-update can see an
    // inconsistent combination of the two. A live test saw this turn a
    // calibration run's position rescale into a nonsense multi-million-step
    // reading. getStepPositionSafe()/setStepPositionSafe() must be the only
    // way calibration, homing and the live position getters touch the
    // counter - direct stepper->getCurrentPosition()/setCurrentPosition()
    // calls outside of those two wrappers reintroduce the race.
    expect(rotatorHW).toContain('int32_t RotatorHW::getStepPositionSafe()');
    expect(rotatorHW).toContain('void RotatorHW::setStepPositionSafe(int32_t newPosition)');
    for (const fnName of ['getSensorSnapshot', 'getPosition', 'getMechanicalPosition']) {
      const fn = rotatorHW.slice(rotatorHW.indexOf(`RotatorHW::${fnName}(`));
      const body = fn.slice(0, fn.indexOf('\n}'));
      const liveCode = body.split('\n').filter((line) => !line.trim().startsWith('//')).join('\n');
      expect(liveCode).not.toContain('stepper->getCurrentPosition()');
      expect(body).toContain('getStepPositionSafe()');
    }
  });

  it('broadcasts live raw sensor and motor debug fields alongside the position', () => {
    expect(webServer).toContain('\\"rawSensor\\"');
    expect(webServer).toContain('\\"correctedSensor\\"');
    expect(webServer).toContain('\\"stepPosition\\"');
    expect(webServer).toContain('\\"hall\\"');
    expect(webServer).toContain('getSensorSnapshot()');
  });
});
