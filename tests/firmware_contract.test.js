import { readFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';

const webServer = readFileSync(new URL('../main/WebServer.cpp', import.meta.url), 'utf8');
const wifiManager = readFileSync(new URL('../main/WifiManager.cpp', import.meta.url), 'utf8');
const ota = readFileSync(new URL('../main/OTAUpdate.c', import.meta.url), 'utf8');
const rotatorHW = readFileSync(new URL('../main/RotatorHW.cpp', import.meta.url), 'utf8');
const mainCpp = readFileSync(new URL('../main/main.cpp', import.meta.url), 'utf8');
const alpacaApi = readFileSync(new URL('../main/alpaca_server/api.cpp', import.meta.url), 'utf8');
const expertLock = readFileSync(new URL('../main/ExpertLock.c', import.meta.url), 'utf8');
const fileRoutes = readFileSync(new URL('../main/FileRoutes.c', import.meta.url), 'utf8');
const nvsRoutes = readFileSync(new URL('../main/NvsRoutes.c', import.meta.url), 'utf8');
const logBuffer = readFileSync(new URL('../main/LogBuffer.c', import.meta.url), 'utf8');

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

  it('gates the raw microstep jog debug endpoint behind expert mode and a distance clamp', () => {
    // This endpoint bypasses every degree/offset conversion the normal
    // Alpaca motion API applies, so it must stay behind the same expert gate
    // as the rest of this file's raw hardware access, and must not accept an
    // unbounded distance that could block the HTTP server task or spin the
    // motor arbitrarily far on a typo.
    expect(webServer).toContain('"/api/debug/jog"');
    const fn = webServer.slice(webServer.indexOf('debug_jog_handler(httpd_req_t *req)'));
    const body = fn.slice(0, fn.indexOf('\n}'));
    expect(body).toContain('expert_lock_guard(req)');
    expect(body).toMatch(/JOG_LIMIT/);
    expect(body).toContain('jogMicrosteps(');
    expect(body).toContain('measureRawAngle(samples)');
  });

  it('gates the full-step alignment debug endpoint behind expert mode', () => {
    // Regression test: a within-full-step analysis over /api/debug/jog's raw
    // microsteps is only meaningful once "phase 0" is known to actually sit
    // on a true mechanical full-step position, not just assumed to - two
    // live sweeps found materially different (and unexplained) results
    // before this existed. See RotatorHW::alignToFullStep().
    expect(webServer).toContain('"/api/debug/align-fullstep"');
    const fn = webServer.slice(webServer.indexOf('debug_align_fullstep_handler(httpd_req_t *req)'));
    const body = fn.slice(0, fn.indexOf('\n}'));
    expect(body).toContain('expert_lock_guard(req)');
    expect(body).toContain('alignToFullStep()');
  });

  it('takes its one real full step with forwardStep(), not MOVE_WAIT', () => {
    // Regression test, same lesson as calibrateAngleSensor() above applied
    // to the same microstep-mode switch: MOVE_WAIT's stepper->move() +
    // isRunning() poll measurably degraded results when tried for a
    // full-step-mode move on this motor, while forwardStep()+delay() is the
    // proven-safe primitive for it. alignToFullStep() must take one real
    // step (not zero - a zero-length "step" would just re-arm full-step mode
    // without ever proving the driver reached a mechanical equilibrium).
    const fn = rotatorHW.slice(rotatorHW.indexOf('RotatorHW::alignToFullStep()'));
    const body = fn.slice(0, fn.indexOf('\n}'));
    expect(body).toContain('setMicrostepsPerStep(1)');
    expect(body).toContain('forwardStep()');
    expect(body).toContain('setMicrostepsPerStep(256)');
    expect(body).not.toContain('MOVE_WAIT');
  });
});

describe('TMC2209 current configuration firmware contract', () => {
  it('does not let setRunCurrent()/enableCoolStep() override the RMS current setup', () => {
    // Regression test: begin() used to call setRMSCurrent(250, 0.11, 0.2) and
    // then immediately setRunCurrent(100), which discards the CS that
    // setRMSCurrent() had just computed and instead drives IRUN to the
    // driver's absolute maximum (CS=31, ~980mA at this Rsense/vsense) while
    // IHOLD stays at the small setRMSCurrent()-derived value. At this
    // motor's 5V supply the coil (17ohm/phase) can physically draw at most
    // ~290mA, so StealthChop's current regulator saturated its PWM duty
    // cycle for most of each microstep's sine/cosine wave - only regulating
    // correctly within roughly +-17 degrees of each zero crossing - a likely
    // major contributor to the measured motor-wobble. enableCoolStep()
    // compounds this by pulling current below IRUN whenever it estimates low
    // load, fighting positional stiffness. Precision, not quietness or power
    // draw, is what this application needs.
    const start = rotatorHW.indexOf('void RotatorHW::begin()');
    const end = rotatorHW.indexOf('\nvoid RotatorHW::', start + 1);
    const body = rotatorHW.slice(start, end);
    expect(body).toContain('setRMSCurrent(');
    expect(body).not.toContain('setRunCurrent(');
    expect(body).not.toContain('enableCoolStep()');
    expect(body).toContain('disableCoolStep()');
  });
});

describe('HTTP server handler-table headroom', () => {
  it('registers real boot-time URI handlers with real headroom below max_uri_handlers', () => {
    // Regression test: a live device panicked with ESP_ERR_HTTPD_HANDLERS_FULL
    // (abort() in register_web_handles(), registering the final/wildcard
    // handler, WebServer.cpp:706) after one more debug route was added on top
    // of a boot-time total that was already sitting almost exactly at the
    // then-configured max_uri_handlers=64 - found by reproducing the crash on
    // an unrelated bare board with a full serial console attached, since the
    // real rotator's own console is UART0, physically unreachable over its
    // single dual-use USB-C port. Fix was raising the limit (see main.cpp);
    // this test recomputes the real total and checks for headroom, so the
    // next added route fails a fast local test instead of an on-device
    // ESP_ERROR_CHECK abort. The total deliberately does NOT come from a
    // naive count of every REGISTER_DEVICE_ROUTE(...) in
    // alpaca_server/api.cpp - most of those are for Alpaca device types
    // (covercalibrator, dome, focuser, ...) whose registration functions
    // exist in that file but are never called for this project's Rotator
    // device, and counting them overstates the real total enormously.
    const countOccurrences = (text, pattern) => (text.match(new RegExp(pattern, 'g')) || []).length;
    const sliceFunctionBody = (text, functionSignature) => {
      const start = text.indexOf(functionSignature);
      expect(start, `function not found: ${functionSignature}`).toBeGreaterThan(-1);
      const body = text.slice(start);
      return body.slice(0, body.indexOf('\n}'));
    };

    const maxUriHandlersMatch = mainCpp.match(/http_cfg\.max_uri_handlers\s*=\s*(\d+);/);
    expect(maxUriHandlersMatch).not.toBeNull();
    const maxUriHandlers = Number(maxUriHandlersMatch[1]);

    // WebServer.cpp and LogBuffer.c call httpd_register_uri_handler() directly,
    // no local wrapper.
    const webServerRoutes = countOccurrences(webServer, 'httpd_register_uri_handler\\(');
    const logBufferRoutes = countOccurrences(logBuffer, 'httpd_register_uri_handler\\(');

    // OTAUpdate.c, ExpertLock.c, FileRoutes.c and NvsRoutes.c each go through
    // a local wrapper (register_uri()/add_route()) that itself calls
    // httpd_register_uri_handler() exactly once - counting the wrapper's own
    // textual occurrence, rather than how many times callers invoke it, is
    // the undercounting mistake this bug's own investigation made first.
    const otaRoutes = countOccurrences(ota, 'register_uri\\("');
    const expertRoutes = countOccurrences(expertLock, 'add_route\\(server,');
    const fileRoutesCount = countOccurrences(fileRoutes, 'add_route\\(server,');
    const nvsRoutesCount = countOccurrences(nvsRoutes, 'add_route\\(server,');

    // alpaca_server/api.cpp: only what a Rotator device actually reaches -
    // Api::register_routes()'s own direct /management/ routes,
    // Api::register_device_routes()'s common REGISTER_DEVICE_ROUTE(...)
    // calls (shared by every Alpaca device type), and
    // Api::register_rotator_routes(), the only case this project's device
    // type ever makes register_device_routes()'s switch(device_type)
    // dispatch to - NOT its covercalibrator/dome/focuser/etc. siblings,
    // which exist in the same file for other device types but are never
    // called here.
    const managementBlock = sliceFunctionBody(alpacaApi, 'void Api::register_routes(httpd_handle_t server)');
    const commonBlock = sliceFunctionBody(alpacaApi, 'void Api::register_device_routes(');
    const rotatorBlock = sliceFunctionBody(alpacaApi, 'void Api::register_rotator_routes(');
    const alpacaRoutes =
      countOccurrences(managementBlock, 'ESP_ERROR_CHECK\\(httpd_register_uri_handler\\(') +
      countOccurrences(commonBlock, 'REGISTER_DEVICE_ROUTE\\(') +
      countOccurrences(rotatorBlock, 'REGISTER_DEVICE_ROUTE\\(');

    const total = webServerRoutes + logBufferRoutes + otaRoutes + expertRoutes +
      fileRoutesCount + nvsRoutesCount + alpacaRoutes;

    // Sanity floor: this project has always registered well over 50 routes -
    // a much smaller total would mean the counting logic above broke (e.g. a
    // renamed wrapper function), silently making the headroom check below
    // meaningless rather than failing loudly.
    expect(total).toBeGreaterThan(50);
    expect(total + 10).toBeLessThanOrEqual(maxUriHandlers);
  });
});
