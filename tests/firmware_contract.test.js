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
const configuration = readFileSync(new URL('../main/Configuration.cpp', import.meta.url), 'utf8');
const rotatorHeader = readFileSync(new URL('../main/RotatorHW.h', import.meta.url), 'utf8');
const cameraAngle = readFileSync(new URL('../main/CameraAngle.c', import.meta.url), 'utf8');

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
    expect(rotatorHW).toContain('computeResidual(raw, error)');
    expect(webServer).toContain('\\"residualBeforeDeg\\"');
    expect(webServer).toContain('\\"residualAfterDeg\\"');
    expect(webServer).toContain('\\"peakAfterDeg\\"');
  });

  it('reports the two checks that say whether a run can be trusted at all', () => {
    // A residual only means something if the sweep itself was sound. Closure
    // says no steps were lost - the AS5600 is absolute, so a clean revolution
    // comes back to itself - and repeatability between the two repeats is the
    // noise floor the residual has to be judged against. Reporting the
    // residual without them invites trusting a number from a broken run.
    expect(rotatorHeader).toContain('double closureCounts;');
    expect(rotatorHeader).toContain('double repeatabilityCounts;');
    expect(webServer).toContain('\\"closureCounts\\"');
    expect(webServer).toContain('\\"repeatabilityCounts\\"');
  });

  it('keeps the correction and everything derived from it on one generation', () => {
    // The mechanical zero is stored as a corrected sensor value and the
    // full-step table is indexed by one, so both are only meaningful
    // alongside the coefficients they were measured against. Live on
    // 2026-09-19 a table left over from a previous correction was found
    // injecting 353 mdeg peak to peak with nothing reporting it. Every write
    // of the coefficients bumps a generation; the two derived artefacts are
    // stamped with it and checked at load.
    expect(configuration).toContain('ANGLECAL_MAGIC');
    expect(configuration).toContain('blob.generation');
    expect(configuration).toContain('FULLSTEP_GEN_NVS_KEY');
    expect(rotatorHW).toContain('"zeroGen"');
    expect(rotatorHW).toContain('_zeroCalibrationStale');
    expect(webServer).toContain('"/api/calibration/status"');
    // A stale table is dropped rather than applied: all-zero is a no-op,
    // a mismatched one is a confident wrong answer.
    const load = configuration.slice(configuration.indexOf('bool Configuration::loadFullStepTableFromNvs()'));
    const body = load.slice(0, load.indexOf('\n}'));
    expect(body).toContain('fullStepTable.fill(0.0f)');
    // ...but an already-zero table is a no-op correction and belongs to every
    // generation, so it must not be reported as orphaned forever.
    expect(body).toContain('allZero');
  });

  it('migrates a pre-header calibration instead of silently discarding it', () => {
    expect(configuration).toContain('LegacyAngleCalBlob');
    const load = configuration.slice(configuration.indexOf('bool Configuration::loadAngleCalFromNvs()'));
    expect(load.slice(0, load.indexOf('\n}'))).toContain('sizeof(LegacyAngleCalBlob)');
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

  it('sweeps at the normal microstep resolution and never switches the driver', () => {
    // This reverses an earlier contract, on the strength of a bench campaign
    // that the earlier one explicitly asked for before it could be reversed
    // ("don't reintroduce either of them here without first resolving that
    // separately and re-validating on the bench").
    //
    // The old routine switched the driver to full-step mode and used
    // forwardStep(), which leaves FastAccelStepper's position counter
    // undercounting by up to 256x and forces a re-home afterwards, and which
    // is the prime suspect for the hangs that killed 4 of 4 live attempts.
    // Moving 256 microsteps at the normal resolution had been tried once and
    // judged "measurably worse", but that attempt kept the rest of the
    // routine as it was - in particular a 20 ms settle, far too short for a
    // 256-microstep move to stop ringing, and a fit anchored to the sweep's
    // own first step rather than to the machine's zero.
    //
    // Re-measured on 2026-09-18/19 with a settle of 250 ms and an absolute
    // reference: two independent sweeps agreeing within a few percent on
    // every harmonic, closure over a revolution within 0.5 counts, forward
    // repeatability 0.149 counts, residual 0.347 counts (3.0 mdeg of output
    // angle) - and the result verified end to end against an independent
    // output-shaft camera. See scripts/calib/RESULTS.md.
    const body = rotatorHW.slice(rotatorHW.indexOf('RotatorHW::calibrateAngleSensor('));
    const fn = body.slice(0, body.indexOf('\n}'));
    const liveCode = fn.split('\n').filter((line) => !line.trim().startsWith('//')).join('\n');
    expect(liveCode).not.toContain('setMicrostepsPerStep');
    expect(liveCode).not.toContain('forwardStep()');
    expect(liveCode).not.toContain('setStepPositionSafe');
    expect(liveCode).toContain('MOVE_WAIT');
  });

  it('references the fit to the absolute step counter, not to the sweep\'s own start', () => {
    // The mechanical-zero target is stored as a *corrected* sensor value, so
    // the constant term of the correction decides where the machine thinks
    // zero is. Anchoring the fit to the sweep's first step made that constant
    // depend on wherever the sweep happened to begin, which is why every
    // recalibration used to silently move the zero. Against the step counter
    // it is a property of the machine instead.
    const body = rotatorHW.slice(rotatorHW.indexOf('RotatorHW::calibrateAngleSensor('));
    const fn = body.slice(0, body.indexOf('\n}'));
    expect(fn).toContain('COUNTS_PER_MICROSTEP');
    expect(fn).not.toMatch(/ideal\s*=\s*4096\.0f?\s*\*\s*step_counter/);
  });

  it('settles long enough after each step for the reading to mean anything', () => {
    const body = rotatorHW.slice(rotatorHW.indexOf('RotatorHW::calibrateAngleSensor('));
    const fn = body.slice(0, body.indexOf('\n}'));
    const settle = fn.match(/SETTLE_MS\s*=\s*(\d+)/);
    expect(settle).not.toBeNull();
    expect(Number(settle[1])).toBeGreaterThanOrEqual(150);
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
      // Either directly, or via getMechanicalPosition() (itself checked by
      // this same loop) - getPosition() composes its coarse term from that
      // wrapper rather than repeating the getStepPositionSafe() call inline.
      expect(body).toMatch(/getStepPositionSafe\(\)|getMechanicalPosition\(\)/);
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

describe('Calibration runs off the HTTP server task', () => {
  it('hands the request to a worker instead of streaming from the server task', () => {
    // ESP-IDF's httpd serves every socket from one task, so a handler that
    // streams progress for minutes stops the whole server answering - not
    // just its own connection. Live on 2026-09-19 a run pointed at an
    // unreachable angle source took the device off the network for hours.
    // Moving only the measurement would not have helped: the SSE handler is
    // what holds the task. Measured after the change: 12 of 12 status
    // probes answered in 94-193 ms while a calibration was running.
    expect(webServer).toContain('httpd_req_async_handler_begin');
    expect(webServer).toContain('httpd_req_async_handler_complete');
    expect(webServer).toContain('xTaskCreate(calibration_worker');
    // The handler must not do the work itself any more.
    for (const name of ['calibration_zero_stream', 'calibration_angle_stream',
                        'calibration_camera_stream']) {
      const fn = webServer.slice(webServer.indexOf(`static esp_err_t ${name}(httpd_req_t *req)`));
      const body = fn.slice(0, fn.indexOf('\n}'));
      expect(body).toContain('start_calibration(req,');
      expect(body).not.toContain('httpd_resp_send_chunk');
    }
  });

  it('always relinquishes the socket, and refuses a second run', () => {
    // An async request that is never completed leaves the socket owned
    // forever; the server then stops accepting connections altogether.
    const start = webServer.slice(webServer.indexOf('static esp_err_t start_calibration('));
    const body = start.slice(0, start.indexOf('\n}'));
    expect(body).toContain('s_calibrationBusy');
    expect(body).toContain('409 Conflict');
    // the failure path after a successful begin() must still complete it
    expect(body).toContain('httpd_req_async_handler_complete(async)');
    const worker = webServer.slice(webServer.indexOf('static void calibration_worker('));
    expect(worker.slice(0, worker.indexOf('\n    vTaskDelete'))).toContain('httpd_req_async_handler_complete(req)');
  });

  it('lets the server reclaim idle sockets, now that a run holds one for half an hour', () => {
    expect(mainCpp).toContain('lru_purge_enable = true');
  });

  it('gives an upload long enough to survive the filesystem erase', () => {
    // The filesystem upload handler erases all 10 MB of the littlefs
    // partition in one call before reading the rest of the body, and that
    // erase alone outlasts httpd's five-second default. A live upload timed
    // out mid-write and left the device with an erased partition and no web
    // UI - much worse than a refused upload.
    const match = mainCpp.match(/recv_wait_timeout\s*=\s*(\d+)/);
    expect(match).not.toBeNull();
    expect(Number(match[1])).toBeGreaterThanOrEqual(15);
  });
});

describe('Output-angle calibration firmware contract', () => {
  it('asks an external service for the angle instead of doing vision on the device', () => {
    // The image processing belongs where the image already is. Doing it here
    // would mean pulling a 270 KB JPEG over WiFi and decoding it into 1.9 MB
    // of PSRAM to reach pixels another device already holds, plus the
    // largest and least testable code in the project.
    expect(cameraAngle).toContain('camera_angle_read');
    expect(cameraAngle).toContain('esp_http_client_perform');
    expect(webServer).toContain('"/api/calibration/camera-source"');
    expect(webServer).toContain('"/api/calibration/camera/stream"');
  });

  it('bounds the TCP connect itself instead of trusting the client timeout', () => {
    // esp_http_client's timeout governs the transaction, not the connect,
    // and this build retransmits SYN twelve times. Against an address that
    // silently drops packets one connect then takes minutes - live on
    // 2026-09-19 that turned a 164-position run into hours during which the
    // device answered nothing, because httpd serves every socket from the
    // task the handler runs in.
    expect(cameraAngle).toContain('host_reachable');
    const fn = cameraAngle.slice(cameraAngle.indexOf('static bool host_reachable('));
    const body = fn.slice(0, fn.indexOf('\n}'));
    expect(body).toContain('O_NONBLOCK');
    expect(body).toContain('select(');
    const read = cameraAngle.slice(cameraAngle.indexOf('bool camera_angle_read('));
    expect(read.slice(0, read.indexOf('\n}'))).toContain('host_reachable(url,');
  });

  it('treats any failure of the angle source as no measurement, never as zero', () => {
    // A skipped position costs one sample. A zero treated as an angle poisons
    // the fit for every position after it.
    const fn = cameraAngle.slice(cameraAngle.indexOf('bool camera_angle_read('));
    const client = fn.slice(0, fn.indexOf('\n}'));
    for (const guard of ['status != 200', 'body.length == 0', 'end == body.buffer'])
      expect(client).toContain(guard);
    const run = rotatorHW.slice(rotatorHW.indexOf('RotatorHW::calibrateOutputAngle('));
    const body = run.slice(0, run.indexOf('\n}'));
    expect(body).toContain('no angle at');
    // And it stops rather than driving the whole travel to discover the
    // source was never reachable - with progress counted per position
    // attempted, so a dead source does not look like a hung run.
    expect(body).toContain('GIVE_UP_AFTER');
    const progressAt = body.indexOf('onProgress(');
    const readAt = body.indexOf('camera_angle_read(');
    expect(progressAt).toBeGreaterThan(0);
    expect(progressAt).toBeLessThan(readAt);
  });

  it('refuses a correction that cannot be a gear error', () => {
    // Most likely cause of a huge one is an angle source that counts the
    // other way, which would make the "correction" roughly minus twice the
    // angle and wreck every move afterwards.
    const run = rotatorHW.slice(rotatorHW.indexOf('RotatorHW::calibrateOutputAngle('));
    const body = run.slice(0, run.indexOf('\n}'));
    expect(body).toContain('SANITY_LIMIT_MDEG');
    expect(body).toMatch(/rmsBeforeMdeg > SANITY_LIMIT_MDEG/);
  });

  it('measures through the normal commanded-move path, not a private one', () => {
    // A correction measured with a different motion discipline than the one
    // it will be used under - in particular without the one-sided approach -
    // is a correction for a different machine.
    const run = rotatorHW.slice(rotatorHW.indexOf('RotatorHW::calibrateOutputAngle('));
    expect(run.slice(0, run.indexOf('\n}'))).toContain('putMechanicalPosition(target)');
  });

  it('feeds the correction forward onto the target only, never onto the reported position', () => {
    // The closed loop drives the sensor, and the sensor cannot see the
    // gearing. Correcting the reported position instead would make the loop
    // chase its own correction.
    for (const fnName of ['putAbsolutePosition', 'putMechanicalPosition']) {
      const fn = rotatorHW.slice(rotatorHW.indexOf(`RotatorHW::${fnName}(`));
      expect(fn.slice(0, fn.indexOf('\n}'))).toContain('outputAngleCorrection(');
    }
    for (const fnName of ['getPosition', 'getMechanicalPosition']) {
      const fn = rotatorHW.slice(rotatorHW.indexOf(`RotatorHW::${fnName}(`));
      expect(fn.slice(0, fn.indexOf('\n}'))).not.toContain('outputAngleCorrection(');
    }
  });

  it('stamps the output correction with the sensor calibration it was measured against', () => {
    expect(rotatorHW).toContain('OUTCAL_MAGIC');
    const load = rotatorHW.slice(rotatorHW.indexOf('bool RotatorHW::loadOutputCalibration()'));
    expect(load.slice(0, load.indexOf('\n}'))).toContain('calibrationGeneration()');
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
