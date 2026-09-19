"""Thin, dependency-free access layer to the rotator's expert-gated debug
endpoints (main/WebServer.cpp), for the fresh calibration campaign under
scripts/calib/.

Deliberately minimal and deliberately separate from the ASCOM path
(scripts/alpaca_client.py): everything here drives the motor in raw
microsteps and reads the *uncorrected* AS5600, which is exactly what a
from-scratch sensor characterisation needs. The Alpaca client stays the
tool for the final, end-user-representative validation run.

Units, all from main/RotatorHW.cpp:
    1 full step   = 256 microsteps = 0.9 deg motor = 0.09 deg output
    1 motor rev   = 400 full steps = 36 deg output = one AS5600 revolution
    1 AS5600 LSB  = 4096 per motor rev = 36/4096 = 8.7891e-3 deg output
"""

import json
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen

MICROSTEPS_PER_FULLSTEP = 256
FULLSTEPS_PER_MOTOR_REV = 400
SENSOR_COUNTS = 4096.0
OUTPUT_DEG_PER_MOTOR_REV = 36.0
OUTPUT_DEG_PER_COUNT = OUTPUT_DEG_PER_MOTOR_REV / SENSOR_COUNTS
OUTPUT_DEG_PER_FULLSTEP = OUTPUT_DEG_PER_MOTOR_REV / FULLSTEPS_PER_MOTOR_REV
COUNTS_PER_FULLSTEP = SENSOR_COUNTS / FULLSTEPS_PER_MOTOR_REV  # 10.24


class Rotator:
    def __init__(self, host, timeout=60):
        self.host = host
        self.timeout = timeout

    def _post(self, path, payload):
        body = json.dumps(payload).encode()
        req = Request(f"http://{self.host}{path}", data=body,
                      headers={"Content-Type": "application/json"}, method="POST")
        with urlopen(req, timeout=self.timeout) as resp:
            body = resp.read()
        # The write handlers answer with a bare "OK" (set_fullstep_table) or
        # an empty body (set_coefficients) rather than JSON. Both are
        # successes - only a non-200 is a failure, and urlopen already
        # raises on those - so a body that is not JSON is returned as text
        # instead of being treated as a parse error.
        text = body.decode(errors="replace").strip()
        if not text:
            return {}
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            return {"text": text}

    def _get(self, path):
        with urlopen(f"http://{self.host}{path}", timeout=self.timeout) as resp:
            return json.loads(resp.read())

    # --- raw motion + sensor -------------------------------------------------
    def jog(self, microsteps, samples=1):
        """Move `microsteps`, then read the sensor. microsteps=0 measures only."""
        return self._post("/api/debug/jog", {"microsteps": int(microsteps),
                                             "samples": int(samples)})

    def measure(self, samples=16, settle=0.0):
        """Pure measurement, no motion. Returns the jog response dict."""
        if settle:
            time.sleep(settle)
        return self.jog(0, samples)

    def align_fullstep(self):
        """Drive onto a true mechanical full step; returns its stepPosition."""
        return self._post("/api/debug/align-fullstep", {})["stepPosition"]

    def sensor_diagnostics(self):
        return self._get("/api/debug/sensor-diagnostics")

    def goto_mechanical_zero(self):
        return self._post("/api/debug/goto-mechanical-zero", {})

    # --- stored calibration --------------------------------------------------
    def get_coefficients(self):
        return self._get("/api/calibration/coefficients")

    def set_coefficients(self, c0, a, b):
        return self._post("/api/calibration/coefficients", {"C0": c0, "A": list(a), "B": list(b)})

    def get_fullstep_table(self):
        return self._get("/api/calibration/fullstep-table")

    def set_fullstep_table(self, table):
        return self._post("/api/calibration/fullstep-table", {"table": list(table)})


def unwrap_counts(values, period=SENSOR_COUNTS):
    """Unwrap a monotone-ish sequence of AS5600 readings into a continuous
    ramp, so a sweep across the 4095->0 seam stays differentiable."""
    out = []
    offset = 0.0
    previous = None
    for v in values:
        if previous is not None:
            delta = v + offset - previous
            while delta < -period / 2:
                offset += period
                delta += period
            while delta > period / 2:
                offset -= period
                delta -= period
        out.append(v + offset)
        previous = out[-1]
    return out


def check_reachable(rotator):
    """Fail fast with a clear message if the target or the expert gate is not
    where the campaign needs it."""
    try:
        rotator.measure(samples=1)
    except HTTPError as e:
        raise SystemExit(
            f"debug endpoint refused the request (HTTP {e.code}) - "
            "is expert mode unlocked on the target?")
