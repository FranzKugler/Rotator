"""Serve the output-shaft angle over HTTP, so the rotator can calibrate
against it without carrying any image processing itself.

This is the host-side half of the split the firmware assumes: the rotator
knows how to move, fit and store; something else knows how to look at the
shaft. Eventually that something else should be RotatorCam, which already
has the frame in its own memory - this exists so the feature works before
that, and so the rotator's side can be tested against a known-good
measurement.

    GET /angle  ->  {"angleDeg": <float>, "frames": n, "residualPx": r}

The angle is relative to the first frame served, which is all the rotator
needs: it removes a constant offset of its own. The measurement is the same
homography-ratio estimator the whole campaign used - see measure_frames.py -
and it is quoted in OUTPUT degrees, the same units the rotator commands in.

    python3 scripts/calib/angle_service.py --camera 172.22.102.226 --port 8080

Point the rotator at it with the camera-source setting, e.g.
"http://<this host>:8080/angle".
"""

import argparse
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.request import urlopen

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import board as boardmod
from camera_model import homography_dlt


class AngleMeter:
    """Serialised, because the rotator asks one question at a time and two
    concurrent captures would only fight over the camera."""

    def __init__(self, camera, average, sign):
        self.camera = camera
        self.average = average
        self.sign = sign
        self.lock = threading.Lock()
        self.reference = None
        self.model = boardmod.model_points()[:, :2]
        self.count = 0

    def grab(self, timeout=20):
        with urlopen(f"http://{self.camera}/capture", timeout=timeout) as response:
            data = response.read()
        image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise RuntimeError("camera returned something that is not an image")
        return image

    def homography(self):
        corners = boardmod.detect(self.grab())
        if corners is None:
            raise RuntimeError("board not found in the frame")
        H = homography_dlt(self.model, corners)
        homogeneous = np.column_stack([self.model, np.ones(len(self.model))]) @ H.T
        projected = homogeneous[:, :2] / homogeneous[:, 2:3]
        residual = float(np.sqrt(np.mean(np.sum((projected - corners) ** 2, axis=1))))
        return H, residual

    def measure(self):
        with self.lock:
            angles, residuals = [], []
            for _ in range(self.average):
                H, residual = self.homography()
                if self.reference is None:
                    self.reference = np.linalg.inv(H)
                    angles.append(0.0)
                    residuals.append(residual)
                    continue
                G = self.reference @ H
                G = G / G[2, 2]
                U, _, Vt = np.linalg.svd(G[:2, :2])
                R = U @ np.diag([1.0, np.linalg.det(U @ Vt)]) @ Vt
                angles.append(np.degrees(np.arctan2(R[1, 0], R[0, 0])))
                residuals.append(residual)
            # Averaged on the circle, so a run sitting near the +/-180 seam
            # does not average to zero.
            radians = np.radians(angles)
            mean = np.degrees(np.arctan2(np.sin(radians).mean(), np.cos(radians).mean()))
            self.count += 1
            return self.sign * mean, float(np.mean(residuals))


def make_handler(meter, verbose):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass  # the measurement log below is the useful one

        def do_GET(self):
            if self.path.split("?")[0] not in ("/angle", "/"):
                self.send_error(404, "only /angle")
                return
            started = time.time()
            try:
                angle, residual = meter.measure()
            except Exception as error:  # noqa: BLE001 - any failure is "no measurement"
                # A 503 rather than a made-up number: the rotator treats a
                # non-200 as "skip this position", which is right.
                self.send_error(503, str(error))
                if verbose:
                    print(f"  FAILED: {error}", flush=True)
                return
            payload = json.dumps({"angleDeg": round(angle, 5),
                                  "frames": meter.count,
                                  "residualPx": round(residual, 4)}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            if verbose:
                print(f"  #{meter.count:4d}  {angle:+10.4f} deg   "
                      f"residual {residual:.3f} px   {time.time()-started:.1f}s", flush=True)
    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", default="172.22.102.226")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--average", type=int, default=3,
                    help="frames per answer; 3 puts the noise near 1.3 mdeg")
    ap.add_argument("--invert", action="store_true",
                    help="flip the sign, when the camera turns the other way "
                         "from the rotator's own angle")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    meter = AngleMeter(args.camera, args.average, -1.0 if args.invert else 1.0)
    # Establish the reference before announcing readiness, so the first
    # question the rotator asks is already answered against it.
    angle, residual = meter.measure()
    print(f"reference frame taken from {args.camera} (residual {residual:.3f} px)")
    server = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(meter, not args.quiet))
    print(f"serving the output-shaft angle on port {args.port}; point the rotator's "
          f"camera source at http://<this host>:{args.port}/angle")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped")


if __name__ == "__main__":
    main()
