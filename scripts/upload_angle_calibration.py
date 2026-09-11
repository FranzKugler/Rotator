"""Pushes an AS5600 correction fit computed by scripts/camera_angle_analyze.py
(its `as5600FitAgainstCamera` result - fit against the camera's independent
angle reference instead of correctSensorReading()'s only available self-
referential phase basis, see CALIBRATION_FINDINGS.md) onto the rotator, via
the expert-gated main/WebServer.cpp:/api/calibration/coefficients endpoint
(main/RotatorHW.cpp's setAngleCalCoefficients()).

Usage:
    python3 scripts/camera_angle_analyze.py --sweep-dir /tmp/sweep1 --kmax 4 --out /tmp/sweep1/analysis.json
    python3 scripts/upload_angle_calibration.py --host 172.22.102.30 --analysis /tmp/sweep1/analysis.json

--kmax 4 above is not optional: the firmware's KMAX (main/RotatorHW.h) is 4,
a fixed-size NVS blob - a fit computed at any other order will be rejected
(count mismatch) rather than silently truncated or zero-padded.

After uploading, gotoMechanicalZero() should be re-run before trusting
absolute position commands - recalibrating shifts C0, which the stored
mechanical-zero reference was measured against (same rule
calibrateAngleSensor()'s own on-device path already documents).
"""

import argparse
import json
import sys
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

EXPECTED_KMAX = 4  # main/RotatorHW.h's KMAX - see module docstring


def fetch_coefficients(host, timeout=10):
    with urlopen(f"http://{host}/api/calibration/coefficients", timeout=timeout) as resp:
        return json.loads(resp.read())


def push_coefficients(host, c0, a, b, timeout=10):
    body = json.dumps({"C0": c0, "A": a, "B": b}).encode()
    req = Request(f"http://{host}/api/calibration/coefficients", data=body, method="POST",
                  headers={"Content-Type": "application/json"})
    with urlopen(req, timeout=timeout) as resp:
        return resp.read()


def fmt_coeffs(c0, a, b):
    lines = [f"  C0 = {c0:.4f}"]
    for k in range(1, len(a)):
        lines.append(f"  A{k} = {a[k]:8.4f}   B{k} = {b[k]:8.4f}")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", required=True)
    ap.add_argument("--analysis", required=True,
                     help="the --out JSON from camera_angle_analyze.py (must contain as5600FitAgainstCamera)")
    ap.add_argument("--dry-run", action="store_true", help="print what would be uploaded, but don't send it")
    args = ap.parse_args()

    with open(args.analysis) as f:
        analysis = json.load(f)

    fit = analysis.get("as5600FitAgainstCamera")
    if not fit:
        sys.exit(f"{args.analysis} has no 'as5600FitAgainstCamera' - "
                 f"was it produced by camera_angle_analyze.py (or alpaca_random_sweep.py analyze, "
                 f"which does not compute this)?")

    c0, a, b = fit["C0"], fit["A"], fit["B"]
    if len(a) != EXPECTED_KMAX + 1 or len(b) != EXPECTED_KMAX + 1:
        sys.exit(f"fit has {len(a) - 1} harmonics, firmware KMAX is {EXPECTED_KMAX} - "
                 f"rerun camera_angle_analyze.py with --kmax {EXPECTED_KMAX}")

    print(f"Fit quality: RMS={fit['rmsDeg']:.4f} deg  peak={fit['peakDeg']:.4f} deg "
          f"(motor-shaft degrees, against the camera reference)")

    try:
        before = fetch_coefficients(args.host)
    except (HTTPError, URLError) as e:
        sys.exit(f"could not read current coefficients from {args.host}: {e}")

    print("\nCurrently on the device:")
    print(fmt_coeffs(before["C0"], before["A"], before["B"]))
    print("\nUploading:")
    print(fmt_coeffs(c0, a, b))

    if args.dry_run:
        print("\n--dry-run: not uploaded")
        return

    try:
        push_coefficients(args.host, c0, a, b)
    except (HTTPError, URLError) as e:
        sys.exit(f"\nupload failed: {e}")

    after = fetch_coefficients(args.host)

    def close(x, y, tol=1e-6):
        return abs(x - y) < tol

    mismatched = (not close(after["C0"], c0) or
                  any(not close(x, y) for x, y in zip(after["A"], a)) or
                  any(not close(x, y) for x, y in zip(after["B"], b)))
    if mismatched:
        sys.exit(f"\nuploaded, but read-back does not match:\n{fmt_coeffs(after['C0'], after['A'], after['B'])}")

    print("\nUploaded and confirmed by read-back.")
    print("Re-run gotoMechanicalZero() before trusting absolute position commands - "
          "this shifted C0, which the stored mechanical-zero reference was measured against.")


if __name__ == "__main__":
    main()
