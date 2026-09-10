"""Runs camera_angle_analyze.py's corner tracking over a
backlash_and_random_validation.py capture set and reports the camera-vs-ideal
residual per visit, plus (for `backlash` manifests specifically) the cw-vs-ccw
difference at each nominal position - the actual output-stage backlash
number, independent of the AS5600.

Needs numpy/opencv - run in the same venv as camera_angle_analyze.py.

Usage:
    python3 scripts/backlash_camera_analyze.py --dir /tmp/backlash
    python3 scripts/backlash_camera_analyze.py --dir /tmp/random100
"""

import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from camera_angle_analyze import find_corners, track_rotation  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", required=True)
    ap.add_argument("--cols", type=int, default=9)
    ap.add_argument("--rows", type=int, default=8)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    with open(os.path.join(args.dir, "manifest.json")) as f:
        manifest = json.load(f)
    records = manifest["records"]
    pattern_size = (args.cols - 1, args.rows - 1)

    # track_rotation() expects each frame dict to have "image" and "index" -
    # already the case for these manifests.
    tracked = track_rotation(args.dir, records, pattern_size)
    tracked_by_index = {t["index"]: t for t in tracked}
    print(f"  {len(tracked)}/{len(records)} frames usable")

    rows = []
    for rec in records:
        t = tracked_by_index.get(rec["index"])
        if t is None:
            continue
        rows.append({
            "index": rec["index"], "approach": rec["approach"],
            "idealOutputDeg": rec["idealOutputDeg"], "cameraOutputDeg": t["cameraOutputDeg"],
            "as5600ConfirmedErrorDeg": rec["as5600ConfirmedErrorDeg"],
        })

    offset = rows[0]["idealOutputDeg"] - rows[0]["cameraOutputDeg"]
    for r in rows:
        r["cameraOutputDeg"] += offset
        r["residualDeg"] = r["cameraOutputDeg"] - r["idealOutputDeg"]

    resid = [r["residualDeg"] for r in rows]
    rms = math.sqrt(sum(x * x for x in resid) / len(resid))
    peak = max(abs(x) for x in resid)
    print(f"  camera residual (vs. commanded ideal): RMS={rms:.4f} deg  peak={peak:.4f} deg")

    if manifest["kind"] == "backlash":
        print("  per-position backlash (ccw camera angle - cw camera angle, same nominal target):")
        diffs = []
        for i in range(0, len(rows), 2):
            cw, ccw = rows[i], rows[i + 1]
            assert cw["approach"] == "cw" and ccw["approach"] == "ccw", "unexpected record order"
            diff = ccw["cameraOutputDeg"] - cw["cameraOutputDeg"]
            diffs.append(diff)
            print(f"    target~{cw['idealOutputDeg']:+7.3f} deg: cw={cw['cameraOutputDeg']:+.4f}  "
                  f"ccw={ccw['cameraOutputDeg']:+.4f}  backlash={diff:+.4f} deg")
        mean_bl = sum(diffs) / len(diffs)
        rms_bl = math.sqrt(sum((d - mean_bl) ** 2 for d in diffs) / len(diffs))
        print(f"  mean backlash={mean_bl:+.4f} deg  (position-to-position spread, RMS={rms_bl:.4f} deg)  "
              f"peak={max(abs(d) for d in diffs):.4f} deg")

    if manifest["kind"] == "random":
        cw_resid = [r["residualDeg"] for r in rows if r["approach"] == "cw"]
        ccw_resid = [r["residualDeg"] for r in rows if r["approach"] == "ccw"]

        def stats(xs):
            if not xs:
                return None
            m = sum(xs) / len(xs)
            return m, math.sqrt(sum((x - m) ** 2 for x in xs) / len(xs)), max(abs(x) for x in xs)

        for label, xs in (("cw", cw_resid), ("ccw", ccw_resid)):
            m, sd, pk = stats(xs)
            print(f"  {label} approach (n={len(xs)}): mean={m:+.4f} deg  stdev={sd:.4f} deg  peak={pk:.4f} deg")
        print(f"  cw mean - ccw mean = {stats(cw_resid)[0] - stats(ccw_resid)[0]:+.4f} deg "
              f"(a directional accuracy bias, if consistently nonzero, is another way backlash shows up)")

    if args.out:
        with open(args.out, "w") as f:
            json.dump({"kind": manifest["kind"], "rows": rows}, f, indent=2)
        print(f"  wrote {args.out}")


if __name__ == "__main__":
    main()
