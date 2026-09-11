"""Drives the rotator to a sequence of random positions using ONLY the
Alpaca REST API (scripts/alpaca_client.py) - never main/WebServer.cpp's
/api/debug/* routes - and independently verifies each stop against
RotatorCam, the sibling camera project's output-shaft-referenced view (see
scripts/camera_client.py). Meant to run after scripts/alpaca_conformance.py
passes: that suite checks the protocol is implemented correctly; this
checks the reported Position is actually where the rotator physically is.

Two phases, split like scripts/camera_angle_sweep.py / camera_angle_analyze.py:

`capture` - stdlib only, drives the live rotator + camera:
    python3 scripts/alpaca_random_sweep.py capture --rotator-host 172.22.102.30 \\
        --camera-host 172.22.102.226 --out-dir /tmp/alpaca_sweep --n 50

`analyze` - needs opencv/numpy (its own venv, see
scripts/requirements-camera-analysis.txt - not part of the ESP-IDF dev
container's pinned Python env):
    python3 -m venv /tmp/camvenv && /tmp/camvenv/bin/pip install -r scripts/requirements-camera-analysis.txt
    /tmp/camvenv/bin/python scripts/alpaca_random_sweep.py analyze --sweep-dir /tmp/alpaca_sweep

Why random RELATIVE steps (Move()), not random absolute MoveAbsolute()
targets directly: the camera measures rotation between CONSECUTIVE
photographed frames via checkerboard-corner correspondence (see
camera_angle_analyze.py's track_rotation()) - a closed-form single-angle fit
that is fundamentally ambiguous beyond +/-180 deg (it cannot tell a 190 deg
turn from a -170 deg one). Bounding each individual visit's requested delta
well under that (--max-step, default 15 deg) keeps every frame-to-frame
step unambiguous; 50 such visits still cover a broad, effectively random
spread of positions across the rotator's whole +/-190 deg range, exactly
like scripts/backlash_and_random_validation.py's `random` command already
does for its own (non-Alpaca) purposes - this is the Alpaca-only equivalent.

Two things Franz asked to change after the first run (RMS 0.19 deg, higher
than this project's earlier closed-loop figures suggested it should be):

1. `--settle-time` (default 10s) between the move finishing and the photo -
   RotatorHW.cpp's holdTask() keeps correcting for drift after a move
   completes (this session's continuous-holding feature), so the position
   right when Move() returns is not necessarily its final, settled one.
2. The "ideal" curve compared against the camera is now the REQUESTED
   delta (what was asked for), not the Position Alpaca reports back
   afterwards - the two should agree, but comparing the camera against the
   request itself is the more independent check and does not let a shared
   blind spot between "what we asked for" and "what got reported" hide
   inside an agreeing-with-itself number.
"""

import argparse
import json
import os
import random
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from alpaca_client import AlpacaClient  # noqa: E402
from camera_client import capture as camera_capture  # noqa: E402


def run_capture(rotator_host, camera_host, out_dir, n, max_step_deg, seed, capture_timeout, settle_time):
    os.makedirs(out_dir, exist_ok=True)
    client = AlpacaClient(rotator_host)
    client.put_connected(True)
    rng = random.Random(seed)

    print(f"[capture] {n} random Alpaca Move() visit(s), step<=+/-{max_step_deg} deg, seed={seed}, "
          f"settle={settle_time}s")

    frames = []
    cumulative_ideal_deg = 0.0
    t0 = time.time()

    def snapshot(index, ideal_deg, alpaca_position, requested_delta, reported_delta):
        img_name = f"visit_{index:04d}.jpg"
        img_path = os.path.join(out_dir, img_name)
        ok = True
        try:
            jpeg = camera_capture(camera_host, timeout=capture_timeout)
            with open(img_path, "wb") as f:
                f.write(jpeg)
        except Exception as e:  # noqa: BLE001
            print(f"  !! capture failed at visit {index}: {e}")
            ok = False
        frames.append({
            "index": index,
            "image": img_name if ok else None,
            # The value compared against the camera - what was ASKED for,
            # not what Alpaca reported back. See module docstring point 2.
            "idealOutputDeg": ideal_deg,
            "alpacaPosition": alpaca_position,
            "requestedDeltaDeg": requested_delta,
            # Informational only (should match requestedDeltaDeg closely) -
            # not used for the ideal curve above.
            "reportedDeltaDeg": reported_delta,
        })

    # Frame 0: wherever Alpaca already is - the tracker's own angle=0 anchor.
    # Give holdTask() a chance to settle here too, same as every other visit.
    time.sleep(settle_time)
    snapshot(0, 0.0, client.get_position(), None, None)

    for i in range(1, n + 1):
        delta = rng.uniform(-max_step_deg, max_step_deg)
        before = client.get_position()
        client.move(delta)
        client.wait_until_stopped()
        # Let RotatorHW.cpp's continuous holdTask() finish correcting for
        # any post-move drift before judging where the rotator actually is -
        # see module docstring point 1.
        time.sleep(settle_time)
        after = client.get_position()
        reported_delta = ((after - before + 180.0) % 360.0) - 180.0
        cumulative_ideal_deg += delta

        snapshot(i, cumulative_ideal_deg, after, delta, reported_delta)
        if i % 10 == 0 or i == n:
            print(f"  visit {i:3d}/{n}  cumulative~{cumulative_ideal_deg:+.3f} deg  "
                  f"requested={delta:+.2f} reported={reported_delta:+.2f} deg  "
                  f"({time.time() - t0:.1f}s elapsed)")

    client.put_connected(False)

    manifest = {
        "kind": "alpaca_random_sweep",
        "n": n,
        "maxStepDeg": max_step_deg,
        "seed": seed,
        "settleTimeS": settle_time,
        "frames": frames,
    }
    with open(os.path.join(out_dir, "manifest.json"), "w") as f:
        json.dump(manifest, f, indent=2)
    print(f"  wrote {out_dir}/manifest.json ({time.time() - t0:.1f}s total)")
    return manifest


def run_analyze(sweep_dir, pattern_size):
    # Imported here, not at module level - this phase needs opencv/numpy,
    # which the capture phase (and the ESP-IDF dev container's pinned
    # Python env) deliberately does not depend on. See module docstring.
    from camera_angle_analyze import track_rotation  # noqa: E402

    with open(os.path.join(sweep_dir, "manifest.json")) as f:
        manifest = json.load(f)
    frames = manifest["frames"]

    print(f"[analyze] {len(frames)} frames, pattern inner-corner grid={pattern_size}")
    tracked = track_rotation(sweep_dir, frames, pattern_size)
    tracked_by_index = {t["index"]: t for t in tracked}

    rows = []
    for fr in frames:
        t = tracked_by_index.get(fr["index"])
        if t is None:
            continue
        rows.append({
            "index": fr["index"],
            "idealOutputDeg": fr["idealOutputDeg"],
            "cameraOutputDeg": t["cameraOutputDeg"],
            "residualPx": t["residualPx"],
        })
    print(f"  {len(rows)}/{len(frames)} frames usable")
    if len(rows) < len(frames) * 0.9:
        print("  !! more than 10% of frames failed detection - treat results as provisional")

    # Anchor the camera curve's offset to frame 0, same idiom as
    # camera_angle_analyze.py's analyze() - only the shape matters after that.
    offset = rows[0]["idealOutputDeg"] - rows[0]["cameraOutputDeg"]
    for r in rows:
        r["cameraOutputDeg"] += offset
        r["residualDeg"] = r["cameraOutputDeg"] - r["idealOutputDeg"]

    residuals = [r["residualDeg"] for r in rows]
    rms = (sum(e * e for e in residuals) / len(residuals)) ** 0.5
    peak = max(abs(e) for e in residuals)
    print(f"  Alpaca Position vs. camera ground truth: RMS={rms:.4f} deg  peak={peak:.4f} deg  "
          f"over {len(rows)} visits")

    worst = sorted(rows, key=lambda r: -abs(r["residualDeg"]))[:5]
    print("  worst 5 visits:")
    for r in worst:
        print(f"    visit {r['index']:3d}: alpaca={r['idealOutputDeg']:+8.3f} deg  "
              f"camera={r['cameraOutputDeg']:+8.3f} deg  residual={r['residualDeg']:+7.4f} deg")

    result = {"sweepDir": sweep_dir, "rows": rows, "rmsDeg": rms, "peakDeg": peak}
    out_path = os.path.join(sweep_dir, "analysis.json")
    with open(out_path, "w") as f:
        json.dump(result, f, indent=2)
    print(f"  wrote {out_path}")
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="command", required=True)

    cap = sub.add_parser("capture")
    cap.add_argument("--rotator-host", default="172.22.102.30")
    cap.add_argument("--camera-host", default="172.22.102.226")
    cap.add_argument("--out-dir", required=True)
    cap.add_argument("--n", type=int, default=50)
    cap.add_argument("--max-step", type=float, default=15.0,
                      help="max output degrees per random Move() - see module docstring for why this must "
                           "stay well under 180 deg")
    cap.add_argument("--seed", type=int, default=42)
    cap.add_argument("--capture-timeout", type=float, default=15.0)
    cap.add_argument("--settle-time", type=float, default=10.0,
                      help="seconds to wait after a move finishes, before the photo, so holdTask() "
                           "(continuous PI position holding) can settle")

    an = sub.add_parser("analyze")
    an.add_argument("--sweep-dir", required=True)
    an.add_argument("--cols", type=int, default=9, help="printed squares across (generate_pattern.py default: 9)")
    an.add_argument("--rows", type=int, default=8, help="printed squares down (generate_pattern.py default: 8)")

    args = ap.parse_args()

    if args.command == "capture":
        run_capture(args.rotator_host, args.camera_host, args.out_dir, args.n, args.max_step,
                    args.seed, args.capture_timeout, args.settle_time)
    elif args.command == "analyze":
        run_analyze(args.sweep_dir, (args.cols - 1, args.rows - 1))


if __name__ == "__main__":
    main()
