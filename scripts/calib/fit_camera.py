"""Fit the camera against the rotator's own exact motion, then report how
well it reproduces that motion.

Run it on a capture_frames.py --mode steps directory, where the commanded
increments are whole full steps and therefore exact.
"""

import argparse
import json
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import board as boardmod
import camera_model as cm
from measure_frames import resolve_flips


def load_manifest(directory):
    header, frames = None, []
    with open(os.path.join(directory, "manifest.jsonl")) as handle:
        for line in handle:
            record = json.loads(line)
            if record.get("record") == "__header__":
                header = record
            else:
                frames.append(record)
    return header, frames


def detect_all(directory, frames, refresh=False):
    path = os.path.join(directory, "corners.npz")
    if os.path.exists(path) and not refresh:
        stored = np.load(path, allow_pickle=True)
        return ([np.asarray(c, np.float64) for c in stored["corners"]],
                list(stored["ok"]))
    corners, ok = [], []
    for record in frames:
        image = cv2.imread(os.path.join(directory, record["file"]))
        found = boardmod.detect(image)
        corners.append(found if found is not None else np.zeros((0, 2)))
        ok.append(found is not None)
        print(f"  {record['file']}: {'ok' if found is not None else 'NOT FOUND'}",
              flush=True)
    np.savez(path, corners=np.array(corners, dtype=object), ok=np.array(ok))
    return corners, ok


def _seed_angle(corners, index, model):
    """Rough in-plane angle of one frame relative to frame 0, from plain
    homographies - enough to tell a half turn from no half turn."""
    from camera_model import homography_dlt
    reference = np.linalg.inv(homography_dlt(model[:, :2], corners[0]))
    G = reference @ homography_dlt(model[:, :2], corners[index])
    U, _, Vt = np.linalg.svd(G[:2, :2] / G[2, 2])
    R = U @ np.diag([1.0, np.linalg.det(U @ Vt)]) @ Vt
    return np.degrees(np.arctan2(R[1, 0], R[0, 0]))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dir", required=True)
    ap.add_argument("--export", help="write the fitted camera here")
    ap.add_argument("--refresh", action="store_true", help="redo corner detection")
    args = ap.parse_args()

    header, frames = load_manifest(args.dir)
    corners, ok = detect_all(args.dir, frames, args.refresh)
    keep = [i for i, good in enumerate(ok) if good]
    corners = [corners[i] for i in keep]
    commanded = np.array([frames[i]["commanded"]["outputDeg"] for i in keep])
    print(f"{len(keep)}/{len(frames)} frames usable, commanded "
          f"{commanded.min():.1f} .. {commanded.max():.1f} deg")

    image = cv2.imread(os.path.join(args.dir, frames[keep[0]]["file"]))
    size = (image.shape[1], image.shape[0])
    model = boardmod.model_points()

    # The board's 180-degree labelling ambiguity (see board.py) reaches this
    # script too: a flipped frame would be fitted as a board turned half a
    # turn from where it is. Settled the same way as everywhere else, against
    # the commanded angles, before anything is fitted.
    flipped = [i for i in range(len(corners))
               if abs(((_seed_angle(corners, i, model) - commanded[i] + 180.0)
                       % 360.0) - 180.0) > 90.0]
    if flipped:
        print(f"  {len(flipped)} frame(s) need the 180-degree labelling resolved; "
              f"reversing their corner order")
        for i in flipped:
            corners[i] = corners[i][::-1]

    # The camera's sense of rotation is not known in advance; fit both and
    # keep whichever explains the frames.
    best = None
    for sign in (1.0, -1.0):
        angles = sign * (commanded - commanded[0])
        params, rms = cm.fit(corners, model, angles, size)
        print(f"  sign {sign:+.0f}: reprojection rms {rms:.4f} px")
        if best is None or rms < best[1]:
            best = (params, rms, sign, angles)
    params, rms, sign, angles = best

    print(f"\nfitted camera (reprojection rms {rms:.4f} px, "
          f"{len(corners)*len(model)} corners)")
    f, cx, cy, k1, k2, rvec0, t0, axis, point = cm.unpack(params)
    print(f"  focal length   {f:9.2f} px")
    print(f"  centre         {cx:9.2f}, {cy:.2f}  "
          f"(image centre {size[0]/2:.1f}, {size[1]/2:.1f})")
    print(f"  k1, k2         {k1:9.5f}, {k2:.5f}")
    print(f"  axis in camera {np.array2string(axis, precision=4)}")
    print(f"  board tilt to axis "
          f"{np.degrees(np.arccos(abs(np.dot(axis, rodrigues_z(rvec0))))):.3f} deg")

    measured, point_rms = cm.measure_all(params, model, corners, angles)
    error = (measured - angles) - (measured - angles).mean()
    print(f"\nangle reproduction over {commanded.max()-commanded.min():.0f} deg:")
    print(f"  rms   {np.sqrt(np.mean(error**2))*1000:8.2f} mdeg")
    print(f"  max   {np.abs(error).max()*1000:8.2f} mdeg")
    print(f"  per-frame corner residual: mean {point_rms.mean():.4f} px, "
          f"max {point_rms.max():.4f} px")
    print(f"\n{'commanded':>10} {'measured':>11} {'error mdeg':>11} {'px rms':>8}")
    for index in range(len(angles)):
        print(f"{angles[index]:10.3f} {measured[index]:11.4f} "
              f"{error[index]*1000:11.2f} {point_rms[index]:8.4f}")

    if args.export:
        with open(args.export, "w") as handle:
            json.dump({"params": params.tolist(), "paramNames": cm.PARAM_NAMES,
                       "sign": sign, "imageSize": list(size),
                       "reprojectionRmsPx": rms,
                       "angleRmsMdeg": float(np.sqrt(np.mean(error**2)) * 1000),
                       "source": args.dir}, handle, indent=2)
        print(f"\nwrote {args.export}")


def rodrigues_z(rvec):
    """Board-plane normal in camera coordinates - the third column of R."""
    return cm.rodrigues(rvec)[:, 2]


if __name__ == "__main__":
    main()
