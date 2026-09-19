"""Measure shaft angles from a capture directory with the homography-ratio
estimator.

    G_i = H_0^-1 H_i

where H_i is frame i's own model-to-image homography. When the board plane
is fixed, H_i = H_plane Rot(phi_i), the plane mapping cancels, and G_i is a
pure 2D rotation whose angle is the shaft rotation. Each frame keeps its own
homography, so anything that is per-frame about a frame - illumination
shifting the corner detector, JPEG artefacts, a nudge of the camera - is
absorbed there rather than pushed into the angle.

Cross-checked on the 41-frame 360-degree calibration sweep against a full
16-parameter rigid-rotation fit and against a shared fixed-plane fit: all
three agreed to within 1 mdeg rms of each other, which is why this one is
used - it is the cheapest and the best conditioned of the three.

The anisotropy reported alongside each angle is how far G_i is from a pure
rotation. It is the estimator's own alarm: it stays near zero while the
board plane really is fixed, and grows if the board starts moving out of
plane.
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
from camera_model import homography_dlt


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


def detect_all(directory, frames, refresh=False, quiet=False):
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
        if not quiet:
            print(f"  {record['file']}: {'ok' if found is not None else 'NOT FOUND'}",
                  flush=True)
    np.savez(path, corners=np.array(corners, dtype=object), ok=np.array(ok))
    return corners, ok


def angles_from_corners(corners, model_uv=None):
    """(angles_deg, anisotropy, homography_residual_px), frame 0 the reference."""
    if model_uv is None:
        model_uv = boardmod.model_points()[:, :2]

    def apply(H, points):
        h = np.column_stack([points, np.ones(len(points))]) @ H.T
        return h[:, :2] / h[:, 2:3]

    homographies, residual = [], []
    for points in corners:
        H = homography_dlt(model_uv, points)
        homographies.append(H)
        residual.append(float(np.sqrt(np.mean(
            np.sum((apply(H, model_uv) - points) ** 2, axis=1)))))

    reference = np.linalg.inv(homographies[0])
    angles, anisotropy = [], []
    for H in homographies:
        G = reference @ H
        G = G / G[2, 2]
        U, S, Vt = np.linalg.svd(G[:2, :2])
        R = U @ np.diag([1.0, np.linalg.det(U @ Vt)]) @ Vt
        angles.append(np.degrees(np.arctan2(R[1, 0], R[0, 0])))
        anisotropy.append(float(S[0] / S[1] - 1.0))
    # Deliberately not unwrapped: a run of random targets has no continuous
    # track to unwrap along, and everything downstream compares modulo 360.
    return np.array(angles), np.array(anisotropy), np.array(residual)


def resolve_flips(angles, reference_deg):
    """Settle the board's 180-degree labelling ambiguity (see board.py).

    Returns the corrected angles and how many frames had to be turned."""
    angles = np.asarray(angles, float)
    delta = (angles - np.asarray(reference_deg, float) + 180.0) % 360.0 - 180.0
    flipped = np.abs(delta) > 90.0
    return np.where(flipped, angles + 180.0, angles), int(flipped.sum())


def reference_angles(header, frames):
    """Whatever the run itself commanded, in output degrees."""
    if header["mode"] == "steps":
        return np.array([f["commanded"]["outputDeg"] for f in frames]), "commanded steps"
    return np.array([f["commanded"]["alpacaTargetDeg"] for f in frames]), "Alpaca target"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", required=True)
    ap.add_argument("--refresh", action="store_true")
    ap.add_argument("--export", help="write per-frame results as JSON here")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    header, frames = load_manifest(args.dir)
    corners, ok = detect_all(args.dir, frames, args.refresh, args.quiet)
    keep = [i for i, good in enumerate(ok) if good]
    if len(keep) < len(frames):
        print(f"WARNING: board not found in {len(frames)-len(keep)} frames")
    corners = [corners[i] for i in keep]
    usable = [frames[i] for i in keep]

    angles, anisotropy, residual = angles_from_corners(corners)
    commanded, label = reference_angles(header, usable)

    # Compared modulo 360, never by unwrapping both sequences. A run of
    # random targets can jump nearly half a turn between frames, and the
    # rotator may take the other way round to stay inside its cable limit,
    # so the two unwrapped tracks can land on different branches and the
    # difference then reads 360 degrees wrong. Modulo arithmetic cannot:
    # a branch error is a whole number of turns and cancels exactly.
    def residual_for(sign):
        resolved, _ = resolve_flips(sign * angles, commanded)
        difference = np.radians(resolved - commanded)
        offset = np.arctan2(np.sin(difference).mean(), np.cos(difference).mean())
        return (np.degrees(np.angle(np.exp(1j * (difference - offset)))),
                np.degrees(offset))

    candidates = [(np.sqrt(np.mean(residual_for(s)[0] ** 2)), s) for s in (1.0, -1.0)]
    _, sign = min(candidates)
    error, offset = residual_for(sign)
    signed, flips = resolve_flips(sign * angles, commanded)

    print(f"\n{len(corners)} frames, reference = {label}")
    print(f"  camera sense {'as commanded' if sign > 0 else 'reversed'}, "
          f"constant offset {offset:.4f} deg removed, "
          f"{flips} frame(s) had the board's 180-degree labelling resolved")
    print(f"  angle error vs {label}: rms {np.sqrt(np.mean(error**2))*1000:.2f} mdeg, "
          f"max {np.abs(error).max()*1000:.2f} mdeg")
    print(f"  95th percentile |error|: {np.percentile(np.abs(error),95)*1000:.2f} mdeg")
    print(f"  within 10 mdeg: {100*np.mean(np.abs(error)<=0.010):.1f} %, "
          f"within 20 mdeg: {100*np.mean(np.abs(error)<=0.020):.1f} %")
    print(f"  spread of measured angle: {signed.std()*1000:.2f} mdeg "
          f"(meaningful only when nothing moved)")
    print(f"  homography residual: mean {residual.mean():.4f} px, max {residual.max():.4f} px")
    print(f"  anisotropy: mean {anisotropy.mean():.2e}, max {np.abs(anisotropy).max():.2e}")

    if args.export:
        with open(args.export, "w") as handle:
            json.dump({"dir": args.dir, "mode": header["mode"],
                       "file": [f["file"] for f in usable],
                       "commandedDeg": commanded.tolist(),
                       "sign": sign, "offsetDeg": float(offset),
                       "measuredDeg": signed.tolist(),
                       "errorDeg": error.tolist(),
                       "anisotropy": anisotropy.tolist(),
                       "homographyResidualPx": residual.tolist(),
                       "rawSensor": [f.get("rawSensor") for f in usable],
                       "stepPosition": [f.get("stepPosition") for f in usable]},
                      handle, indent=2)
        print(f"wrote {args.export}")


if __name__ == "__main__":
    main()
