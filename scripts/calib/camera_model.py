"""Camera model, calibration and angle measurement - written against numpy
and scipy rather than OpenCV's calibration entry points.

Two reasons for not using cv2.calibrateCamera/solvePnP/findHomography here.
The practical one is that they crash: this machine's CPU (a QEMU virtual CPU
with SSE4.2 and no AVX) hits an illegal instruction inside OpenCV's
Levenberg-Marquardt path, so calibrateCamera, findHomography, solvePnP's
ITERATIVE flag and solvePnPRefineLM all die with SIGILL, while the
closed-form routines - findChessboardCornersSB, solvePnP's IPPE flag,
undistortPoints, projectPoints, Rodrigues - are fine. The better reason is
that the generic calibration throws away what we know: every view here is
the *same* board, rigidly attached to *one* shaft, turned by angles that are
exact by construction.

That knowledge collapses the problem. A generic calibration over 41 views
solves 5 intrinsics plus 6 pose parameters per view - 251 unknowns. The
rigid-rotation model below solves 16:

    f, cx, cy, k1, k2          the camera
    rvec0, t0                  where the board sits at the reference angle
    axis (2), point (3)        the rotation axis in camera coordinates

for the same 4592 residuals, and it cannot express a physically impossible
solution the way the generic one can.

Measurement then runs in the other direction: intrinsics and axis frozen,
one free parameter per frame - the angle - so what comes out is the shaft
angle and nothing else.
"""

import cv2
import numpy as np
from scipy.optimize import least_squares


# --- geometry ---------------------------------------------------------------

def rodrigues(rvec):
    R, _ = cv2.Rodrigues(np.asarray(rvec, np.float64).reshape(3, 1))
    return R


def axis_from_angles(theta, phi):
    """Unit vector from two spherical angles - a 2-parameter unit vector,
    so the optimiser cannot drift along the (meaningless) length direction."""
    return np.array([np.sin(theta) * np.cos(phi),
                     np.sin(theta) * np.sin(phi),
                     np.cos(theta)])


def angles_from_axis(axis):
    axis = axis / np.linalg.norm(axis)
    return np.arccos(np.clip(axis[2], -1.0, 1.0)), np.arctan2(axis[1], axis[0])


def rotation_about(axis, angle_deg):
    """Rodrigues' formula for a rotation about a unit axis."""
    a = np.radians(angle_deg)
    k = np.asarray(axis, np.float64)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * (K @ K)


def project(points_cam, f, cx, cy, k1, k2):
    """Pinhole with two radial distortion terms, square pixels.

    Two terms and no tangential: the board covers a limited, fixed annulus of
    this frame, and every extra term the data cannot constrain is a parameter
    free to absorb real angle error instead."""
    x = points_cam[:, 0] / points_cam[:, 2]
    y = points_cam[:, 1] / points_cam[:, 2]
    r2 = x * x + y * y
    scale = 1.0 + k1 * r2 + k2 * r2 * r2
    return np.column_stack([f * x * scale + cx, f * y * scale + cy])


# --- the rigid-rotation model ----------------------------------------------

PARAM_NAMES = ["f", "cx", "cy", "k1", "k2",
               "rvec0_x", "rvec0_y", "rvec0_z", "t0_x", "t0_y", "t0_z",
               "axis_theta", "axis_phi", "p_x", "p_y", "p_z"]


def unpack(params):
    f, cx, cy, k1, k2 = params[0:5]
    rvec0 = params[5:8]
    t0 = params[8:11]
    axis = axis_from_angles(params[11], params[12])
    point = params[13:16]
    return f, cx, cy, k1, k2, rvec0, t0, axis, point


def predict(params, model_points, angles_deg):
    """Predicted pixel positions of every corner in every view."""
    f, cx, cy, k1, k2, rvec0, t0, axis, point = unpack(params)
    base = model_points @ rodrigues(rvec0).T + t0
    out = []
    for angle in angles_deg:
        M = rotation_about(axis, angle)
        out.append(project((base - point) @ M.T + point, f, cx, cy, k1, k2))
    return out


def residuals(params, model_points, corners, angles_deg):
    predicted = predict(params, model_points, angles_deg)
    return np.concatenate([(p - c).ravel() for p, c in zip(predicted, corners)])


def initial_guess(corners, model_points, angles_deg, image_size, f_guess=1200.0):
    """Seed the fit from closed-form poses only - nothing here uses a solver
    that this CPU cannot run."""
    width, height = image_size
    K = np.array([[f_guess, 0, width / 2.0], [0, f_guess, height / 2.0], [0, 0, 1]])
    zero = np.zeros(5)

    poses = []
    for points in corners:
        ok, rvec, tvec = cv2.solvePnP(model_points.astype(np.float64),
                                      points.reshape(-1, 1, 2).astype(np.float64),
                                      K, zero, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            raise RuntimeError("IPPE pose failed on a view")
        poses.append((rodrigues(rvec), tvec.reshape(3)))

    R0, t0 = poses[0]
    # Axis direction: the rotation taking view 0 to view i is about the shaft
    # axis, so every view votes for the same direction. Views at small angles
    # vote weakly, so weight by the angle actually turned.
    votes = []
    for (R, _), angle in zip(poses[1:], angles_deg[1:]):
        rvec, _ = cv2.Rodrigues(R @ R0.T)
        rvec = rvec.reshape(3)
        norm = np.linalg.norm(rvec)
        if norm > 1e-6:
            votes.append(np.sign(angle) * rvec / norm * abs(angle))
    axis = np.sum(votes, axis=0)
    axis = axis / np.linalg.norm(axis)

    # Axis position: view i's motion is X -> M(X - p) + p, so
    # (I - M_i) p = t_i - M_i t_0 over all views, solved in least squares.
    rows, rhs = [], []
    for (R, t), angle in zip(poses[1:], angles_deg[1:]):
        M = R @ R0.T
        rows.append(np.eye(3) - M)
        rhs.append(t - M @ t0)
    point, *_ = np.linalg.lstsq(np.vstack(rows), np.concatenate(rhs), rcond=None)

    rvec0, _ = cv2.Rodrigues(R0)
    theta, phi = angles_from_axis(axis)
    return np.concatenate([[f_guess, width / 2.0, height / 2.0, 0.0, 0.0],
                           rvec0.ravel(), t0, [theta, phi], point])


def fit(corners, model_points, angles_deg, image_size, f_guess=1200.0,
        verbose=False):
    """Solve the 16-parameter model against all views at once."""
    start = initial_guess(corners, model_points, angles_deg, image_size, f_guess)
    result = least_squares(
        residuals, start, args=(model_points, corners, angles_deg),
        method="lm", max_nfev=20000, xtol=1e-14, ftol=1e-14)
    rms = float(np.sqrt(np.mean(result.fun ** 2)))
    if verbose:
        for name, value in zip(PARAM_NAMES, result.x):
            print(f"    {name:11s} {value: .6f}")
    return result.x, rms


# --- measurement ------------------------------------------------------------

def measure_angle(params, model_points, points, bracket_deg, samples=9):
    """Shaft angle for one frame, with the camera and the axis held fixed.

    One free parameter, so the answer is an angle and cannot quietly become a
    pose that fits better by moving the board somewhere it cannot go. The
    coarse bracket scan first is what keeps a large rotation from converging
    onto a neighbouring local minimum."""
    def cost(angle):
        predicted = predict(params, model_points, [float(angle)])[0]
        return (predicted - points).ravel()

    grid = np.linspace(bracket_deg - 4.0, bracket_deg + 4.0, samples)
    scores = [np.sum(cost(g) ** 2) for g in grid]
    seed = grid[int(np.argmin(scores))]
    result = least_squares(cost, [seed], method="lm", xtol=1e-14, ftol=1e-14)
    return float(result.x[0]), float(np.sqrt(np.mean(result.fun ** 2)))


def measure_all(params, model_points, corners, bracket_deg):
    angles, residual = [], []
    for points, bracket in zip(corners, bracket_deg):
        angle, rms = measure_angle(params, model_points, points, bracket)
        angles.append(angle)
        residual.append(rms)
    return np.array(angles), np.array(residual)


# --- the fixed-plane model --------------------------------------------------
#
# The rigid-rotation model above is the general one, and on this rig it is
# degenerate: fitted live on 2026-09-18, the board plane came out 0.020
# degrees from perpendicular to the rotation axis, which is as perpendicular
# as makes no difference. A planar target whose orientation never changes
# gives no leverage on focal length, and the fit duly ran away to f = 8000 px
# with the principal point 8000 px outside the frame while still reproducing
# the corners to 0.29 px.
#
# But a board that stays in one fixed physical plane is the easy case, not
# the hard one. The plane-to-image mapping is then a single fixed homography,
# and the shaft's rotation is an ordinary 2D rotation inside the plane. No
# focal length, no principal point and no pose ever has to be separated out -
# the homography carries all of it, and only the lens distortion has to be
# modelled explicitly, because it is the one part that is not a homography.
#
#     plane point   q(phi) = Rot(phi) (m - c) + c        m = board corner
#     ideal pixel   p_u    = H q                          (8 dof)
#     observed      p      = pc + (p_u - pc) (1 + d1 r^2 + d2 r^4)
#
# c is where the rotation axis pierces the board - not the centre of the
# chessboard and not the centre of the image, both of which this measures
# rather than assumes.

DISTORTION_SCALE = 1000.0  # pixels; keeps d1, d2 near unity for conditioning

PLANE_PARAM_NAMES = (["H%d" % i for i in range(8)] +
                     ["c_u", "c_v", "pc_x", "pc_y", "d1", "d2"])


def plane_unpack(params):
    H = np.append(params[0:8], 1.0).reshape(3, 3)
    centre = params[8:10]
    pc = params[10:12]
    d1, d2 = params[12], params[13]
    return H, centre, pc, d1, d2


def rot2d(angle_deg):
    a = np.radians(angle_deg)
    return np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])


def plane_predict_one(params, model_uv, angle_deg):
    H, centre, pc, d1, d2 = plane_unpack(params)
    q = (model_uv - centre) @ rot2d(angle_deg).T + centre
    homogeneous = np.column_stack([q, np.ones(len(q))]) @ H.T
    ideal = homogeneous[:, :2] / homogeneous[:, 2:3]
    delta = ideal - pc
    r2 = np.sum(delta * delta, axis=1) / (DISTORTION_SCALE ** 2)
    return pc + delta * (1.0 + d1 * r2 + d2 * r2 * r2)[:, None]


def plane_residuals(params, model_uv, corners, angles_deg):
    return np.concatenate([(plane_predict_one(params, model_uv, angle) - points).ravel()
                           for points, angle in zip(corners, angles_deg)])


def homography_dlt(source, target):
    """Plain normalised DLT - cv2.findHomography is one of the routines that
    dies with SIGILL on this CPU, and the data here is clean enough that the
    linear solution needs no robust fitting."""
    def normalise(points):
        centre = points.mean(axis=0)
        shifted = points - centre
        scale = np.sqrt(2.0) / np.sqrt(np.mean(np.sum(shifted ** 2, axis=1)))
        T = np.array([[scale, 0, -scale * centre[0]],
                      [0, scale, -scale * centre[1]], [0, 0, 1]])
        return shifted * scale, T

    src, Ts = normalise(np.asarray(source, float))
    dst, Td = normalise(np.asarray(target, float))
    rows = []
    for (x, y), (u, v) in zip(src, dst):
        rows.append([-x, -y, -1, 0, 0, 0, u * x, u * y, u])
        rows.append([0, 0, 0, -x, -y, -1, v * x, v * y, v])
    _, _, Vt = np.linalg.svd(np.array(rows))
    H = Vt[-1].reshape(3, 3)
    H = np.linalg.inv(Td) @ H @ Ts
    return H / H[2, 2]


def plane_fit(corners, model_uv, angles_deg, image_size, max_nfev=4000):
    H = homography_dlt(model_uv, corners[0])
    start = np.concatenate([H.ravel()[:8], model_uv.mean(axis=0),
                            [image_size[0] / 2.0, image_size[1] / 2.0], [0.0, 0.0]])
    result = least_squares(plane_residuals, start,
                           args=(model_uv, corners, angles_deg),
                           method="trf", max_nfev=max_nfev, xtol=1e-14, ftol=1e-14)
    return result.x, float(np.sqrt(np.mean(result.fun ** 2)))


def plane_measure(params, model_uv, points, bracket_deg, span=5.0, samples=11):
    def cost(angle):
        return (plane_predict_one(params, model_uv, float(angle)) - points).ravel()

    grid = np.linspace(bracket_deg - span, bracket_deg + span, samples)
    seed = grid[int(np.argmin([np.sum(cost(g) ** 2) for g in grid]))]
    result = least_squares(cost, [seed], method="lm", xtol=1e-14, ftol=1e-14)
    return float(result.x[0]), float(np.sqrt(np.mean(result.fun ** 2)))


def plane_measure_all(params, model_uv, corners, bracket_deg, span=5.0):
    angles, residual = [], []
    for points, bracket in zip(corners, bracket_deg):
        angle, rms = plane_measure(params, model_uv, points, bracket, span)
        angles.append(angle)
        residual.append(rms)
    return np.array(angles), np.array(residual)
