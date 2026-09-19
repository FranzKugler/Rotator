"""Chessboard detection and camera-referenced angle extraction for the fresh
calibration campaign.

Measurement principle
---------------------
The board is rigid on the output shaft, so its corners never move in board
coordinates - the rotation has to come out of the board's *pose*. For each
frame we solve the planar pose (R_i, t_i) of the board in the camera, then

    dR_i = R_i @ R_0.T

is the rigid rotation the shaft performed between the reference frame and
frame i. Its rotation angle is the output-shaft angle, and its axis is the
rotator axis expressed in camera coordinates.

Two consequences worth stating, because they decide what has to be
calibrated and what does not:

  * the angle of dR_i is independent of where the axis sits, so neither the
    image centre nor the board's centring on the shaft enters the result.
    Nothing has to be measured about the axis position;
  * it is *not* independent of the intrinsics. Focal length, principal point
    and lens distortion all bias the recovered pose, so those do have to be
    calibrated. That is the whole reason the camera calibration step exists.
"""

import numpy as np
import cv2

PATTERN = (8, 7)  # interior corners (cols, rows) - 9x8 squares
SQUARE = 1.0      # board units; only the angle matters, so the scale is free

SUBPIX_WIN = (7, 7)
SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 60, 1e-4)


def model_points(pattern=PATTERN, square=SQUARE):
    """Board-frame coordinates of the interior corners, z = 0."""
    cols, rows = pattern
    grid = np.zeros((rows * cols, 3), np.float64)
    grid[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    return grid


def detect(image, pattern=PATTERN, refine=True):
    """Return the (N,2) interior corners, or None if the board is not found.

    findChessboardCornersSB is the primary detector: it is markedly more
    reliable than the classic one on this camera's soft, noisy frames, and
    already returns near-subpixel corners. The classic detector plus
    cornerSubPix is kept as the fallback.
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    ok, corners = cv2.findChessboardCornersSB(
        gray, pattern, cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    if not ok:
        ok, corners = cv2.findChessboardCorners(
            gray, pattern,
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok:
            return None
        if refine:
            corners = cv2.cornerSubPix(gray, corners, SUBPIX_WIN, (-1, -1),
                                       SUBPIX_CRITERIA)
    return corners.reshape(-1, 2).astype(np.float64)


# The 180-degree labelling ambiguity, and why it is resolved elsewhere.
#
# The 8x7 interior-corner grid maps onto itself under a half turn - corner
# (i, j) goes to (7-i, 6-j), another corner of the same grid - so pairing
# the detector's corner list with the model grid is only ever determined up
# to that half turn. Both pairings fit a homography perfectly and neither
# looks worse by any geometric measure, because they differ by exactly a
# 180-degree rotation, which is a rotation like any other.
#
# Only the board's colours break the tie, and findChessboardCornersSB does
# use them - but not dependably here: on one 61-frame sweep its ordering
# flipped on some frames and not others.
#
# Two things that look like fixes are not. Re-ordering each frame to
# whichever pairing sits closer to the previous frame works only while
# consecutive frames are a few degrees apart; on a run of random targets it
# is a coin toss, and measured 68 degrees rms against commanded angles
# where the unmodified ordering measured 0.022. And no self-consistency
# check can help, since the wrong pairing is perfectly self-consistent.
#
# So it is resolved in measure_frames.resolve_flips() against the rotator's
# own reported position. That is sound rather than circular: the choice is
# between two options 180 degrees apart, the reference is good to a few
# hundredths of a degree, and picking the near one cannot shift the fine
# angle it then reports.
def pose(corners, camera_matrix, dist_coeffs, pattern=PATTERN, square=SQUARE):
    """Planar pose of the board: returns (R, t), R a 3x3 rotation matrix."""
    ok, rvec, tvec = cv2.solvePnP(
        model_points(pattern, square), corners.reshape(-1, 1, 2),
        camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise RuntimeError("solvePnP failed")
    # One IPPE-seeded refinement: the planar problem has a twofold pose
    # ambiguity at low tilt, and the refinement pulls a marginally-wrong
    # branch back onto the true one before it can corrupt an angle.
    rvec, tvec = cv2.solvePnPRefineLM(
        model_points(pattern, square), corners.reshape(-1, 1, 2),
        camera_matrix, dist_coeffs, rvec, tvec)
    R, _ = cv2.Rodrigues(rvec)
    return R, tvec.reshape(3)


def relative_rotation(R_ref, R):
    """Rotation from the reference pose to this one: (angle_deg, axis_unit)."""
    dR = R @ R_ref.T
    rvec, _ = cv2.Rodrigues(dR)
    rvec = rvec.reshape(3)
    angle = np.linalg.norm(rvec)
    axis = rvec / angle if angle > 0 else np.array([0.0, 0.0, 1.0])
    return np.degrees(angle), axis


def signed_angles(rotations, R_ref, axis_hint=None):
    """Angles of a whole run, signed consistently and unwrapped.

    Rodrigues always reports a non-negative angle about whichever axis
    direction makes it so, which flips the axis when the shaft turns the
    other way. Projecting the rotation vector on one fixed axis restores the
    sign; the axis used is the mean of the run's own axes, which is the
    rotator axis in camera coordinates.
    """
    vectors = []
    for R in rotations:
        rvec, _ = cv2.Rodrigues(R @ R_ref.T)
        vectors.append(rvec.reshape(3))
    vectors = np.array(vectors)

    if axis_hint is None:
        # Sign-align the vectors against the largest one before averaging,
        # so opposite-sign rotations do not cancel into a meaningless mean.
        anchor = vectors[np.argmax(np.linalg.norm(vectors, axis=1))]
        aligned = np.where((vectors @ anchor)[:, None] < 0, -vectors, vectors)
        axis_hint = aligned.mean(axis=0)
        axis_hint = axis_hint / np.linalg.norm(axis_hint)

    return np.degrees(vectors @ axis_hint), axis_hint
