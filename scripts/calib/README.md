# Rotator calibration campaign

A from-scratch calibration of the rotator's angle chain, built 2026-09-18.
Everything here drives real hardware over the network; nothing is simulated
except where a script says so.

    rotator   172.22.102.30      expert mode must be unlocked
    camera    172.22.102.226     RotatorCam, GET /capture

## The chain, and where each part is measured

    stepper --- AS5600 --- 10:1 reduction --- output shaft --- chessboard
                  |                                               |
            sensor sees this                             camera sees this

The AS5600 sits on the motor shaft, so it measures the motor and only
*infers* the output. That split decides the whole campaign: everything
upstream of the reduction can be calibrated from the sensor alone, and
everything downstream cannot be seen by it at all.

## Order of operations

| Step | Script | What it establishes |
|---|---|---|
| 1 | `sweep_fullstep.py` | raw sensor at each of the 400 full steps, forward and reverse. The firmware now does the same thing itself - `GET /api/calibration/angle/stream` - so this is the research version, not the only way |
| 2 | `fit_sensor.py` | Fourier correction, order chosen from the data; hysteresis and repeatability |
| 3 | `plot_sensor.py` | the calibration graphs |
| 4 | `upload_coefficients.py` | writes the fit to the device without moving its zero |
| 5 | `capture_frames.py --mode steps` | camera frames at exactly known positions |
| 6 | `fit_camera.py` | camera geometry (see the caveat below) |
| 7 | `gear_error.py` + `plot_gear.py` | what the sensor cannot see |
| 8 | `control_lab.py` | P / PI / PD / PID compared on the real machine |
| 8b | `log_tail.py` | drains the device's /log ring buffer, which wraps in ~30 s |
| 9 | `capture_frames.py --mode alpaca` + `measure_frames.py` | the end-to-end test run |
| 10 | `error_budget.py` | splits that run into loop / backlash / gear shape, cross-validated |
| 10b | `angle_service.py` | serves the output-shaft angle over HTTP, so the device can calibrate against it by itself |
| 11 | `plot_validation.py` | the test-run graph |

`rotator_io.py`, `board.py`, `camera_model.py`, `gear_model.py` and
`vizstyle.py` are the shared layers; none of them is a program.

`capture_frames.py --mode alpaca` takes two options that exist because of
what step 10 found: `--approach-deg` reaches every target from the same
side, which removes the gear backlash the firmware's loop cannot see, and
`--precompensate` shifts each target by a measured gear model so the shaft
lands where the target says rather than where the sensor says.

## Three things that will bite

**OpenCV's solver path crashes here.** This machine's CPU is a QEMU virtual
CPU with SSE4.2 and no AVX, and OpenCV's Levenberg-Marquardt code hits an
illegal instruction on it. `cv2.calibrateCamera`, `cv2.findHomography`,
`cv2.solvePnP` with `SOLVEPNP_ITERATIVE` and `cv2.solvePnPRefineLM` all die
with SIGILL - no exception, no traceback, just a dead process. The
closed-form routines are fine: `findChessboardCornersSB`, `solvePnP` with
`SOLVEPNP_IPPE`, `undistortPoints`, `projectPoints`, `Rodrigues`.
`camera_model.py` therefore carries its own DLT homography and its own
least-squares fits.

**The board's orientation is ambiguous by 180 degrees.** The 8x7 interior-
corner grid maps onto itself under a half turn, so both pairings fit
perfectly and no geometric test can separate them. `board.py` explains it;
`measure_frames.resolve_flips()` settles it against the rotator's own
reported position.

**Focal length is not observable on this rig.** The board plane sits within
0.1 degrees of perpendicular to the rotation axis, so rotating it produces
no new plane orientations and the focal length is free. A first fit ran away
to f = 8000 px while still reproducing corners to 0.29 px. This is not a
problem to solve - a board that stays in one fixed plane is the easy case,
and the homography-ratio estimator in `measure_frames.py` never needs a
focal length. `fit_camera.py` is kept for the diagnosis, not for production.

## Letting the device calibrate itself against the camera

The firmware can run the output-angle calibration on its own - a button in
the web UI, `GET /api/calibration/camera/stream` underneath - but it does no
image processing. It asks an HTTP address for a number. `angle_service.py`
is what answers, until RotatorCam serves angles itself:

    python3 scripts/calib/angle_service.py --camera 172.22.102.226 --port 8080

Then set the device's angle source (web UI, or POST
`/api/calibration/camera-source`) to `http://<that host>:8080/angle` and
press the button.

**It has to be reachable from the rotator, which the dev container is not.**
The container sits on a Docker bridge (172.23.0.0/16) that the rotator's
network does not route to, and `compose.yaml` publishes only the Vite port,
bound to localhost. Run the service on the host instead - it needs numpy and
opencv, the same throwaway venv as the rest of the analysis - or publish a
port from the container deliberately.

If the sign comes out inverted the device will refuse the result rather than
store it: a correction that big cannot be a gear error, and it says so.
Re-run the service with `--invert`.

## Environments

Acquisition is standard library only and runs under the container's own
python3. Analysis needs numpy, scipy, opencv and matplotlib in a throwaway
venv, per the note in `../requirements-camera-analysis.txt`:

    python3 -m venv /tmp/cv4
    /tmp/cv4/bin/pip install "opencv-python-headless==4.10.0.84" numpy scipy matplotlib

## Results as of 2026-09-18

Measured on the one unit, after the firmware switched to the AS5600's RAW
ANGLE register:

| | |
|---|---|
| sensor characteristic, uncorrected | 0.225 deg p-p |
| after the order-5 Fourier fit | 3.0 mdeg rms |
| useful harmonic orders | 1..5; order 6 and beyond are 0.02-0.03 counts, i.e. noise |
| forward repeatability | 1.3 mdeg |
| direction hysteresis (motor shaft) | 27.9 mdeg |
| camera repeatability | 2.3 mdeg (1.3 mdeg over 3 frames) |
| **output error the sensor cannot see** | **~37 mdeg rms, ~150 mdeg p-p** |
| direction split on randomly ordered targets | 64 mdeg p-p, 38.7 after the firmware's one-sided approach |
| lost motion on reversal | ~89 mdeg |
| position hysteresis no approach length removes | ~25-30 mdeg |
| firmware closed loop, sensor against Alpaca target | 3.9 mdeg rms |

The last two lines matter together: the firmware already positions the motor
shaft to better than 0.01 degrees, so every remaining error is downstream of
the reduction. Approaching every target from the same side removes the part of the split
that is backlash; `refineToTarget()` and `holdTask()` now do that on their
own, and `--approach-deg` does the same from the host side. What is left is
stiction, which no approach discipline touches.

The gear error is the accuracy floor of a calibration done without a camera.
It reproduced across two runs taken at different step spacings, agreeing to
0.8 degrees of phase on its dominant term, so it is a fixed property of the
machine and correctable - but only against an external reference.

The device stores orders up to KMAX = 5 as of v0.10.0, with the NVS blob
carrying its own KMAX and migrating a smaller one. Coefficients, mechanical
zero and full-step table share a generation number so a correction replaced
without re-measuring the other two is reported rather than silently applied -
`GET /api/calibration/status`, and `upload_coefficients.py --keep-zero` for
the one case where the caller has arranged for the zero to stay valid.
