# Angle calibration findings

Working notes from a deep pass on the rotator's angle accuracy: the TMC2209
current bug, why the AS5600's own harmonic self-calibration is structurally
biased, the RotatorCam independent camera reference, and the closed-loop PI
controller that ended up mattering more than any of the sensor modeling.

All angles are **output-shaft degrees** unless stated otherwise (AS5600
raw/corrected values live in "counts", 4096/rev; motor-shaft degrees are
counts × 360/4096; output-shaft = motor-shaft / 10, the fixed gear
reduction).

## TMC2209 current fix

`RotatorHW::begin()` was overriding `setRMSCurrent()`'s computed run current
with the driver's absolute maximum (~980 mA) and leaving the hold current at
the small `setRMSCurrent()`-derived value - a ~16:1 run:hold mismatch, and
far beyond what the 17 ohm/phase coil can physically draw at 5V (~290 mA
ceiling), so StealthChop's current regulator saturated for most of each
microstep waveform. Fixed by removing the override and running at 280 mA
(safely under the ~290 mA ceiling). Live-verified via motor-sweep: not a
clean unambiguous win at every measured position, but net positive and the
only physically correct setting.

## AS5600 self-referential correction bias

`calibrateAngleSensorStep()`/`fit_fourier()` fit a harmonic error model with
phase `theta` taken from the sensor's own raw reading, because
`correctSensorReading()` never has anything else available at runtime. This
is a real, previously-documented, deliberately-accepted tradeoff (an
ideal-phase-basis version was tried once and made things worse - see
`RotatorHW.cpp`'s history) - but it comes at a cost that got a lot clearer
this pass:

- On a full-revolution sweep, a 4th-order self-referential fit was
  **reproducibly worse** than simply subtracting the mean (C0-only): 0.107 deg
  vs 0.065 deg RMS (output-shaft), confirmed at two averaging depths.
- At full-step resolution (400 points/revolution), the effect is dramatic:
  a 6th-order self-referential fit turned a 0.64 deg RMS C0-only residual
  into 2.3-6.3 deg (`calibration_lab.py fullstep-accuracy`). Not usable.
- The bias only goes away with an **independent** reference - i.e. not
  derived from the same raw reading being corrected. That's what the camera
  rig is for.

## RotatorCam: an independent, output-shaft-referenced reference

New side project (`../RotatorCam`, XIAO ESP32S3 Sense + OV2640): photographs
a printed checkerboard mounted on the rotation axis. Unlike the AS5600 (motor
shaft, upstream of the 10:1 reduction), the camera watches the true output
shaft directly. `scripts/camera_angle_sweep.py` drives a sweep and
photographs each point; `scripts/camera_angle_analyze.py` tracks rotation
frame-to-frame via sub-pixel checkerboard corner detection + a Procrustes
(rotation-only) fit, resolving the classic "which end is which" checkerboard
ambiguity by picking whichever candidate gives the smaller frame-to-frame
rotation (valid as long as steps stay well under ~90 deg).

**Camera repositioning mattered more than any correction did:** raising the
camera for better focus dropped the raw residual from 0.198 deg to 0.058 deg
RMS before any calibration. A "closure check" (comparing the same physical
angle visited a full revolution apart) went from 0.524 deg to 0.071 deg,
confirming most of the original error was focus/tracking noise, not real
mechanical slip.

### Eccentricity calibration (target mounting error)

The target can't be glued perfectly centered on the axis - Franz's proposal:
probe 10 points exactly 400 full steps apart (= exactly 1/10 output
revolution, same motor phase every time, verified via `align_fullstep()`),
fit a low-order (1st+2nd harmonic) model to those 10 camera residuals, and
subtract it from the whole sweep. Two independent runs (an 11-frame probe
and the full 361-frame sweep) agreed almost exactly on amplitude and phase -
strong validation. Result: camera RMS 0.058 -> 0.019 deg, finally below the
AS5600's own 0.064 deg.

### Refitting the AS5600 against the camera

With eccentricity removed, refitting the AS5600's harmonic model using the
camera as phase reference (instead of the sensor's own raw reading) at
**order 1 only** gave 0.047 deg RMS - beating the existing 0.064 deg bias-only
correction for the first time in this project, self- or externally-referenced.
Higher orders (2-6) got worse again - camera measurement noise dominates the
extra coefficients past order 1.

## Full-step accuracy (`calibration_lab.py fullstep-accuracy`)

400 points, one per full step, repeated over 2 motor revolutions:
- C0-only residual: 0.64 deg RMS, 1.41 deg peak (motor-shaft), dominated by
  the same known low orders (1,2,4).
- **Extremely repeatable** rev-to-rev: correlation +0.998, diff RMS 0.039 deg -
  a real, deterministic function of full-step position, not noise.
- One genuine, fully reproducible readout glitch found: `raw=4095.000`
  exactly, both revolutions, at the exact full step where the AS5600's raw
  register should wrap 4095->0 - looks like a torn read straddling the
  rollover, not a physical position error. `scripts/fullstep_table.py`
  despikes it before building anything from the data.
- Small (~0.02 deg motor-shaft) but real content near order ~100 - plausibly
  the motor's own rotor-tooth cogging (0.9 deg/step implies ~100 rotor teeth).

A 400-entry lookup-table replacement for `correctSensorReading()`'s
KMAX=4 self-referential Fourier model was designed (code sketch,
`config.json` format, regeneration workflow) but **deliberately not
implemented** - see "What's deferred" below.

## Closed-loop PI position control

The insight that mattered most: instead of chasing an increasingly complex
motor/gearbox model (cogging, position-dependent full-step gain that varies
+/-25% around the revolution, weak inter-step wobble correlation), close the
loop. `scripts/pi_position_control.py` moves toward a target, re-measures
(Kalman-filtered AS5600 reading, `scripts/angle_filter_lab.py`'s
`AngleKalman1D`), and corrects - absorbing whatever the true open-loop gain
turns out to be at that spot, without needing to predict it.

First live test overshot badly (16 iterations, visible ringing) from
classic **integral windup**: the large initial error entered the integral
term and dominated for many iterations after the proportional term was
already small. Fixed with conditional integration (only integrate while
`|error| < 0.05 deg`) - a standard anti-windup technique, not a hardware
issue. After the fix: 1-3 iterations, confirmed final error 0.004-0.007 deg.

## Backlash and 100-point random validation

`scripts/backlash_and_random_validation.py` + `scripts/backlash_camera_analyze.py`:
approach a target from a fast open-loop overshoot then a precise PI
convergence, from a chosen direction (cw/ccw), photograph, repeat.

- **Dedicated backlash test** (8 positions, cw+ccw each): mean 0.015 deg,
  spread (RMS) 0.019 deg, peak 0.052 deg - at or below the camera's own noise
  floor, so best read as "backlash, if present at all, is under ~0.02-0.05 deg,"
  not a precisely resolved value.
- **100 random targets, random cw/ccw direction each, independent camera
  verification**: overall RMS 0.037 deg, peak 0.076 deg against the pure
  open-loop commanded odometer - the best end-to-end accuracy number this
  project has produced. A directional bias (cw mean +0.010 deg, ccw mean
  +0.045 deg, difference -0.034 deg) is statistically solid at this sample
  size (~8 standard errors from zero) and refines the smaller-sample
  backlash test's estimate.

## What's deferred

- The full-step lookup-table firmware change (`correctSensorReading()`
  replacement) - designed, not implemented. Revisit if the PI-controller
  approach ever turns out to need it as a feedforward starting point (it
  hasn't so far).
- Verifying camera FOV/focus distance formally (mount is still adjustable).
- SpreadCycle re-tuning (HSTRT/HEND/TBL) now that current is fixed - not
  revisited this pass, since closed-loop control looks like it matters more
  than the open-loop waveform shape.

## Tools produced this pass

| Script | Purpose |
|---|---|
| `scripts/camera_angle_sweep.py` | drives a full output revolution, photographs each point |
| `scripts/camera_angle_analyze.py` | corner tracking, camera-vs-AS5600 comparison, camera-referenced refit |
| `scripts/camera_eccentricity_calibrate.py` | fits/removes target-mounting eccentricity from 10 full-step-aligned probe points |
| `scripts/fullstep_table.py` | builds (and despikes) a 400-entry per-full-step correction table - data only, not wired into firmware |
| `scripts/pi_position_control.py` | closed-loop PI position control with Kalman-filtered feedback |
| `scripts/backlash_and_random_validation.py` | directional (cw/ccw) approach + camera capture, for backlash and broad accuracy checks |
| `scripts/backlash_camera_analyze.py` | analyzes the above: per-position backlash, cw/ccw statistics |
| `../RotatorCam/generate_pattern.py` | printable checkerboard target (separate repo) |

Requires `numpy`/`opencv-python-headless` in a separate venv for anything
that touches images - see `scripts/requirements-camera-analysis.txt`. Deliberately
not added to the ESP-IDF dev container's pinned Python environment.
