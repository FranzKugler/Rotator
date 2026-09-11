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

### Fitting method matters: least-squares vs. the correlation-sum formula

`fit_fourier_against_reference()`'s original implementation computed each
harmonic amplitude via the plain DFT-style correlation sum (`2/n * sum(
residual * cos/sin(k*theta))`) - exactly right when theta is sampled
uniformly over 0-360 deg (true for `calibration_lab.py`'s self-referential
`fit_fourier()`, whose theta is the sensor's own evenly-stepped raw
reading), but only approximately right when theta comes from the camera's
independent measurement, which has its own real nonlinearity/noise and so
is close to but not exactly evenly spaced. Live consequence
(2026-09-11, full-revolution sweep): the sum formula left 0.20-0.26 deg of
residual at orders 1-2 - the very orders it was supposed to already
remove. Switching to a proper linear least-squares solve
(`numpy.linalg.lstsq`) drove that to exactly 0 by construction and cut the
overall order<=4 residual roughly 5x (0.27 deg -> 0.055 deg motor-shaft
degrees on that sweep). Note from later in this file: this fit-quality
improvement alone did **not** reliably improve the real closed-loop
(Alpaca+camera) verification number - see "Full-step lookup table" below
for what turned out to matter more.

## Full-step accuracy (`calibration_lab.py fullstep-accuracy`)

400 points, one per full step, repeated over 2 motor revolutions:
- C0-only residual: 0.64 deg RMS, 1.41 deg peak (motor-shaft), dominated by
  the same known low orders (1,2,4). (Re-measured 2026-09-11 against the
  camera-refit model then in effect: order-6 residual RMS=1.30 deg,
  peak=3.56 deg - a different absolute number because the smooth model
  itself had changed by then, not a contradiction. The property that
  matters, rev-to-rev repeatability, was confirmed again below.)
- **Extremely repeatable** rev-to-rev: correlation +0.998, diff RMS 0.039 deg -
  a real, deterministic function of full-step position, not noise.
  (2026-09-11 re-check: correlation +1.000, diff RMS 0.049 deg - same
  conclusion.)
- One genuine, fully reproducible readout glitch found: `raw=4095.000`
  exactly, both revolutions, at the exact full step where the AS5600's raw
  register should wrap 4095->0 - looks like a torn read straddling the
  rollover, not a physical position error. `scripts/fullstep_table.py`
  despikes it before building anything from the data.
- Small (~0.02 deg motor-shaft) but real content near order ~100 - plausibly
  the motor's own rotor-tooth cogging (0.9 deg/step implies ~100 rotor teeth).

### Full-step lookup table: implemented and verified (2026-09-11)

The 400-entry lookup table sketched above is now real: `RotatorHW::
setFullStepTable()`/`getFullStepTable()`, applied in `correctSensorReading()`
as a zero-mean residual layered on top of C0/A[]/B[] (indexed by which full
step the already-smooth-corrected reading estimates it is nearest, linearly
interpolated), persisted in NVS (`"anglecal"/"fullsteptable"`), uploaded via
the expert-gated `/api/calibration/fullstep-table` endpoint. Built this time
from a **camera-referenced** capture (`scripts/camera_fullstep_sweep.py`,
2 motor revolutions, one photo per full step) instead of the self-referential
`fullstep-accuracy` data the original design sketch assumed -
`scripts/camera_fullstep_table.py` bins the smooth-corrected-vs-camera
residual per full step, despikes, mean-subtracts; `scripts/
upload_fullstep_table.py` pushes it.

Two real bugs found and fixed while building this, both worth knowing about
if this is ever redone:
- `RotatorHW::jogMicrosteps()` (the debug jog used by every full-step-scale
  capture script) did not disengage the continuous position-holding task
  (this session's `holdTask()` feature) before moving the motor. A long
  sequence of jogs (a real calibration sweep) never updates the hold
  target, so the hold task kept trying to correct back toward wherever it
  was holding *before* the sweep started, physically fighting the sweep's
  own motion the whole time it ran - corrupted an entire ~12-minute capture
  before this was caught (the Hall sensor stayed permanently "active" and
  the sensor reading got stuck for long stretches, both symptoms of the
  motor barely actually moving). Fixed by disengaging holding at the start
  of `jogMicrosteps()`, matching every other raw-motion function.
- `fit_fourier_against_reference()`'s theta must be computed from the RAW
  sensor reading, not from the camera - see the least-squares section
  below for why, and for a separate, independent bug (a single un-despiked
  ~90 deg camera tracking glitch) that made an early attempt at re-deriving
  this look far worse than it was.

**Verified end-to-end** (Alpaca `MoveAbsolute`/`Move` + independent camera
ground truth over 50 random positions, `scripts/alpaca_random_sweep.py`):
RMS improved from 0.104-0.166 deg (smooth model alone, either fitting
method) to **0.091 deg RMS / 0.156 deg peak** with the table applied - the
best closed-loop accuracy this project has measured through the firmware's
own `refineToTarget()`/Alpaca path (see "Two closed-loop paths" below for
why this isn't directly comparable to the older Python-PI-controller
figure further down).

### Does a smooth fit at a higher order replace the table? No.

Tested directly (`scripts/fullstep_order_sweep_experiment.py`): fit
`camera_angle_analyze.py`'s model at orders 1 through 199 on one measured
revolution, score the held-out residual against an *independently
measured second revolution*. Getting this measurement right took two
false starts worth recording:
- Using the camera's own phase as the fit's theta (as `fit_fourier_against_
  reference()` already does for the smooth low-order model) works for
  *that* model, but is wrong for evaluating what gets deployed: at runtime
  `correctSensorReading()` only ever has the raw sensor reading to compute
  theta from, and a fixed alignment mismatch between "camera phase" and
  "raw phase" gets amplified by the harmonic order `k`, scrambling higher
  orders long before it visibly affects order 1-4. Franz's fix: index the
  fit by the raw sensor reading itself (exactly what deployment does),
  fitting against the camera measurement only as the *target*, never as
  the phase argument - a wrong additive offset there only ever shifts C0
  (already re-derived from `gotoMechanicalZero()` regardless), never the
  harmonics.
- Once that was right, a single un-despiked ~90 deg camera corner-tracking
  glitch (the same "large per-frame step" cases `track_rotation()` already
  warns about) was still enough to wreck a least-squares fit outright - one
  bad point at ~1300+ counts off dominates a squared-error objective.
  Excluding the two flagged frames fixed it.

With both fixed, the held-out residual is essentially **flat from order 4
through order 150** (~0.755-0.855 deg motor-shaft, full-step resolution),
only degrading at order 199 (399 parameters for 400 points - classic
overfitting at the Nyquist edge). Order 7 visually tracks the measured
curve's sharp peak and small secondary bumps well (not a sine - looks like
a ramp with smaller harmonics riding on it, matching a visual Franz
independently called as "sawtooth plus sine"), but earns no measurable
held-out advantage over the already-deployed order 4. This refines (not
contradicts) the "higher orders (2-6) got worse again" finding further
above - that earlier test may have suffered a version of the same
phase-basis issue - but the practical conclusion is the same: don't chase
higher harmonic orders here, the full-step table above is what actually
works.

### Does per-segment calibration help? No.

Franz's hypothesis: if the belt pulley driving the output stage is itself
eccentric or unevenly deformed, a fixed motor-shaft angle might not behave
identically depending which of the 10 motor revolutions (segments) per
output revolution it falls in - in which case ten segment-specific AS5600
fits should out-perform one global fit. Tested on the existing
full-revolution sweep (`scripts/segment_calibration_experiment.py`): fit
each segment on half its points (even index), score on the other half (odd
index) - a held-out design specifically to guard against 10x the
parameters looking better purely by fitting noise. Result: the segmented
fit was *worse* than the global one on held-out data (RMS 0.377 deg vs
0.249 deg), and no individual segment beat the global fit convincingly.
Ruled out as an explanation for this project's remaining residual.

### Camera measurement noise floor

Never directly measured before this pass: `scripts/
camera_repeatability_test.py` photographs a completely unmoving rotator
repeatedly (30 frames) and runs the same corner-tracking analysis
everything else in this project relies on. Result: RMS=0.0022 deg,
peak-to-peak=0.009 deg - two orders of magnitude below every real-world
residual this project has measured. The camera has never been the
limiting factor; every residual documented in this file is a real
property of the rotator/sensor/calibration, not a measurement artifact.

## Closed-loop PI position control

**Two closed-loop paths now exist in this project, not directly comparable:**
this section's `scripts/pi_position_control.py` is a Python-orchestrated
prototype (drives the debug jog endpoint directly, does its own PI math on
the host) - it produced the 0.037 deg/0.076 deg figure in "Backlash and
100-point random validation" below. Since then, that design was ported into
the firmware itself (`RotatorHW::refineToTarget()`, run after every
`Move`/`MoveAbsolute`/`MoveMechanical`/`Sync`) and extended with continuous
position holding after a move settles (`RotatorHW::holdTask()`). The
full-step lookup table's 0.091 deg/0.156 deg verification above went
through *that* path, driven by genuine Alpaca calls
(`scripts/alpaca_random_sweep.py`), not the Python prototype - a real,
different measurement, not a regression from 0.037 deg.

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

- ~~The full-step lookup-table firmware change~~ - **done, 2026-09-11** -
  see "Full-step lookup table: implemented and verified" above.
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

### Added 2026-09-11 (full-step table + verification tooling)

| Script | Purpose |
|---|---|
| `scripts/camera_fullstep_sweep.py` | camera-referenced version of `fullstep-accuracy`: one photo per full step, >=2 motor revolutions |
| `scripts/camera_fullstep_table.py` | builds the full-step table from that capture, referenced against the currently-deployed model |
| `scripts/upload_fullstep_table.py` | pushes a table to `/api/calibration/fullstep-table`, confirms by read-back |
| `scripts/upload_angle_calibration.py` | pushes C0/A[]/B[] to `/api/calibration/coefficients`, confirms by read-back |
| `scripts/alpaca_client.py` | minimal stdlib ASCOM Alpaca REST client - the one thing every Alpaca-only tool below builds on |
| `scripts/alpaca_conformance.py` | 20-check Alpaca protocol conformance suite (error codes, gating, motion-limit guarantees) |
| `scripts/alpaca_random_sweep.py` | 50 random Alpaca `Move()` calls + independent camera verification - the closed-loop accuracy check this pass's numbers come from |
| `scripts/camera_repeatability_test.py` | isolates the camera+corner-detection pipeline's own noise floor (rotator held still) |
| `scripts/fullstep_order_sweep_experiment.py` | held-out (rev-vs-rev) test of whether a higher-order smooth fit can replace the full-step table |
| `scripts/segment_calibration_experiment.py` | held-out (odd/even split) test of per-motor-segment AS5600 fits vs. one global fit |

Requires `numpy`/`opencv-python-headless` in a separate venv for anything
that touches images - see `scripts/requirements-camera-analysis.txt`. Deliberately
not added to the ESP-IDF dev container's pinned Python environment.
