#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Ask an external service for an independent measurement of the output
 * shaft's angle, in degrees.
 *
 * The rotator's own AS5600 sits on the motor shaft, upstream of the 10:1
 * reduction, so it cannot see anything the gearing contributes - measured
 * live at ~36 mdeg rms of position-dependent error plus backlash. Correcting
 * that needs a reference that watches the output shaft, and the only one
 * this machine has is the camera looking at the chessboard on it.
 *
 * Deliberately just an HTTP GET returning a number: the image processing
 * belongs where the image already is, not here. A full chessboard detector
 * on this device would be the largest and least testable code in the
 * project, and it would first have to pull a 270 KB JPEG over WiFi and
 * decode it into 1.9 MB of PSRAM to get at pixels another device already
 * holds.
 *
 * Accepts either a bare number or a JSON object with an "angleDeg" member,
 * so the same setting can point at a host-side service today and at the
 * camera itself once it serves angles.
 *
 * Returns false and leaves *angleDeg untouched on any failure - not
 * configured, unreachable, non-200, unparseable. The caller must treat that
 * as "no measurement", never as zero.
 */
bool camera_angle_read(const char *url, double *angleDeg, int timeoutMs);

#ifdef __cplusplus
}
#endif
