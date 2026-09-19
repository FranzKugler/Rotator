"""Upload a fitted sensor correction to the device, without moving its zero.

The device stores the mechanical-zero Hall target as a *corrected* sensor
value (RotatorHW::setZeroPosSensorValue), so replacing the correction also
moves what the machine calls zero - here by 0.22 output degrees, which would
silently shift every Alpaca position it reports.

The constant term is exactly the freedom needed to avoid that. Choosing

    C0' = C0_old + harmonics_old(raw0) - harmonics_new(raw0)

makes the new correction agree with the old one at raw0, the reading at
mechanical zero, so the stored Hall target stays valid and the absolute
scale is untouched. It costs nothing: a constant offset is the one part of
the correction that carries no shape, and every accuracy figure in this
campaign is computed with the mean removed anyway.

The device's stored format is KMAX = 4 (main/RotatorHW.h), so only orders up
to 4 fit. Order 5 measured 3.04 mdeg residual against order 4's 4.80; going
further would be a persistent-format change.
"""

import argparse
import json
import math
import sys
import urllib.request

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from rotator_io import Rotator, OUTPUT_DEG_PER_COUNT


def harmonics(raw, a, b):
    theta = 2.0 * math.pi * raw / 4096.0
    return sum(a[k] * math.cos(k * theta) + b[k] * math.sin(k * theta)
               for k in range(1, len(a)))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="172.22.102.30")
    ap.add_argument("--fit", required=True, help="fit_sensor.py --export JSON")
    ap.add_argument("--raw-at-zero", type=float, required=True,
                    help="raw sensor reading where the machine last homed")
    ap.add_argument("--zero-fullstep-table", action="store_true",
                    help="also clear the 400-entry full-step residual table, which "
                         "was fitted against the previous correction and is not "
                         "valid against this one")
    ap.add_argument("--apply", action="store_true", help="actually write to the device")
    args = ap.parse_args()

    rotator = Rotator(args.host)
    old = rotator.get_coefficients()
    with open(args.fit) as handle:
        new = json.load(handle)

    if len(new["A"]) > len(old["A"]):
        sys.exit(f"fit has order {len(new['A'])-1}, device stores up to "
                 f"{len(old['A'])-1} - refit with --order {len(old['A'])-1}")

    a = list(new["A"]) + [0.0] * (len(old["A"]) - len(new["A"]))
    b = list(new["B"]) + [0.0] * (len(old["B"]) - len(new["B"]))

    raw0 = args.raw_at_zero
    shifted = (old["C0"] + harmonics(raw0, old["A"], old["B"])
               - harmonics(raw0, a, b))

    def corrected(raw, c0, aa, bb):
        return raw - c0 - harmonics(raw, aa, bb)

    print(f"raw at mechanical zero: {raw0:.3f}")
    print(f"  old correction there: {corrected(raw0, old['C0'], old['A'], old['B']):.4f}")
    print(f"  new C0 as fitted:     {new['C0']:.4f}  -> would move zero by "
          f"{(corrected(raw0, new['C0'], a, b) - corrected(raw0, old['C0'], old['A'], old['B'])) * OUTPUT_DEG_PER_COUNT:.4f} deg")
    print(f"  C0 shifted to:        {shifted:.4f}  -> moves zero by "
          f"{(corrected(raw0, shifted, a, b) - corrected(raw0, old['C0'], old['A'], old['B'])) * OUTPUT_DEG_PER_COUNT:.6f} deg")
    print(f"\n  old A: {[round(v,4) for v in old['A']]}")
    print(f"  new A: {[round(v,4) for v in a]}")
    print(f"  old B: {[round(v,4) for v in old['B']]}")
    print(f"  new B: {[round(v,4) for v in b]}")

    if not args.apply:
        print("\n(dry run - pass --apply to write)")
        return

    rotator.set_coefficients(shifted, a, b)
    back = rotator.get_coefficients()
    print(f"\nwrote coefficients; device now reports C0={back['C0']:.4f}")
    if args.zero_fullstep_table:
        rotator.set_fullstep_table([0.0] * 400)
        print("cleared the full-step residual table")


if __name__ == "__main__":
    main()
