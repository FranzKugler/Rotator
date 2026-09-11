"""Pushes a full-step correction table built by
scripts/camera_fullstep_table.py (or scripts/fullstep_table.py, the
self-referential equivalent) onto the rotator, via the expert-gated
main/WebServer.cpp:/api/calibration/fullstep-table endpoint
(main/RotatorHW.cpp's setFullStepTable()).

Usage:
    /tmp/camvenv/bin/python scripts/camera_fullstep_table.py \\
        --sweep-dir /tmp/fullstep_sweep1 --host 172.22.102.30 --out /tmp/fullstep_sweep1/table.json
    python3 scripts/upload_fullstep_table.py --host 172.22.102.30 --table /tmp/fullstep_sweep1/table.json
"""

import argparse
import json
import sys
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

EXPECTED_N_STEPS = 400  # main/RotatorHW.h's N_STEPS


def fetch_table(host, timeout=10):
    with urlopen(f"http://{host}/api/calibration/fullstep-table", timeout=timeout) as resp:
        return json.loads(resp.read())["table"]


def push_table(host, table, timeout=10):
    body = json.dumps({"table": table}).encode()
    req = Request(f"http://{host}/api/calibration/fullstep-table", data=body, method="POST",
                  headers={"Content-Type": "application/json"})
    with urlopen(req, timeout=timeout) as resp:
        return resp.read()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", required=True)
    ap.add_argument("--table", required=True, help="the --out JSON from *fullstep_table.py")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    with open(args.table) as f:
        data = json.load(f)
    table = data["table"]
    if len(table) != EXPECTED_N_STEPS:
        sys.exit(f"table has {len(table)} entries, firmware N_STEPS is {EXPECTED_N_STEPS}")

    if "rmsDeg" in data:
        print(f"Table quality: RMS={data['rmsDeg']:.4f} deg  peak={data['peakDeg']:.4f} deg (motor-shaft)")

    try:
        before = fetch_table(args.host)
    except (HTTPError, URLError) as e:
        sys.exit(f"could not read the current table from {args.host}: {e}")
    before_nonzero = sum(1 for v in before if v != 0)
    print(f"Currently on the device: {before_nonzero}/{len(before)} nonzero entries")

    if args.dry_run:
        print("--dry-run: not uploaded")
        return

    try:
        push_table(args.host, table)
    except (HTTPError, URLError) as e:
        sys.exit(f"upload failed: {e}")

    after = fetch_table(args.host)

    def close(x, y, tol=1e-4):
        return abs(x - y) < tol

    mismatched = any(not close(x, y) for x, y in zip(after, table))
    if mismatched:
        sys.exit("uploaded, but read-back does not match")

    after_nonzero = sum(1 for v in after if v != 0)
    print(f"Uploaded and confirmed by read-back: {after_nonzero}/{len(after)} nonzero entries")


if __name__ == "__main__":
    main()
