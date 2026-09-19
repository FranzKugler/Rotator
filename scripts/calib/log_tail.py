"""Continuously drain the rotator's /log ring buffer into a file.

The buffer holds about a hundred lines and angle_producer_task writes two of
them every 700 ms, so it wraps in roughly half a minute - far too fast to
still hold the interesting part by the time a fault is noticed. Polling it
faster than it wraps and keeping everything is the only way to have a record
of what the firmware was doing when something went wrong.

Sequence numbers are used to stitch polls together, so a gap is reported
rather than silently swallowed.
"""

import argparse
import json
import sys
import time
from urllib.request import urlopen


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="172.22.102.30")
    ap.add_argument("--out", required=True)
    ap.add_argument("--interval", type=float, default=5.0)
    ap.add_argument("--drop-noise", action="store_true",
                    help="skip angle_producer_task's tick/tock, which is 90 percent "
                         "of the volume and never the thing being looked for")
    args = ap.parse_args()

    seen = -1
    dropped = 0
    with open(args.out, "a", buffering=1) as out:
        out.write(f"# log_tail started {time.strftime('%Y-%m-%d %H:%M:%S')} "
                  f"host={args.host}\n")
        while True:
            try:
                with urlopen(f"http://{args.host}/log", timeout=6) as response:
                    data = json.loads(response.read())
            except Exception as error:  # noqa: BLE001 - a dead device is the event
                out.write(f"# {time.strftime('%H:%M:%S')} UNREACHABLE {type(error).__name__}: {error}\n")
                time.sleep(args.interval)
                continue

            if data["oldest"] > seen + 1 and seen >= 0:
                out.write(f"# GAP: {data['oldest'] - seen - 1} lines lost "
                          f"(polling too slowly)\n")
            for line in data["lines"]:
                if line["s"] <= seen:
                    continue
                seen = line["s"]
                message = line["m"]
                if args.drop_noise and "angProd:" in message:
                    dropped += 1
                    continue
                out.write(f"{line['s']:7d} {line['t']:9d} {message}\n")
            out.write(f"# uptime={data['uptime']} heap={data['heap']} "
                      f"heapMin={data['heapMin']} reset={data['reset']} "
                      f"seq={data['seq']} droppedNoise={dropped}\n")
            time.sleep(args.interval)


if __name__ == "__main__":
    main()
