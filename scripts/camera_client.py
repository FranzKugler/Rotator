"""Client for the RotatorCam side project (../RotatorCam): fetches single
still JPEGs over WiFi from a XIAO ESP32S3 Sense, for the independent,
output-shaft-referenced angle measurement the AS5600 cannot provide (it sits
on the motor shaft, upstream of the 10:1 reduction). Standard library only.

Usage:
    python3 scripts/camera_client.py --host 172.22.102.31 --out shot.jpg
    python3 scripts/camera_client.py --host 172.22.102.31 status
"""

import argparse
import json
import sys
from urllib.error import HTTPError, URLError
from urllib.request import urlopen


def capture(host, timeout=15):
    """GET /capture - one fresh JPEG. Returns the raw image bytes."""
    with urlopen(f"http://{host}/capture", timeout=timeout) as resp:
        return resp.read()


def status(host, timeout=5):
    """GET /status - camera/board health as a dict."""
    with urlopen(f"http://{host}/status", timeout=timeout) as resp:
        return json.loads(resp.read())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", required=True, help="RotatorCam board IP or hostname")
    ap.add_argument("command", nargs="?", default="capture", choices=["capture", "status"])
    ap.add_argument("--out", default="capture.jpg", help="capture: where to save the JPEG")
    args = ap.parse_args()

    try:
        if args.command == "status":
            print(json.dumps(status(args.host), indent=2))
        else:
            data = capture(args.host)
            with open(args.out, "wb") as f:
                f.write(data)
            print(f"wrote {args.out} ({len(data)} bytes)")
    except HTTPError as e:
        sys.exit(f"camera refused the request ({e.code})")
    except URLError as e:
        sys.exit(f"could not reach {args.host}: {e.reason}")


if __name__ == "__main__":
    main()
