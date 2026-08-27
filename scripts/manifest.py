"""Write the small release manifest for Rotator firmware and filesystem images."""

import argparse
import datetime
import hashlib
import json
import os
import sys


def describe(path, base_url):
    digest = hashlib.sha256()
    with open(path, "rb") as image:
        for block in iter(lambda: image.read(65536), b""):
            digest.update(block)
    return {
        "url": f"{base_url.rstrip('/')}/{os.path.basename(path)}",
        "size": os.path.getsize(path),
        "sha256": digest.hexdigest(),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--channel", required=True, choices=["stable", "edge"])
    parser.add_argument("--version", required=True)
    parser.add_argument("--notes", default="")
    parser.add_argument("--base-url", required=True)
    parser.add_argument("--out", default="manifest.json")
    parser.add_argument("firmware")
    parser.add_argument("filesystem")
    args = parser.parse_args()

    for path in (args.firmware, args.filesystem):
        if not os.path.isfile(path):
            sys.exit(f"no such image: {path}")

    manifest = {
        "channel": args.channel,
        "version": args.version,
        "notes": args.notes,
        "built": datetime.datetime.now(datetime.timezone.utc)
        .replace(microsecond=0)
        .isoformat()
        .replace("+00:00", "Z"),
        "firmware": describe(args.firmware, args.base_url),
        "filesystem": describe(args.filesystem, args.base_url),
    }
    with open(args.out, "w", encoding="utf-8") as out:
        json.dump(manifest, out, indent=2)
        out.write("\n")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
