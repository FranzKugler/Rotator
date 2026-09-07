"""Write the manifest ESP Web Tools reads to flash a blank chip from a browser.

See docs/index.html, the "first flashing" page this feeds - read once by a
browser that has never met the chip, so unlike manifest.py's OTA manifest (only
ever read by a rotator that already runs this firmware and only ever touches
the two OTA app partitions) it has to name every part a factory-fresh XIAO
ESP32-S3 needs, the bootloader and partition table themselves included.

Every offset comes from files ESP-IDF's own build already writes -
build/flasher_args.json (bootloader, factory app, partition table, otadata)
and build/littlefs-flash_args (the filesystem image) - rather than being
repeated here, so a changed partitions.csv or sdkconfig cannot silently drift
out of step with what this writes.

Usage:
    python3 scripts/web_tools_manifest.py --version 1.2.3 \\
        --base-url . --out esp-web-tools-manifest.json build
"""

import argparse
import json
import os
import re
import sys


def flasher_args_parts(build_dir):
    """[(offset, path), ...] for the bootloader, partition table, otadata
    stub and factory app, straight out of ESP-IDF's own flasher_args.json -
    the same file `idf.py flash` itself reads."""
    with open(os.path.join(build_dir, "flasher_args.json"), encoding="utf-8") as f:
        flasher_args = json.load(f)
    return [
        (int(offset, 16), os.path.join(build_dir, rel_path))
        for offset, rel_path in flasher_args["flash_files"].items()
    ]


def littlefs_part(build_dir):
    """(offset, path) for the filesystem image, out of the flash_args file
    `idf.py littlefs-flash` reads - never part of flasher_args.json, since a
    running rotator's own OTA update writes it separately from the app."""
    path = os.path.join(build_dir, "littlefs-flash_args")
    with open(path, encoding="utf-8") as f:
        match = re.search(r"(0x[0-9a-fA-F]+)\s+(\S+)", f.read())
    if not match:
        sys.exit(f"could not find an offset/filename pair in {path}")
    return int(match.group(1), 16), os.path.join(build_dir, match.group(2))


def main():
    parser = argparse.ArgumentParser(
        description="Write the ESP Web Tools manifest for flashing a blank chip."
    )
    parser.add_argument("--version", required=True)
    parser.add_argument("--base-url", required=True, help="where the images will be served from")
    parser.add_argument("--out", default="esp-web-tools-manifest.json")
    parser.add_argument("build_dir", help="e.g. build")
    args = parser.parse_args()

    parts = sorted(flasher_args_parts(args.build_dir) + [littlefs_part(args.build_dir)])
    for _, path in parts:
        if not os.path.isfile(path):
            sys.exit(f"no such image: {path}")

    base = args.base_url.rstrip("/")
    manifest = {
        "name": "AG2998 Rotator",
        "version": args.version,
        # A browser here has never met this chip - offering the erase
        # checkbox is right every time, not just the first.
        "new_install_prompt_erase": True,
        "builds": [
            {
                "chipFamily": "ESP32-S3",
                "parts": [
                    {"path": f"{base}/{os.path.basename(path)}", "offset": offset}
                    for offset, path in parts
                ],
            }
        ],
    }

    with open(args.out, "w", encoding="utf-8") as out:
        json.dump(manifest, out, indent=2)
        out.write("\n")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
