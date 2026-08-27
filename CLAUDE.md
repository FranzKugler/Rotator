# CLAUDE.md

This file gives Claude Code project-specific guidance for work in this
repository.

## Project overview

Rotator is firmware for an astronomical instrument rotator built around a Seeed
Studio XIAO ESP32-S3 with 16 MB flash. It controls a TMC2209-driven stepper,
reads an AS5600 magnetic encoder and a mechanical-zero Hall sensor, presents an
ASCOM Alpaca Rotator API, and serves a browser UI from LittleFS.

This is an **ESP-IDF 5.4.1 project**. Arduino-ESP32 is used as an ESP-IDF
component; it is not a PlatformIO or Arduino-IDE project.

## Mandatory working environment

Run builds and ESP-IDF tools inside the repository's development container:

```sh
scripts/dev-container.sh up
scripts/dev-container.sh build
scripts/dev-container.sh web
scripts/dev-container.sh shell
scripts/dev-container.sh claude
scripts/dev-container.sh exec <command...>
```

Inside the container the repository is `/workspace`, ESP-IDF is sourced by the
image entrypoint, and the canonical build command is:

```sh
idf.py build
```

Do not install ESP-IDF, Python packages, Node packages or compiler tools on the
host merely to complete repository work. Do not modify the pinned tool versions
without explaining the compatibility reason and rebuilding the image.

## Safety and Git rules

- Inspect `git status --short --branch` before editing.
- Preserve all existing modified, untracked and ignored files. Never use
  `git reset --hard`, `git clean`, or broad deletion to make the tree clean.
- Do not commit, merge, rebase, tag or push unless the user explicitly asks.
- Keep secrets, Wi-Fi credentials, private keys and device-specific credentials
  out of Git.
- Do not add `privileged: true`, mount `/var/run/docker.sock`, mount the host home
  directory, or expose all of `/dev` in the development container.
- Hardware movement is safety-critical. Never claim a motion, calibration,
  homing or flashing change is verified by a compile alone.

## Architecture

### Startup and services

[`main/main.cpp`](main/main.cpp) owns system startup:

1. initialise Arduino and NVS;
2. initialise ESP-Netif and the default event loop;
3. install TinyUSB CDC and USB RNDIS;
4. start Wi-Fi;
5. start the HTTP server and OTA routes;
6. initialise `RotatorHW` and register Alpaca routes;
7. start browser angle-update tasks, static UI routes, Alpaca discovery and FTP;
8. home the mechanics with `gotoMechanicalZero()`.

The final point is load-bearing: the current firmware moves hardware during
boot.

### Motion and sensing

[`main/RotatorHW.cpp`](main/RotatorHW.cpp) owns the motor, encoder, Hall sensor,
mechanical homing, calibration and angle-estimation state. Its fixed assumptions
include:

- 400 motor full steps;
- 256 microsteps;
- 10:1 reduction;
- TMC2209 UART and FastAccelStepper control;
- AS5600 angle feedback;
- an active-low Hall sensor on GPIO 2.

Pin definitions and hardware constants are near the top of that file. Treat
changes there as hardware changes, not ordinary refactoring.

### APIs and browser UI

- [`main/RotatorApi.cpp`](main/RotatorApi.cpp) adapts `RotatorHW` to the ASCOM
  Alpaca Rotator contract.
- [`main/alpaca_server/`](main/alpaca_server/) implements Alpaca HTTP routing and
  discovery.
- [`main/WebServer.cpp`](main/WebServer.cpp) serves files from `/lfs`, publishes
  live angles over WebSockets, and exposes network/calibration handlers.
- [`web/`](web/) is the Svelte 5 source. `npm run build` replaces
  [`main/data/`](main/data/), which ESP-IDF packs into the `littlefs` partition.
- [`main/OTAUpdate.c`](main/OTAUpdate.c) owns OTA registration.

When changing an API consumed by the browser, update the firmware handler,
`web/src/lib/api.js`, the matching Svelte section, Vite's proxy list and tests.
Never edit generated files in `main/data/` by hand. Keep the embedded UI
self-contained; do not add runtime CDN or cloud dependencies.

### Persistence and networking

[`main/Configuration.cpp`](main/Configuration.cpp) mounts LittleFS and stores
angle-correction coefficients plus USB RNDIS network settings in
`/lfs/config.json`. Wi-Fi credentials are separate and live in NVS.

The default USB RNDIS network is `192.168.7.1/24`. Changes to stored fields must
update defaults, parsing, writing and any corresponding API/UI together.

## ESP-IDF configuration

- Target: `esp32s3`
- ESP-IDF: `5.4.1`
- Defaults: [`sdkconfig.defaults`](sdkconfig.defaults)
- Partition table: [`partitions.csv`](partitions.csv)
- Flash: 16 MB QIO
- Main component manifest: [`main/idf_component.yml`](main/idf_component.yml),
  requiring IDF `>=5.4.0` and managed TinyUSB components

The top-level `CMakeLists.txt` adds the ESP-IDF managed TinyUSB include paths
before Arduino's copies. Do not remove that ordering without proving the USB
build still uses compatible headers.

`main/CMakeLists.txt` currently uses recursive source globs and builds the
LittleFS image. New source files under `main/` are therefore picked up
automatically, but explicit source lists are preferred if this is later cleaned
up.

## Verification

For every source or build-system change:

```sh
scripts/dev-container.sh build
scripts/dev-container.sh exec idf.py size
```

For web work, use strict RED-GREEN-REFACTOR in `tests/` and run `npm test`
before `npm run build`. The full helper build performs both before ESP-IDF.

Then inspect:

```sh
git status --short --branch
git diff --check
git diff --stat
git diff
```

A successful compile verifies only integration with the pinned toolchain.
Changes in these areas require real target testing and must be reported as
unverified until that happens:

- motor direction, speed, acceleration, current or enable behaviour;
- homing and Hall-sensor handling;
- angle calibration or the EKF;
- AS5600/TMC2209 UART/I²C wiring;
- USB CDC/RNDIS descriptors and networking;
- Wi-Fi, Alpaca discovery, OTA, FTP or LittleFS persistence;
- partition layout or flashing.

There is currently no automated test suite. Add host-side tests for pure maths,
parsing and protocol logic when extracting such logic from hardware-bound code.

## Code conventions

- Keep the existing C/C++ split and ESP-IDF error-reporting style.
- Use ESP-IDF logging (`ESP_LOG*`) rather than ad-hoc output.
- Prefer typed constants (`constexpr`) over new macros where practical.
- Check every external input before dereferencing JSON members or copying into
  fixed-size buffers.
- Avoid blocking the HTTP server task with long motor/calibration operations;
  use tasks or an asynchronous status path where appropriate.
- Do not silently change persistent formats, GPIO assignments, Alpaca semantics
  or mechanical constants during cleanup.
- Comments may be German or English, but new public documentation and API names
  should be clear, consistent English.
