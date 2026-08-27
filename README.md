# Rotator

ESP32-S3 firmware for an astronomical instrument rotator. The device drives a
stepper motor, measures the shaft angle, exposes an **ASCOM Alpaca Rotator** over
the network, and serves a local browser interface for movement, calibration and
network setup.

The firmware is an ESP-IDF project with Arduino-ESP32 included as a component.
It targets a **Seeed Studio XIAO ESP32-S3 with 16 MB flash**.

## Features

- ASCOM Alpaca Rotator API and Alpaca discovery
- Absolute, relative and mechanical-angle movement
- TMC2209 stepper driver and FastAccelStepper motion control
- AS5600 magnetic angle feedback
- Hall-sensor mechanical-zero search
- Fourier-based angle-sensor calibration and an extended Kalman filter
- Browser UI from a LittleFS partition, including live angle updates
- Wi-Fi station support plus USB RNDIS networking
- USB CDC logging, FTP access and OTA firmware updates
- QlockThreeW32-style GitHub update channels (stable/edge), version checks,
  verified one-click installation, scheduled automatic updates and local image upload

## Hardware

| Function | Device / GPIO |
|---|---|
| Controller | Seeed Studio XIAO ESP32-S3, 16 MB flash |
| Stepper driver UART RX / TX | GPIO 7 / GPIO 8 |
| Stepper enable | GPIO 1 |
| Step / direction | GPIO 3 / GPIO 4 |
| AS5600 I²C SDA / SCL | GPIO 5 / GPIO 6 |
| AS5600 direction | GPIO 43 |
| Mechanical-zero Hall sensor | GPIO 2, active low with pull-up |

The current motion model assumes a 400-full-step motor, 256 microsteps and a
10:1 reduction. Hardware constants and the pin assignment live in
[`main/RotatorHW.cpp`](main/RotatorHW.cpp).

> **Warning:** the firmware moves to mechanical zero during startup. Do not run
> newly built firmware on connected mechanics until the pinout, travel and motor
> current have been checked.

## Development environment

The supported build environment is the repository's Docker/Dev Container. It
pins ESP-IDF 5.4.1 and Claude Code and keeps the ESP-IDF, npm and Claude state in
named volumes. Only this repository is bind-mounted into the container; no
Docker socket or host home directory is exposed.

Prerequisites on the host:

- Docker Engine with Docker Compose
- Git

```sh
# Build and start the development container
scripts/dev-container.sh up

# Test/build Svelte, then build firmware, bootloader, partitions and LittleFS
scripts/dev-container.sh build

# Only test and build the Svelte interface into main/data
scripts/dev-container.sh web

# Open an ESP-IDF shell
scripts/dev-container.sh shell

# Start Claude Code in the project
scripts/dev-container.sh claude
```

The first image build downloads ESP-IDF and installs the pinned Claude Code
release. The first project build resolves npm and ESP-IDF managed components.
The browser UI is a Svelte 5/Vite application under `web/`; its production
output replaces `main/data/` before ESP-IDF creates `littlefs.bin`.

### Releases and versions

The fallback project version starts at **0.8.0**. A release tag such as
`v0.8.0` produces the bare version `0.8.0`; later commits become strings such as
`0.8.0-4-gabc123`, matching QlockThreeW32. Pushes to `main` replace the rolling
`edge` pre-release, while `v*` tags create stable releases. Each release contains
`Rotator.bin`, `littlefs.bin` and a checksummed `manifest.json`.

### Direct ESP-IDF commands

Inside `scripts/dev-container.sh shell`:

```sh
idf.py set-target esp32s3
idf.py build
idf.py size
idf.py menuconfig
idf.py fullclean
```

The project defaults are in [`sdkconfig.defaults`](sdkconfig.defaults), and the
16 MB flash layout is in [`partitions.csv`](partitions.csv). Build artifacts are
written to `build/`; the generated LittleFS image is part of the normal
`idf.py build` because `main/CMakeLists.txt` calls
`littlefs_create_partition_image()`.

## Flashing and monitoring

USB devices are deliberately **not** passed into the normal development
container. This keeps ordinary builds and Claude Code isolated from host
hardware. Once a device is attached, identify its stable
`/dev/serial/by-id/...` path and add only that device in a local Compose
override before using:

```sh
idf.py -p /dev/serial/by-id/<device> flash monitor
```

Do not solve serial access with `privileged: true` or a blanket `/dev` mount.

## Project layout

```text
main/                       application firmware and generated browser UI
  alpaca_server/            ASCOM Alpaca API and discovery implementation
  data/                     files packed into the LittleFS image
components/                 vendored Arduino and hardware-related components
web/                        Svelte 5 source for the embedded interface
tests/                      web API unit tests
partitions.csv              16 MB flash layout
sdkconfig.defaults          ESP32-S3 project defaults
.devcontainer/              pinned Docker/Dev Container definition
scripts/dev-container.sh    host-side environment helper
```

Configuration is stored as `/lfs/config.json`. Wi-Fi credentials are stored in
NVS. The default USB RNDIS address is `192.168.7.1/24`; its locally administered
MAC address and the angle-correction coefficients are defined in
[`main/Configuration.cpp`](main/Configuration.cpp) and can be changed through
the web API.

## Current project status

This repository currently has no automated test suite. The minimum acceptance
check for every change is therefore a clean `idf.py build` in the pinned
container, followed by targeted hardware testing when motion, calibration,
networking, USB or flash behaviour is affected.

## License

No project-wide license file is currently present. Vendored components retain
their respective upstream licenses.
