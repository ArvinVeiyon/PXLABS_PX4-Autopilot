# PXLABS PX4-Autopilot

## Overview

This repository is a customized fork of [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) maintained by **PXLABS**. It targets the **NXP FMU-V6XRT** flight controller with enhanced rover support for Ackermann, Differential, and Mecanum platforms.

## Repository Information

| Property | Value |
|----------|-------|
| Latest Stable | `pxlabs-v1.17.0-r1` |
| Latest Beta | - |
| Development | `pxlabs-v1.17.0-dev` |
| Upstream Base | PX4 v1.17.0 |
| Target Board | NXP FMU-V6XRT |
| Upstream Repo | https://github.com/PX4/PX4-Autopilot |
| Maintainer | PXLABS |

## Branch Structure

| Branch | Description |
|--------|-------------|
| `px4-v1.17.0` | Clean upstream PX4 v1.17.0 — no PXLABS changes (reference base) |
| `pxlabs-v1.17.0-r1` | **Current stable** — PXLABS modifications on PX4 v1.17.0 |
| `pxlabs-v1.17.0-dev` | Active development for next release |
| `pxlabs-v1.16.1-r1` | Previous stable (PX4 v1.16.1 based) |
| `pxlabs-v1.16.1-r2-Beta` | Previous beta — collision prevention (untested) |

## PXLABS Modifications (v1.17.0)

### Sensor Configuration (NXP FMU-V6XRT)

Custom IMU SPI bus and rotation corrections across all 3 hardware variants:

| Variant | LPSPI1 | LPSPI2 | LPSPI3 |
|---------|--------|--------|--------|
| V6XRT000 / V6XRT001 | ICM42688P `-R 12` | ICM45686 `-R 10` | BMI088 `-R 12` |
| V6XRT002 | ICM42688P `-R 12` | ICM45686 `-R 10` | BMI088 `-R 12` |

Additional changes:
- BMM350 magnetometer enabled on V6XRT000
- ist8310 external compass disabled (not used on custom build)

### Board Configuration

- `CONFIG_COMMON_OPTICAL_FLOW=y` — optical flow sensor support
- `CONFIG_MODULES_ROVER_ACKERMANN=y` — Ackermann steering controller
- `CONFIG_MODULES_ROVER_DIFFERENTIAL=y` — differential drive controller
- `CONFIG_MODULES_ROVER_MECANUM=y` — Mecanum wheel controller (via rover.px4board)

### DDS Topics (uXRCE-DDS)

**Publications added (FMU → ROS2):**
- `/fmu/out/input_rc` — RC input telemetry
- `/fmu/out/rover_throttle_setpoint` — rover throttle feedback
- `/fmu/out/rover_steering_setpoint` — rover steering feedback

**Subscriptions (already in upstream v1.17.0):**
- `/fmu/in/rover_position_setpoint`
- `/fmu/in/rover_attitude_setpoint`
- `/fmu/in/rover_rate_setpoint`
- `/fmu/in/rover_throttle_setpoint`
- `/fmu/in/rover_steering_setpoint`

---

## Toolchain Requirements

> **CRITICAL: Use GCC arm-none-eabi 9.3.1 — newer versions cause boot failures on FMU-V6XRT**

```bash
# Remove existing toolchain
sudo apt remove gcc-arm-none-eabi

# Download GCC 9.3.1
wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2020q2/gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2

# Extract
sudo tar -xjf gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2 -C /opt

# Add to PATH (add to ~/.bashrc for persistence)
export PATH="/opt/gcc-arm-none-eabi-9-2020-q2-update/bin:$PATH"

# Verify — must show 9.3.1
arm-none-eabi-gcc --version
```

---

## Getting Started

### Clone Repository

```bash
git clone https://github.com/ArvinVeiyon/PXLABS_PX4-Autopilot.git
cd PXLABS_PX4-Autopilot

# Stable release
git checkout pxlabs-v1.17.0-r1

# OR development
git checkout pxlabs-v1.17.0-dev

git submodule update --init --recursive
```

### Setup Development Environment

```bash
bash ./Tools/setup/ubuntu.sh
```

### Build Commands

```bash
# Firmware for NXP FMU-V6XRT
make px4_fmu-v6xrt_default

# Rover-only firmware (Ackermann + Differential + Mecanum)
make px4_fmu-v6xrt_rover

# Bootloader
make px4_fmu-v6xrt_bootloader

# Upload via USB
make px4_fmu-v6xrt_default upload

# SITL simulation
make px4_sitl gazebo
make px4_sitl gazebo_rover
make px4_sitl gazebo_rover_ackermann

# List all targets
make list_config_targets
```

---

## PXLABS Resources (`pxlabs/` folder)

```
pxlabs/
├── Parameters/
│   └── PXlabs_Differential_Rover_NXP_tested_2026-05-24.params
├── NXP_Bootloader/              (to be added after build)
│   └── px4_fmu-v6xrt_bootloader.bin
├── PXLabs_Bootloader/           (to be added after build)
│   ├── px4_fmu-v6xrt_bootloader.bin
│   ├── px4_fmu-v6xrt_bootloader.elf
│   └── px4_fmu-v6xrt_bootloader.map
├── PXLabs_Firmware/             (to be added after build)
│   ├── px4_fmu-v6xrt_default.bin
│   ├── px4_fmu-v6xrt_default.px4
│   ├── px4_fmu-v6xrt_default.elf
│   └── px4_fmu-v6xrt_default.map
└── PXLABS_V6XRT_CUSTOM.patch
```

### Parameters

| File | Vehicle | Status | Date |
|------|---------|--------|------|
| [`PXlabs_Differential_Rover_NXP_tested_2026-05-24.params`](pxlabs/Parameters/PXlabs_Differential_Rover_NXP_tested_2026-05-24.params) | Differential Rover — NXP FMU-V6XRT | Tested ✓ | 2026-05-24 |

To load parameters: open QGroundControl → Vehicle Setup → Parameters → Tools → Load from file.

### Pre-compiled Bootloaders

> **Note:** Bootloader binaries to be added after v1.17.0 build is complete.

| Bootloader | Path | Notes |
|------------|------|-------|
| NXP Bootloader | `pxlabs/NXP_Bootloader/px4_fmu-v6xrt_bootloader.bin` | Provided by NXP, compiled with GCC 9.3.1 — **use this if experiencing boot issues** |
| PXLabs Bootloader | `pxlabs/PXLabs_Bootloader/px4_fmu-v6xrt_bootloader.bin` | Built by PXLABS with GCC 9.3.1 — includes .elf/.map for debugging |

### Pre-compiled Firmware

> **Note:** Firmware binaries to be added after v1.17.0 build is complete.

| File | Path | Description |
|------|------|-------------|
| Firmware binary | `pxlabs/PXLabs_Firmware/px4_fmu-v6xrt_default.bin` | Flash via QGC |
| PX4 package | `pxlabs/PXLabs_Firmware/px4_fmu-v6xrt_default.px4` | Flash via QGC |
| ELF (debug) | `pxlabs/PXLabs_Firmware/px4_fmu-v6xrt_default.elf` | For GDB debugging |
| Memory map | `pxlabs/PXLabs_Firmware/px4_fmu-v6xrt_default.map` | Symbol map |

### Setup Documents

> **Note:** PDF guides available in the v1.16.x branch. To be carried over after v1.17.0 build.

| Document | Description |
|----------|-------------|
| `pxlabs/Bootloader Update Pixhawk V6X-RT via USB.pdf` | Step-by-step bootloader update via USB using MCUXpresso |
| `pxlabs/Burning fuses on RT7.pdf` | Fuse configuration procedure for RT7 variant |

### Custom Patch File

`pxlabs/PXLABS_V6XRT_CUSTOM.patch` — complete diff of all PXLABS changes against the upstream PX4 v1.17.0 base. Use this to review or re-apply changes to future upstream versions:

```bash
# Regenerate patch
git diff px4-v1.17.0..pxlabs-v1.17.0-r1 -- boards/ src/modules/uxrce_dds_client/

# Apply patch to a new upstream branch
git apply --3way pxlabs/PXLABS_V6XRT_CUSTOM.patch
```

---

## Rover Controllers

| Module | Path | Type |
|--------|------|------|
| Ackermann | `src/modules/rover_ackermann/` | Car-like steering |
| Differential | `src/modules/rover_differential/` | Skid-steer |
| Mecanum | `src/modules/rover_mecanum/` | Omnidirectional |

---

## Changelog

### pxlabs-v1.17.0-r1 — 2026-05-24 (Current Stable)

- Rebased onto PX4 v1.17.0 upstream
- All PX4 v1.17.0 improvements included (Zenoh default, RC deadzone, MAVLink v2 default, barometer auto-cal)
- V6XRT sensor SPI bus/rotation corrections preserved for all 3 hardware variants
- Optical flow enabled in default build
- Rover Ackermann + Differential modules enabled in default build
- DDS publications added: `input_rc`, `rover_throttle_setpoint`, `rover_steering_setpoint`
- ist8310 external compass disabled
- No collision prevention (stable — planned for pxlabs-v1.17.0-r2-Beta)

### pxlabs-v1.16.1-r2-Beta

- Rover collision prevention for Ackermann, Differential, Mecanum
- Uses standard PX4 `CP_*` parameters
- **WARNING: Not yet tested on hardware**

### pxlabs-v1.16.1-r1 — Previous Stable

- Based on PX4 v1.16.1 upstream
- All PX4 v1.16.1 bug fixes
- V6XRT sensor configuration corrections
- No collision prevention

### pxlabs-v1.16.0-r1 — Initial Release

- First PXLABS fork from PX4 v1.16.0
- Ackermann and Differential rover support
- DDS topic updates for rover platforms
- Custom V6XRT driver corrections

---

*This repository is maintained by PXLABS and is not officially affiliated with the PX4 project.*
