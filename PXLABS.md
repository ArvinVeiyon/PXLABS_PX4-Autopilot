# PXLABS PX4-Autopilot

## Overview

This repository is a customized fork of [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) maintained by **PXLABS**. It targets the **NXP FMU-V6XRT** flight controller with enhanced rover support for Ackermann, Differential, and Mecanum platforms.

> See [`HARDFAULT.md`](HARDFAULT.md) for the investigation and fix behind the
> recurring hard fault resolved in `v1.17.0-2.1.0`.

## Repository Information

| Property | Value |
|----------|-------|
| Latest Release | `v1.17.0-2.1.0` (2026-08-29) |
| Latest Beta | `pxlabs-v1.17.0-r2-Beta` (2026-05-29) |
| Development | `pxlabs-v1.17.0-dev` |
| Upstream Base | PX4 v1.17.0 |
| Target Board | NXP FMU-V6XRT |
| Hardware Verified | Yes — 2026-05-24 |
| Upstream Repo | https://github.com/PX4/PX4-Autopilot |
| Maintainer | PXLABS |

## Version Naming Convention

PXLABS uses two parallel tag schemes:

| Tag Format | Example | Purpose |
|------------|---------|---------|
| `pxlabs-v<px4>-r<N>[-Beta]` | `pxlabs-v1.17.0-r2-Beta` | Internal PXLABS release identifier |
| `v<px4>-<major>.<minor>.<patch>` | `v1.17.0-2.0.0` | QGC-parseable tag — controls **Custom Fw. Ver.** display |

> A third scheme, a plain numeric alias tag (`<px4>.<major>.<minor>.<patch>`, e.g.
> `1.17.0.2.0`), was used through the v2.0.0 release but discontinued as of v2.1.0 —
> it was redundant with the `v<px4>-<major>.<minor>.<patch>` tag above and added no
> information. Existing `1.17.0.2.0` etc. tags from before v2.1.0 are left in place as
> history; no new numeric-alias tags will be created going forward.

> **How QGC version display works:** PX4's cmake picks up the nearest `v*` git tag at configure time and bakes it into the firmware. QGC reads the custom version part (after `v1.17.0-`) and displays it as **Custom Fw. Ver.**
> - Tag `v1.17.0-2.0.0` → QGC shows `2.0.0`
> - Tag `v1.17.0-2.0.0-beta1` → QGC shows `2.0.0 (beta)`
> No source code changes are needed — version display is purely tag-driven.

## Branch & Tag Structure

| Branch / Tag | Type | Description |
|--------------|------|-------------|
| `px4-v1.17.0` | Branch | Clean upstream PX4 v1.17.0 — reference base, no PXLABS changes |
| `pxlabs-v1.17.0-r1` | Branch + Tag | Stable release 1 — hardware verified 2026-05-24 |
| `pxlabs-v1.17.0-r2-Beta` | Tag | Beta release 2 — esc_status DDS, tested 2026-05-29 |
| `v1.17.0-2.1.0` | Tag | **Latest release** — QGC shows `2.1.0` (2026-08-29) |
| `pxlabs-v1.17.0-2.1.0` | Branch | Release branch for v2.1.0 |
| `v1.17.0-2.0.0` | Tag | Previous release — QGC shows `2.0.0` (2026-05-31) |
| `1.17.0.2.0` | Tag | Numeric alias for release 2.0.0 (legacy scheme, discontinued as of v2.1.0) |
| `pxlabs-v1.17.0-2.0.0` | Branch | Release branch for v2.0.0 |
| `pxlabs-v1.17.0-dev` | Branch | Active development for next release |
| `pxlabs-v1.16.1-r1` | Branch + Tag | Previous stable (PX4 v1.16.1 based) |
| `pxlabs-v1.16.1-r2-Beta` | Branch + Tag | Previous beta — collision prevention (untested) |
| `pxlabs-v1.16.0-r1` | Branch + Tag | Initial release archive |

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

# Bootloader
make px4_fmu-v6xrt_bootloader

# Upload via USB
make px4_fmu-v6xrt_default upload

# SITL simulation (Gazebo headless + separate GUI)
make px4_sitl gz_rover_differential   # launches PX4 + Gazebo server
gz sim -g                             # launch Gazebo GUI separately

# List all targets
make list_config_targets
```

---

## PXLABS Resources (`pxlabs/` folder)

```
pxlabs/
├── Documents/
│   ├── Bootloader Update Pixhawk V6X-RT via USB _ PX4 Guide (main).pdf
│   ├── bootloader_update_v6xrt.md
│   └── Burning fuses on RT7.pdf
├── Parameters/
│   └── PXlabs_Differential_Rover_NXP_tested_2026-05-24.params
├── NXP_Bootloader/
│   └── px4_fmu-v6xrt_bootloader.bin
├── PXLabs_Bootloader/
│   ├── px4_fmu-v6xrt_bootloader.bin
│   ├── px4_fmu-v6xrt_bootloader.elf
│   ├── px4_fmu-v6xrt_bootloader.map
│   └── px4_fmu-v6xrt_bootloader.px4
├── PXLabs_Firmware/
│   ├── px4_fmu-v6xrt_default.bin
│   ├── px4_fmu-v6xrt_default.px4
│   ├── px4_fmu-v6xrt_default.elf
│   └── px4_fmu-v6xrt_default.map
└── PXLABS_V6XRT_CUSTOM.patch
```

### Parameters

| File | Vehicle | Status | Date |
|------|---------|--------|------|
| [`PXlabs_Differential_Rover_NXP_tested_2026-05-29.params`](pxlabs/Parameters/PXlabs_Differential_Rover_NXP_tested_2026-05-29.params) | Differential Rover — NXP FMU-V6XRT | Tested ✓ | 2026-05-29 |
| [`PXlabs_Differential_Rover_NXP_tested_2026-05-24.params`](pxlabs/Parameters/PXlabs_Differential_Rover_NXP_tested_2026-05-24.params) | Differential Rover — NXP FMU-V6XRT | Tested ✓ | 2026-05-24 |

To load parameters: open QGroundControl → Vehicle Setup → Parameters → Tools → Load from file.

### Pre-compiled Bootloaders

Built with GCC arm-none-eabi 9.3.1 on Ubuntu 24.04 — 2026-05-24.

| Bootloader | Path | Notes |
|------------|------|-------|
| NXP Bootloader | `pxlabs/NXP_Bootloader/px4_fmu-v6xrt_bootloader.bin` | Provided by NXP, compiled with GCC 9.3.1 — **use this if experiencing boot issues** |
| PXLabs Bootloader | `pxlabs/PXLabs_Bootloader/px4_fmu-v6xrt_bootloader.bin` | Built by PXLABS with GCC 9.3.1 — flash: 82KB / 128KB (63.14%) |

### Pre-compiled Firmware

Built with GCC arm-none-eabi 9.3.1 on Ubuntu 24.04 — 2026-08-29 (v1.17.0-2.1.0).

| File | Path | Size | Description |
|------|------|------|-------------|
| Firmware binary | `pxlabs/PXLabs_Firmware/px4_fmu-v6xrt_default.bin` | 2.4 MB | Flash via QGC |
| PX4 package | `pxlabs/PXLabs_Firmware/px4_fmu-v6xrt_default.px4` | 2.2 MB | Flash via QGC |
| ELF (debug) | `pxlabs/PXLabs_Firmware/px4_fmu-v6xrt_default.elf` | 57 MB | For GDB debugging |
| Memory map | `pxlabs/PXLabs_Firmware/px4_fmu-v6xrt_default.map` | 11 MB | Symbol map |

Flash usage: **62.01%** flash · **85.42%** ITCM · **5.50%** SRAM

### Setup Documents

| Document | Description |
|----------|-------------|
| [`Bootloader Update Pixhawk V6X-RT via USB.pdf`](pxlabs/Documents/Bootloader%20Update%20Pixhawk%20V6X-RT%20via%20USB%20_%20PX4%20Guide%20(main).pdf) | Step-by-step bootloader flash via USB using MCUXpresso (with screenshots) |
| [`bootloader_update_v6xrt.md`](pxlabs/Documents/bootloader_update_v6xrt.md) | Bootloader update guide in markdown format |
| [`Burning fuses on RT7.pdf`](pxlabs/Documents/Burning%20fuses%20on%20RT7.pdf) | Fuse configuration procedure for MIMXRT1176 (RT7 variant) |

---

## Hardware Setup Procedures

### 1. Burning Fuses on MIMXRT1176 (First-time board setup)

> **Required for new boards before flashing bootloader.**

**Tool required:** [NXP MCUXpresso Secure Provisioning Tool](https://www.nxp.com/design/software/development-software/mcuxpresso-software-and-tools-/mcuxpresso-secure-provisioning-tool:MCUXPRESSO-SECURE-PROVISIONING#downloads)

**Steps:**

1. Install and launch **MCUXpresso Secure Provisioning**
2. Select **MIMXRT1176** → click **Create**
3. Select **OTP Configuration** → press **Yes** and wait for script to finish
4. Navigate to fuse address **`0x960`**
5. Set **`BT_FUSE_SEL`** → Required value: **1**
6. Switch to **Advanced Mode** (bottom-left corner)
7. Press **Close** → press **Generate Script**
8. Script is generated at:
   ```
   C:\Users\<username>\secure_provisioning\burn_user_OTP_cfg_win.bat
   ```
9. Run `burn_user_OTP_cfg_win.bat` to burn fuses on new boards

> **Note:** This only needs to be done once per board. Do not repeat fuse burning.

---

### 2. Flashing the Bootloader via USB (No debug probe required)

**Tool required:** [NXP MCUXpresso Secure Provisioning Tool](https://www.nxp.com/design/software/development-software/mcuxpresso-software-and-tools-/mcuxpresso-secure-provisioning-tool:MCUXPRESSO-SECURE-PROVISIONING#downloads)

**Bootloader file:** `pxlabs/PXLabs_Bootloader/px4_fmu-v6xrt_bootloader.bin`
(or use `pxlabs/NXP_Bootloader/px4_fmu-v6xrt_bootloader.bin` if experiencing boot issues)

#### Step 1 — Enter ISP Bootloader Mode

**Option A — Board is working (via QGC):**
1. Connect board → open QGroundControl
2. Go to **Analyze Tools** → **MAVLink Console**
3. Type `reboot -i` and press Enter
4. Board enters ISP bootloader mode

**Option B — Board is bricked:**
1. Open the FMUM module
2. Press and hold the **BOOT button** while powering the board

#### Step 2 — Flash with MCUXpresso

1. Launch **MCUXpresso Secure Provisioning**
2. Create **New Workspace** → select `i.MX RT11xx` → `MIMXRT1176`
3. Click **FlexSPI NOR - simplified**
4. In Boot Memory Configuration set Device type to **`Macronix Octal DDR`** → press **OK**
5. Go to **Tools → Flash Programmer**
6. Press **YES** on the ISP mode popup → press **Yes** on target memory config
7. Press **Erase All** — wait for completion
8. Press **Load...** → **Browse** → select `px4_fmu-v6xrt_bootloader.bin` → **Load**
9. Confirm **"Success: load from file"** message
10. Press **Write** to flash
11. Confirm **"Success: Write memory 0x30000000 - 0x3XXXXXXX"**
12. Unplug and re-power the board

#### Step 3 — Load PX4 Firmware via QGroundControl

1. Open **QGroundControl**
2. Go to **Vehicle Setup** → **Firmware**
3. Connect board via USB — QGC will detect it
4. Select **PX4 Pro Stable Release** or use **Advanced** to load custom firmware
5. To load PXLABS firmware: select **Advanced** → **Custom firmware file** → browse to:
   ```
   pxlabs/PXLabs_Firmware/px4_fmu-v6xrt_default.px4
   ```
6. Click **OK** — QGC will flash and reboot the board

---

### 3. Building Bootloader from Source

```bash
export PATH="/opt/gcc-arm-none-eabi-9-2020-q2-update/bin:$PATH"

# Build bootloader
make px4_fmu-v6xrt_bootloader

# Output: build/px4_fmu-v6xrt_bootloader/px4_fmu-v6xrt_bootloader.bin

# Optional: convert to HEX
arm-none-eabi-objcopy -O ihex \
  build/px4_fmu-v6xrt_bootloader/px4_fmu-v6xrt_bootloader.elf \
  px4_fmu-v6xrt_bootloader.hex
```

### Custom Patch File

`pxlabs/PXLABS_V6XRT_CUSTOM.patch` — complete diff of all PXLABS changes against the upstream PX4 v1.17.0 base. Use this to review or re-apply changes to future upstream versions:

```bash
# Regenerate patch
git diff px4-v1.17.0..pxlabs-v1.17.0-r1 -- boards/ src/modules/uxrce_dds_client/

# Apply patch to a new upstream branch
git apply --3way pxlabs/PXLABS_V6XRT_CUSTOM.patch
```

---

## Verified Hardware Status — NXP FMU-V6XRT (pxlabs-v1.17.0-r1)

Live `work_queue status` output captured from hardware running pxlabs-v1.17.0-r1 — confirms all PXLABS sensor corrections are working correctly.

### Sensor Verification

| Bus | Driver | Rate | Status |
|-----|--------|------|--------|
| SPI1 | `icm42688p` | 395.5 Hz | ✓ Confirmed (PXLABS fix: was ICM42686P in upstream) |
| SPI2 | `icm45686` | 400.0 Hz | ✓ Confirmed (PXLABS fix: was ICM42688P in upstream) |
| SPI3 | `bmi088_accel` | 400.0 Hz | ✓ Confirmed (PXLABS fix: bus 3, rotation 12) |
| SPI3 | `bmi088_gyro` | 400.8 Hz | ✓ Confirmed (PXLABS fix: bus 3, rotation 12) |
| I2C3 | `bmm350` | 50.0 Hz | ✓ Confirmed (PXLABS addition) |
| I2C2/I2C3 | `bmp390` | 23.1 Hz | ✓ Running (barometer) |
| I2C1/I2C2 | `ina228` | 2.0 Hz | ✓ Running (power monitor) |

### Rover & Control Verification

| Module | Rate | Status |
|--------|------|--------|
| `rover_differential` | 100.0 Hz | ✓ Running on wq:rate_ctrl |
| `vehicle_angular_velocity` | 400.0 Hz | ✓ Running |
| `ekf2` | 200.0 Hz | ✓ Running |
| `vehicle_imu` (×3) | 197–274 Hz | ✓ All 3 IMUs active |
| `vehicle_magnetometer` | 50.0 Hz | ✓ Running |
| `vehicle_gps_position` | 3.3 Hz | ✓ Running |

### Full work_queue Output (Reference)

```
Work Queue: 12 threads                          RATE        INTERVAL
|__ 1) wq:rate_ctrl
|   |__ 1) pwm_out                           20.0 Hz        49997 us
|   |__ 2) rover_differential               100.0 Hz        10000 us (10000 us)
|   \__ 3) vehicle_angular_velocity         400.0 Hz         2500 us
|__ 2) wq:SPI1
|   \__ 1) icm42688p                        395.5 Hz         2528 us
|__ 3) wq:SPI2
|   \__ 1) icm45686                         400.0 Hz         2500 us (2500 us)
|__ 4) wq:SPI3
|   |__ 1) bmi088_accel                     400.0 Hz         2500 us (2500 us)
|   \__ 2) bmi088_gyro                      400.9 Hz         2495 us
|__ 5) wq:I2C1
|   \__ 1) ina228                             2.0 Hz       501415 us
|__ 6) wq:I2C2
|   |__ 1) bmp390                            23.1 Hz        43295 us (43300 us)
|   \__ 2) ina228                             2.0 Hz       500138 us
|__ 7) wq:I2C3
|   |__ 1) bmm350                            50.0 Hz        19998 us (20000 us)
|   \__ 2) bmp390                            23.1 Hz        43299 us (43300 us)
|__ 8) wq:nav_and_controllers
|   |__ 1) land_detector                    100.0 Hz        10000 us
|   |__ 2) sensors                          200.0 Hz         5000 us
|   |__ 3) vehicle_acceleration             200.0 Hz         5000 us
|   |__ 4) vehicle_air_data                  23.1 Hz        43298 us
|   |__ 5) vehicle_gps_position               3.3 Hz       299671 us
|   \__ 6) vehicle_magnetometer              50.0 Hz        19999 us
|__ 9) wq:INS0
|   |__ 1) ekf2                             200.0 Hz         5000 us
|   |__ 2) vehicle_imu                      197.8 Hz         5057 us
|   |__ 3) vehicle_imu                      200.0 Hz         5000 us
|   \__ 4) vehicle_imu                      274.0 Hz         3650 us
|__ 10) wq:hp_default
|   |__ 1) battery_status                   100.0 Hz        10000 us
|   |__ 2) board_adc                        100.0 Hz        10001 us (10000 us)
|   |__ 3) manual_control                     5.0 Hz       199895 us
|   |__ 4) rc_update                          0.0 Hz            0 us
|   |__ 5) safety_button                     30.3 Hz        33001 us (33000 us)
|   \__ 6) tone_alarm                         0.0 Hz            0 us
|__ 11) wq:uavcan
|   |__ 1) uavcan                           333.2 Hz         3001 us (3000 us)
|   |__ 2) uavcan-actuators-esc              20.0 Hz        50002 us
|   \__ 3) uavcan-actuators-servo             3.3 Hz       299567 us (300000 us)
\__ 12) wq:lp_default
    |__ 1) cdcacm_autostart                   2.0 Hz       499933 us
    |__ 2) gyro_calibration                  49.9 Hz        20036 us (20000 us)
    |__ 3) load_mon                           2.0 Hz       499871 us (500000 us)
    |__ 4) mag_bias_estimator                49.9 Hz        20036 us (20000 us)
    |__ 5) parameters                         0.0 Hz            0 us
    \__ 6) send_event                        29.9 Hz        33391 us (33333 us)
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

### v1.17.0-2.1.0 — 2026-08-29 (Latest Release)

- **Fixes recurring intermittent hard fault** (`UNDEFINSTR`/`NOCP`, random ~5–15 min
  interval, reproduced on 4 flight controllers across 2 sites) by cherry-picking
  upstream PR [PX4/PX4-Autopilot#28141](https://github.com/PX4/PX4-Autopilot/pull/28141)
  — calibrates the FlexSPI DLL read strobe at boot instead of trusting the ROM's
  default "locked but not centered" placement. Root cause, investigation, and
  verification fully documented in [`HARDFAULT.md`](HARDFAULT.md).
- Confirmed via 8+ hour continuous hardware soak test, 2026-08-28→29: zero fault logs,
  zero unexpected reboots.
- Firmware stats: flash 61.80% → **62.01%** (+8664 B), SRAM 5.39% → **5.50%**
  (+2112 B), ITCM unchanged 85.42%.
- **Known gap:** this fix is not yet present in any current upstream v1.18 tag
  (`alpha1`/`beta1`/`beta2` all predate the PR) — see `HARDFAULT.md` for what to check
  before ever building from a v1.18 base.

### v1.17.0-2.0.0 — 2026-05-31

- QGC **Custom Fw. Ver.** displays `2.0.0` — driven purely by git tag, no source changes
- Based on exact `pxlabs-v1.17.0-r2-Beta` source — zero additional code modifications
- Firmware stats: flash 61.90% · ITCM 85.42% · SRAM 5.39%

### pxlabs-v1.17.0-r2-Beta — 2026-05-29 (Beta)

- Added DDS publication: `/fmu/out/esc_status` — VESC UAVCAN motor RPM/current telemetry for companion wheel odometry
- Firmware rebuilt: flash 61.90% (+0.10% from r1)
- New tested parameter file: `PXlabs_Differential_Rover_NXP_tested_2026-05-29.params`

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
