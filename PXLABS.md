# PXLABS PX4-Autopilot v1.16.0

## Overview

This repository is a customized fork of [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) maintained by **PXLABS**. It is based on PX4 v1.16.0 and includes custom modifications for specialized vehicle platforms.

## Modifications

### Rover Support Enhancements

- **Ackermann Steering Rover**: Added support for Ackermann steering geometry vehicles
- **Differential Drive Rover**: Added support for differential drive rover platforms
- **DDS Updates**: Updated DDS (Data Distribution Service) configuration for rover platforms
- **Driver Updates**: Custom driver modifications for enhanced vehicle control

## Repository Information

| Property | Value |
|----------|-------|
| Base Version | PX4 v1.16.0 |
| Branch | pxlabs-v1.16.0-r1 |
| Target Board | NXP FMU-V6XRT |
| Upstream | https://github.com/PX4/PX4-Autopilot |
| Maintainer | PXLABS |

## Getting Started

### Prerequisites

- Ubuntu 20.04 / 22.04 (recommended)
- Git
- Python 3.8+
- ARM GCC Toolchain **v9.3.1** (for hardware builds) - See [Toolchain Requirements](#toolchain-requirements)

## Toolchain Requirements

> **CRITICAL: PX4 v1.16.0 requires GCC 9.3.1 (`arm-none-eabi-gcc`)**

### Known Issue: Compiler Version Incompatibility

Using newer GCC compiler versions with PX4 v1.16.0 causes critical issues affecting both bootloader and application code. This was identified during production testing on FMU-V6XRT boards.

| Issue | Description |
|-------|-------------|
| **Affected Boards** | FMU-V6XRT (and potentially other targets) |
| **Root Cause** | Newer GCC compiler versions introduce bugs in generated code |
| **Symptoms** | Board fails to boot or behaves unexpectedly |
| **Solution** | Use GCC 9.3.1 for all builds |

### Recommended Toolchain Setup

```bash
# Verify your current ARM GCC version
arm-none-eabi-gcc --version

# Expected output should show: arm-none-eabi-gcc (GNU Arm Embedded Toolchain 9-2020-q2-update) 9.3.1
```

### Installing GCC 9.3.1

If you have a different version, install the correct toolchain:

```bash
# Remove existing toolchain (if any)
sudo apt remove gcc-arm-none-eabi

# Download GCC 9.3.1 (9-2020-q2-update)
wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2020q2/gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2

# Extract to /opt
sudo tar -xjf gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2 -C /opt

# Add to PATH (add to ~/.bashrc for persistence)
export PATH="/opt/gcc-arm-none-eabi-9-2020-q2-update/bin:$PATH"

# Verify installation
arm-none-eabi-gcc --version
```

### FMU-V6XRT Bootloader

If you experience boot issues on FMU-V6XRT, ensure the bootloader is compiled with GCC 9.3.1:

```bash
# Build bootloader with correct toolchain
make px4_fmu-v6xrt_bootloader
```

## PXLABS Resources

All PXLABS-specific resources are located in the `pxlabs/` folder.

### Pre-compiled Bootloaders

| Bootloader | Path | Description |
|------------|------|-------------|
| **NXP Bootloader** | `pxlabs/NXP_Bootloader/px4_fmu-v6xrt_bootloader.bin` | Provided by NXP (Peter), compiled with GCC 9.3.1 - **Use this if experiencing boot issues** |
| **PXLabs Bootloader** | `pxlabs/PXLabs_Bootloader/px4_fmu-v6xrt_bootloader.bin` | Built by PXLABS with GCC 9.3.1 after NXP recommendation |

> **Recommendation:** Use the NXP bootloader if you experience boot failures. The PXLabs bootloader includes additional build artifacts (.elf, .map, .px4) for debugging.

### Pre-compiled Firmware

| Firmware | Path | Description |
|----------|------|-------------|
| **PXLabs Firmware** | `pxlabs/PXLabs_Firmware/px4_fmu-v6xrt_default.bin` | FMU-V6XRT firmware with rover modifications |

Build artifacts included:
- `px4_fmu-v6xrt_default.bin` - Binary firmware file
- `px4_fmu-v6xrt_default.px4` - PX4 firmware package
- `px4_fmu-v6xrt_default.elf` - ELF file for debugging
- `px4_fmu-v6xrt_default.map` - Memory map file

### Setup Procedure Documents

Important setup and configuration guides for NXP FMU-V6XRT:

| Document | Description |
|----------|-------------|
| [Bootloader Update Guide](pxlabs/Bootloader%20Update%20Pixhawk%20V6X-RT%20via%20USB%20_%20PX4%20Guide%20(main).pdf) | Step-by-step guide for updating bootloader via USB |
| [Burning Fuses on RT7](pxlabs/Burning%20fuses%20on%20RT7.pdf) | Fuse configuration procedure for RT7 |

> **Important:** Review these documents before performing hardware setup or bootloader updates.

### Clone Repository

```bash
git clone https://github.com/ArvinVeiyon/PXLABS_PX4-Autopilot_v1.16.0.git
cd PXLABS_PX4-Autopilot_v1.16.0
git checkout pxlabs-v1.16.0-r1
git submodule update --init --recursive
```

### Setup Development Environment

```bash
# Install dependencies
bash ./Tools/setup/ubuntu.sh

# Build for SITL (Software In The Loop)
make px4_sitl_default
```

### Build for Hardware (NXP FMU-V6XRT)

```bash
# Build firmware for NXP FMU-V6XRT
make px4_fmu-v6xrt_default

# Build bootloader for NXP FMU-V6XRT
make px4_fmu-v6xrt_bootloader

# Clean build (if needed)
make px4_fmu-v6xrt_default clean

# Upload firmware via USB
make px4_fmu-v6xrt_default upload

# List all available targets
make list_config_targets
```

## Simulation

### Gazebo Simulation

```bash
make px4_sitl gazebo
```

### Rover Simulation

```bash
# For differential drive rover
make px4_sitl gazebo_rover

# For Ackermann rover
make px4_sitl gazebo_rover_ackermann
```

## Custom Modules

### Rover Controllers

The following custom rover modules are included:

1. **Differential Drive Controller**
   - Path: `src/modules/rover_diff_drive/`
   - Description: Controls differential drive rovers with independent wheel speeds

2. **Ackermann Steering Controller**
   - Path: `src/modules/rover_ackermann/`
   - Description: Controls Ackermann steering geometry vehicles

## Configuration

### Vehicle Configuration Files

Custom vehicle configurations are located in:
- `ROMFS/px4fmu_common/init.d-posix/airframes/` - SITL airframes
- `ROMFS/px4fmu_common/init.d/airframes/` - Hardware airframes

## Contributing

1. Fork this repository
2. Create a feature branch (`git checkout -b feature/your-feature`)
3. Commit your changes (`git commit -am 'Add new feature'`)
4. Push to the branch (`git push origin feature/your-feature`)
5. Create a Pull Request

## License

This project inherits the BSD 3-Clause License from PX4-Autopilot.

## Contact

For questions and support related to PXLABS modifications, please open an issue in this repository.

## Development Branch (pxlabs-v1.16.0-dev)

### Rover Collision Prevention - IN PROGRESS

**Status:** Partially implemented, needs update to use standard CP_* parameters

#### Completed Work

1. **Created RoverCollisionPrevention library** (`src/lib/rover_collision_prevention/`)
   - `RoverCollisionPrevention.hpp` - Header file
   - `RoverCollisionPrevention.cpp` - Implementation
   - `rover_collision_prevention_params.c` - Parameters (currently RCP_*)
   - `CMakeLists.txt` - Build configuration

2. **Integrated into all rover modules:**
   - Rover Ackermann (`AckermannVelControl`)
   - Rover Differential (`DifferentialVelControl`)
   - Rover Mecanum (`MecanumPosVelControl`)

3. **Build tested successfully** for px4_fmu-v6xrt_default

#### Pending Work (TODO)

None at this time.

#### Codex CLI Suggestions (PX4 Collision Prevention Parity)

- Distance sensor FOV handling: currently each distance sensor contributes to a single bin; PX4 standard behavior spreads readings across the sensor `h_fov` bins. Consider updating rover collision prevention to fill bins across FOV.
- Delay compensation semantics: current rover logic uses `CP_DELAY` with a simple distance reduction based on speed; PX4 multicopter collision prevention models delay in its kinematic constraints. Consider aligning rover delay compensation more closely.
- Config migration: existing rover configs that relied on `RCP_GO_NO_DATA=1` should be reviewed now that `CP_GO_NO_DATA=0` is the default.

#### Current Parameters (CP_*)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `CP_DIST` | -1 (disabled) | Minimum distance to obstacles. Set > 0 to enable |
| `CP_DELAY` | 0.4 s | Sensor delay compensation |
| `CP_GUIDE_ANG` | 30 deg | Guidance angle to steer around obstacles |
| `CP_GO_NO_DATA` | 0 (disabled) | Allow movement without sensor data |

#### Resume Instructions

To continue this work:
1. Checkout dev branch: `git checkout pxlabs-v1.16.0-dev`
2. Modify collision prevention params to use generic group
3. Update rover code to use CP_* parameters
4. Add missing features (delay, guidance, fused publish, timeout)
5. Test build and commit

---

## Changelog

### pxlabs-v1.16.0-r1 (Initial Release)

- Initial fork from PX4 v1.16.0
- Added Ackermann steering rover support
- Added differential drive rover support
- Updated DDS configuration for rover platforms
- Custom driver updates

---

*This repository is maintained by PXLABS and is not officially affiliated with the PX4 project.*
