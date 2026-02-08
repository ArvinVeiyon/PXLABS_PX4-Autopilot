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
| Upstream | https://github.com/PX4/PX4-Autopilot |
| Maintainer | PXLABS |

## Getting Started

### Prerequisites

- Ubuntu 20.04 / 22.04 (recommended)
- Git
- Python 3.8+
- ARM GCC Toolchain (for hardware builds)

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

### Build for Hardware

```bash
# For Pixhawk 4
make px4_fmu-v5_default

# For other boards, check available targets
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

## Changelog

### pxlabs-v1.16.0-r1 (Initial Release)

- Initial fork from PX4 v1.16.0
- Added Ackermann steering rover support
- Added differential drive rover support
- Updated DDS configuration for rover platforms
- Custom driver updates

---

*This repository is maintained by PXLABS and is not officially affiliated with the PX4 project.*
