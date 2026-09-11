# ecu-drivers

`ecu-drivers` is the single repository for the ECU project's portable C++20
driver ecosystem. It contains the target-independent contracts, target
implementations, and portable external-device drivers. Current validation uses
host integration, register models, and physical-board HIL.

## Repository layout

```text
include/                 Portable HAL contracts, adapters, and foundation utilities
platforms/               Linux, STM32 families, and ESP32-S3 implementations
mcu/                     Exact MCU and module profiles
devices/                 Portable drivers for external chips and protocols
tests/                   Core and package conformance tests
third_party/             CMSIS and ESP-IDF target dependencies (Git submodules)
```

The exact MCU or module profile stays inside its target package. The
`devices/` layer is reserved for external devices such as sensors, memories,
displays, transceivers, and external controllers. It does not contain MCU
families or board support. External-device drivers depend on `include/hal`
contracts and never depend on a target or an RTOS.

## CMake targets

The repository preserves separate targets even though it is consumed as one
package:

```text
hal::linux
hal::stm32f446vet6
hal::stm32h563vit6
hal::stm32h723vgt6
hal::stm32g484ce
hal::esp32s3-wroom-1-n16r8
```

Only the selected concrete target should be linked by a firmware composition.
The target-independent layer remains free of vendor headers, operating-system
headers, hidden heap allocation, virtual dispatch, exceptions, RTTI, and
FreeRTOS dependencies. Core contracts and external-device headers are included
transitively by the selected target; they are not separate selection targets.

`ECU_DRIVERS_TARGETS` is a CMake list selecting which concrete targets are
configured and installed. Its default contains all six targets; a constrained
consumer can select one without requiring another target's vendor dependency,
for example:

```sh
cmake -S . -B build-h7 \
  -DECU_DRIVERS_TARGETS=stm32h723vgt6 \
  -DECU_DRIVERS_BUILD_TESTS=ON
```

Installed headers use isolated roots for core, external devices, each target
family, and each exact profile. The `installed_package` CTest group installs to
a temporary prefix and independently configures a consumer for every selected
target, exercising the exported dependency graph and public include paths.

The ECU firmware template consumes this repository as one dependency and
keeps board wiring, startup policy, FreeRTOS integration, and application code
above it.

See [System Architecture](docs/System_Architecture.md) for the ownership and
dependency rules.

Initialize the vendor dependencies with `git clone --recurse-submodules` or
`git submodule update --init --recursive` before configuring a target build.
