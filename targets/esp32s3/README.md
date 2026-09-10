# hal-target-esp32s3

Strict, MCU-level ESP32-S3 implementations of the `ecu-drivers` contracts.

SoC implementations and SoC facts are separate from module facts. The initial module profile is
`ESP32-S3-WROOM-1-N16R8`; a firmware BSP later selects that profile and owns all PCB wiring.

ESP-IDF v5.5.5 is pinned under `third_party/` for the supported build/startup environment and official
SoC register definitions. Public target objects do not depend on FreeRTOS, allocate driver handles,
or expose ESP-IDF types.

## Status

This is the board-independent strict-profile skeleton. Direct GPIO register access and the bounded
continuous ADC publication primitive are implemented. The register-level ADC/trigger/DMA acquisition
engine and other incomplete on-chip capabilities are marked `planned`. The ESP32-S3 has no on-chip
Ethernet MAC and no CAN-FD controller, so those capabilities are marked `unsupported` rather than
represented by fake drivers.

```cmake
add_subdirectory(path/to/ecu-drivers)
target_link_libraries(firmware PRIVATE hal::esp32s3-wroom-1-n16r8)
```
