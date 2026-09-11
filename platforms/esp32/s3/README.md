# hal-target-esp32s3

Strict, direct-register implementations of the `ecu-drivers` contracts for
the selected ESP32-S3 package profile.

The family implementation is independent of a particular MCU ordering code.
The selected package profile supplies flash/PSRAM population and usable GPIO
facts. A firmware BSP still owns the PCB wiring and pin policy.

The ESP-IDF checkout under `third_party/esp-idf` is the ESP-IDF 6.2
development checkout selected for raw SoC register definitions and firmware
startup integration. The strict target includes raw register descriptions only;
it does not include ESP-IDF peripheral drivers, LL headers, FreeRTOS, driver
handles, or ESP-IDF types in its public driver APIs.

## Implemented pipeline

The target provides subfoldered implementations for GPIO/GPIO matrix, I2C,
SPI, UART, TWAI classic CAN, ADC one-shot and continuous DMA, LEDC PWM,
SYSTIMER time, RTC, timer-group watchdog, internal flash, SDMMC block I/O,
and the shared GDMA descriptor/channel layer. SPI, UART TX, and continuous
ADC use DMA; SDMMC uses its own internal IDMAC. Small command/FIFO peripherals
remain register/FIFO driven where DMA would not improve the transaction model.

All DMA buffers and descriptors are caller-owned. The BSP must place them in
DMA-visible internal memory, and must define the cache/PSRAM policy before
using a zero-copy extension. The provided high-throughput paths stage ordinary
application buffers into static internal scratch storage.

Ethernet and CAN-FD are explicit unsupported native capabilities: the ESP32-S3
has no integrated Ethernet MAC and its TWAI block is classic CAN. External
controllers remain portable device adapters over a selected bus.

## Build and contract test

```text
cmake -S . -B build-esp32s3 -DECU_DRIVERS_BUILD_TESTS=ON -DBUILD_TESTING=ON
cmake --build build-esp32s3 --target hal_esp32s3_device_profile_tests
ctest --test-dir build-esp32s3 -R hal_esp32s3_device_profile --output-on-failure
```

The test instantiates every target driver family and validates the exact
device profile at compile time. It does not access the ESP32-S3 address map on
the host.

Firmware integration is provided by
the ESP32-S3 firmware port, where ESP-IDF owns startup/toolchain/linker,
partitions/PSRAM/FreeRTOS and the BSP owns resource and verified board-pin
policy.

See [System Architecture](docs/System_Architecture.md) for the driver-level
implementation and ownership model.
