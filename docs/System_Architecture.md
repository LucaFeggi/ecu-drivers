# System Architecture

`ecu-drivers` is a static, portable C++20 driver library. It contains portable
contracts and adapters, target implementations, exact MCU/module profiles, and
external-device drivers. The repository does not contain a software-in-the-
loop backend; that is a separate future project.

## Layering

```text
Application and external-device drivers
              |
              v
Portable contracts and adapters in include/hal/
              |
              v
Target implementations in platforms/
              |
              v
Exact MCU/module facts in mcu/
              |
              v
BSP/composition: pins, clocks, interrupts, DMA, memory, wiring, policy
```

The portable layer uses C++20 concepts, concrete stateful types, static
dispatch, caller-owned storage, and `hal::result` error values. It does not
depend on vendor headers, operating-system headers, an RTOS, exceptions, RTTI,
virtual dispatch, or hidden heap allocation. Target extensions remain available
when a peripheral capability cannot be represented portably.

The `devices/` layer contains reusable external-device drivers such as the
ST7735 display driver. It depends on portable contracts only; it does not
select MCU peripherals, pins, boards, or RTOS services.

## Repository components

- `include/hal/contracts/` — GPIO, I2C, SPI, serial, CAN, Ethernet, ADC, block,
  nonvolatile storage, PWM, RTC, time, and watchdog contracts.
- `include/hal/adapters/` — allocation-free composition such as active-low
  GPIO, ADC averaging, journal memory, PWM duty-cycle, serial text output, and
  static SPI-device ownership.
- `include/hal/foundation/` — result values, spans, units, assertions,
  saturating monotonic deadlines, and caller-owned DMA storage/coherency
  policies.
- `platforms/linux/` — Linux integration through GPIO v2, i2c-dev, spidev,
  SocketCAN, TAP/AF_PACKET, IIO, termios, sysfs, RTC, watchdog, files, and
  block devices.
- `platforms/esp32/s3/` — direct-register ESP32-S3-WROOM-1-N16R8 drivers using
  raw ESP-IDF SoC declarations and a shared GDMA layer.
- `platforms/stm32/f4/` — direct-register STM32F4 drivers using legacy DMA
  streams and classic bxCAN.
- `platforms/stm32/g4/` — direct-register STM32G4 drivers using DMA/DMAMUX,
  timer-triggered ADC, and FDCAN.
- `platforms/stm32/h5/` — direct-register STM32H5 drivers using GPDMA,
  reduced-layout FDCAN, SDMMC, and Ethernet MAC/MDIO.
- `platforms/stm32/h7/` — direct-register STM32H7 drivers using ADC1/2 timer
  triggering, DMA streams, FDCAN, SDMMC, and Ethernet MAC/MDIO.
- `mcu/` — exact device/module bindings, capabilities, memory regions, DMA
  facts, interrupt numbers, and pin facts.
- `devices/` — portable external-device drivers.
- `tests/` and each target's `tests/` — contract, adapter, integration, and
  compile/register-model validation.

## Target and BSP ownership

Family packages own reusable register-level protocol and peripheral logic.
Exact profiles bind CMSIS or SoC register types and describe device facts.
Neither layer is a board-support package.

The BSP or firmware composition owns:

- exact target/profile selection;
- clock trees, reset sequencing, and peripheral instances;
- GPIO alternate functions and physical pin wiring;
- DMA channels, requests, interrupt routing, and ISR dispatch;
- cache/MPU attributes and placement of DMA buffers, descriptor rings, and
  message RAM;
- external transceivers, PHY policy, storage wiring, and application policy;
- FreeRTOS tasks, queues, notifications, and scheduler configuration.

Drivers are independent of FreeRTOS. Synchronous contract calls complete with
bounded polling or return a nonblocking result; continuous acquisition and
receive paths expose explicit ISR publication/service boundaries.

Drivers that can observe a running monotonic clock may use an absolute
`hal::deadline`; frequency-dependent poll budgets remain valid for bounded
early-startup sequences where no clock service exists. DMA-capable drivers do
not assume cache coherence: the BSP either supplies a cache-line-aware
`DmaCoherencyPolicy` or proves that the caller-owned region is hardware
coherent/MPU-configured non-cacheable before using the no-op policy.

Portable RTC alarms mean one absolute, future UTC deadline. Targets backed by
calendar hardware filter recurring day-of-month matches in software and
disarm the hardware alarm after the requested deadline is observed.

## CMake targets

The repository exports only concrete device/module integration targets:

```text
hal::linux
hal::stm32f446vet6
hal::stm32g484ce
hal::stm32h563vit6
hal::stm32h723vgt6
hal::esp32s3_wroom_1_n16r8
```

Firmware composition should link only the selected concrete target. Core
contracts and external-device headers are implementation/package contents and
are not separate target selections.

`ECU_DRIVERS_TARGETS` controls configuration and installation. Install-tree
tests build independent consumers against isolated include roots, so an exact
profile cannot accidentally rely on a source-tree-relative family include or
an unexported core/device dependency.

## Validation boundary

Host tests validate portable contracts, adapters, Linux integration behavior,
and register-model instantiation. Target documentation identifies the
remaining hardware validation boundary: clock/reset behavior, exact pin and
DMA routing, interrupt priorities, cache coherency, ADC calibration, flash
execution policy, SD-card behavior, PHY negotiation, and external wiring.

The target architecture documents under each `platforms/*/docs/` directory are
the detailed driver-by-driver references for that implementation.

CTest labels describe evidence rather than aspiration:
`compile_validated`, `register_model_validated`, `host_validated`, and
`hil_validated`. Only a runner that flashes real hardware and verifies the
declared fixture may use the last label.
