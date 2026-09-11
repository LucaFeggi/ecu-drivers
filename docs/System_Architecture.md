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

## Why the architecture looks this way

This document also preserves the design path that led to the implementation.
The original architecture study was intentionally comparative: the project
needed a portable C++ interface, but also needed predictable embedded behavior,
static resource ownership, and access to target-specific features. No existing
HAL was copied wholesale because each reviewed project optimizes for a
different boundary (Rust generic drivers, a full RTOS, binary interoperability,
or runtime extensibility).

The central conclusion was that “portable” should describe the semantics used
by a driver, not pretend that unrelated MCUs have identical registers or
capabilities. C++20 concepts therefore describe contracts, while concrete
stateful target objects implement them. Target and board choices are made at
build/composition time, so unsupported combinations fail through types,
concepts, or target facts rather than a runtime target switch.

The main decisions were:

- Use concepts and static dispatch instead of an inheritance hierarchy,
  function-table API, or virtual interface. This keeps the call graph visible,
  avoids vtables, and permits optimization without requiring a global heap.
- Keep contracts small and consumer-oriented. GPIO, I2C, SPI, CAN, serial,
  storage, and similar APIs express operations that reusable device drivers
  actually need; unusual peripheral features remain target extensions.
- Model I2C as transactions so repeated-start and stop behavior cannot be lost
  in vendor-specific flags. Model SPI as a shared bus plus a device transaction
  that owns chip-select and bus exclusivity.
- Keep configuration and intent separate from register layouts. Family drivers
  own register-level mechanics; exact profiles own device facts; BSP/composition
  owns pins, clocks, DMA, interrupts, cache policy, and external wiring.
- Make storage and failures explicit. Normal objects use caller-owned or
  automatic storage, `hal::result` reports operational errors, and blocking
  operations have bounded progress/timeout rules. Startup failure and an
  invariant violation are separate from an ordinary runtime error.
- Keep FreeRTOS above the library. Tasks, queues, notifications, and safe-state
  policy belong to firmware composition; drivers expose explicit service/ISR
  boundaries and do not call the RTOS merely to notify application code.
- Treat DMA/cache coherency, interrupt context, buffer lifetime, and peripheral
  ownership as part of the contract. These details are especially important on
  Cortex-M7 targets and cannot be hidden behind a generic “DMA-capable” label.

### Alternatives reviewed and what was retained

The historical study reviewed Rust `embedded-hal`, modm, libhal, Zephyr,
CMSIS-Driver, RIOT, TinyUSB, C++20 concepts/customization mechanisms, and
freestanding C++ constraints. The resulting choices were deliberately
selective:

- `embedded-hal` supplied the strongest semantic reference: generic external
  device drivers, unified I2C/SPI transaction concepts, separate execution
  models, and the rule that a new contract should be proven on multiple
  platforms with a generic consumer.
- modm demonstrated why generated target facts, static allocation, and
  compile-time pin/peripheral validation scale better than handwritten macro
  trees.
- Zephyr contributed the separation of immutable device configuration, runtime
  state, and board description, but its runtime API tables were rejected.
- CMSIS-Driver contributed disciplined naming, capability/error vocabulary,
  and validation-suite thinking, but its Access Struct function-pointer
  dispatch was rejected.
- libhal contributed layering, adapters, and explicit ownership; virtual
  interfaces, reference-counted resource management, and async-first design
  were outside this project's V1 constraints.
- RIOT reinforced explicit SPI bus acquisition and release. TinyUSB reinforced
  isolating low-level controller code and keeping complex interrupt work out of
  the ISR.

This explains several intentional omissions: there is no universal `Timer`,
raw NOR flash is not presented as EEPROM, CAN FD is a refinement of classical
CAN, and runtime `supports_x()` checks are not used for capabilities known at
build time. These are design boundaries, not missing abstractions.

### How the design was refined into the current repository

The early baseline focused on STM32H7, ESP32-S3, Linux, and a proposed
deterministic simulation backend. Implementation and review exposed the need
for reusable STM32 family packages, so STM32F4, G4, and H5 were added with
their materially different DMA, CAN, ADC, SDMMC, and Ethernet mechanisms.
The current repository is authoritative: Linux is an integration backend and
there is no `sim` backend in this checkout. The simulation idea remains a
possible future project, not a current layer or supported CMake target.

The same refinement moved complete C++ declarations into
`include/hal/<area>/` headers. This Markdown file records semantics and
rationale; the headers are the source of truth for signatures. Contract tests,
adapter tests, install-tree consumer tests, and register-model validation now
provide evidence at different boundaries rather than treating architecture
prose as proof.

## Design workflow and implementation checkpoints

The intended workflow was:

1. Start from the reusable driver's required behavior, not from a vendor HAL.
2. Compare at least two materially different implementations and document
   where the proposed contract breaks down.
3. Write the semantic contract: ownership, blocking and timeout behavior,
   ISR/thread rules, buffer lifetime, error meaning, and post-failure state.
4. Implement the portable header and its compile-time/host contract tests.
5. Implement target family mechanics and bind them to exact MCU/module facts.
6. Add BSP/composition validation for physical resources, DMA, interrupts,
   memory placement, and cache policy.
7. Validate installation boundaries and generated code/register models before
   claiming hardware support; reserve `hil_validated` for a real flashed
   fixture.

This sequence is why the repository has separate `include/hal/contracts`,
`platforms`, `mcu`, `devices`, and test layers. It also makes future changes
auditable: if a backend proves a contract wrong, update the contract, evidence,
and tests together while retaining the reason for the change in history.

## Research resources

The architecture was informed by the following primary resources. They are
kept here so a future maintainer can reconstruct the reasoning rather than
only see the final directory tree:

- [embedded-hal migration rationale](https://github.com/rust-embedded/embedded-hal/blob/master/docs/migrating-from-0.2-to-1.0.md), [adding a new trait](https://github.com/rust-embedded/embedded-hal/blob/master/docs/how-to-add-a-new-trait.md), [I2C](https://docs.rs/embedded-hal/latest/embedded_hal/i2c/trait.I2c.html), and [SPI](https://docs.rs/embedded-hal/latest/embedded_hal/spi/index.html)
- [modm architecture](https://modm.io/how-modm-works/), [libhal fundamentals](https://libhal.github.io/5.0/user_guide/fundamentals/), [Zephyr device model](https://docs.zephyrproject.org/latest/kernel/drivers/index.html) and [Devicetree](https://docs.zephyrproject.org/latest/build/dts/index.html)
- [CMSIS-Driver](https://arm-software.github.io/CMSIS_6/main/Driver/index.html), [RIOT SPI API](https://api.riot-os.org/group__drivers__periph__spi.html), and [TinyUSB architecture](https://docs.tinyusb.org/en/latest/reference/architecture.html)
- [C++20 concepts](https://en.cppreference.com/cpp/language/constraints), [freestanding C++](https://en.cppreference.com/cpp/freestanding), [`std::span`](https://en.cppreference.com/cpp/container/span), and WG21 [customization mechanisms](https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2021/p2279r0.html)
- [STM32H723VG documentation](https://www.st.com/en/microcontrollers-microprocessors/stm32h723vg), [STM32H7 reference/documentation hub](https://www.st.com/en/microcontrollers-microprocessors/stm32h723-733/documentation.html), [STM32H7 cache/DMA guidance](https://www.st.com/resource/en/application_note/an4839-level-1-cache-on-stm32f7-series-and-stm32h7-series-stmicroelectronics.pdf), and [STM32CubeH7](https://github.com/STMicroelectronics/STM32CubeH7)
- [ESP32-S3 module datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf), [ESP-IDF](https://github.com/espressif/esp-idf), and [TWAI documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/twai.html)
- Linux [SocketCAN](https://docs.kernel.org/networking/can.html), [GPIO character device](https://docs.kernel.org/userspace-api/gpio/chardev.html), [spidev](https://docs.kernel.org/spi/spidev.html), [I2C userspace](https://docs.kernel.org/i2c/dev-interface.html), [IIO buffers](https://docs.kernel.org/iio/iio_devbuf.html), and [TUN/TAP](https://docs.kernel.org/networking/tuntap.html)
- [NXP UM10204 I2C specification](https://www.nxp.com/docs/en/user-guide/UM10204.pdf), [Open-CMSIS-SVD](https://open-cmsis-pack.github.io/svd-spec/latest/index.html), [AUTOSAR C++14 guidelines](https://www.autosar.org/fileadmin/standards/R22-11/AP/AUTOSAR_RS_CPP14Guidelines.pdf), and the [CMake Presets manual](https://cmake.org/cmake/help/latest/manual/cmake-presets.7.html)

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
