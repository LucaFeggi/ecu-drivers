# System Architecture — Static, Portable C++20 Embedded Driver Library

**Status:** architecture baseline; contracts partially implementation-ready (pre-V1)
**Date:** 2026-09-08
**Language baseline:** C++20
**Production/integration targets:** `STM32H723VGT6`, `ESP32-S3-WROOM-1-N16R8`, `Linux`
**Deterministic test backend:** `sim`
**Core memory policy:** static/caller-supplied storage; no hidden heap allocation
**Core dispatch policy:** static dispatch only; no C++ virtual functions, no vtables, no manual runtime function tables
**Core exception/RTTI policy:** designed to build with `-fno-exceptions -fno-rtti`

> This document is a decision record and implementation blueprint. It summarizes the architecture-relevant conclusions from the cited primary documentation and source projects; it is not a claim that every page of every multi-thousand-page vendor reference manual was read line-by-line. Where a peripheral contract matters, the relevant protocol/API sections and target documentation were used to pressure-test the design.

> **Contract source of truth:** the focused `ecu-drivers/include/hal/<capability>.hpp` headers own the
> portable C++ declarations. This document owns semantic rationale, lifecycle rules, target mappings,
> and design constraints. Remaining C++ snippets are illustrative usage, not duplicate declarations.

## 1. Executive decision

The recommended architecture is:

> **C++20 concepts + concrete stateful target-specific types + compile-time target/board composition + generated target metadata + small portable semantic contracts + refinement concepts for genuinely optional capabilities.**

The portable API is **not** a class hierarchy. “One interface” means that the same semantic contract, names, data types, error meanings, and transaction rules are used by every target implementation. A portable device driver is a constrained template and therefore compiles unchanged for every backend that satisfies the required concept.

```text
Portable device/application code
        |
        | C++20 concept requirements
        v
+-------------------------------+
| Portable semantic contracts   |
| GPIO / I2C / SPI / CAN / ...  |
+-------------------------------+
        |
        | compile-time/static dispatch
        v
+----------------+ +-------------+ +----------------+
| STM32H7 target | | ESP32-S3     | | Linux backend  |
| family package | | target pkg   | | real host UAPI |
+----------------+ +-------------+ +----------------+
        |                 |                 |
        v                 v                 |
 STM32H723VGT6      WROOM-1-N16R8           |
 exact profile       module profile          |
        |                 |                  |
        +-----------------+------------------+
                          |
                     BSP/composition
                          |
                          v
                       hardware
```

A **separate `sim` backend** implements the same contracts using deterministic software models and fault injection. Linux is the only host integration backend in the project; host API details used internally by that backend do not create another architectural layer.

This choice follows the most useful lesson of embedded-hal 1.0: **a portable driver interface and a complete MCU HAL are not the same problem.** embedded-hal explicitly narrowed its goal to traits needed by generic drivers after finding that full HAL standardization became too complex because real hardware exposes wildly different capabilities [[R01]](#ref-r01). It also recommends proving a proposed interface using at least two platform implementations and one generic consumer [[R02]](#ref-r02). Those two rules are adopted as project policy here.

The project deliberately borrows different ideas from different ecosystems:

- **embedded-hal:** semantic, device-driver-focused contracts; transaction-oriented I2C; SPI bus/device separation; separate execution models; generic error categories [[R01]](#ref-r01) [[R03]](#ref-r03) [[R04]](#ref-r04).
- **modm:** generated target metadata, compile-time validation, static allocation, and target-specific optimized code [[R06]](#ref-r06).
- **Zephyr:** strict device/subsystem separation, build-time immutable device facts, board description, and explicit vendor-specific extensions — but *not* its runtime function-table dispatch [[R08]](#ref-r08) [[R09]](#ref-r09).
- **CMSIS-Driver:** disciplined peripheral specifications, validation suites, common capability/error vocabulary, and explicit acknowledgement that vendor-specific extensions remain necessary — but *not* its runtime Access Struct dispatch [[R10]](#ref-r10) [[R11]](#ref-r11).
- **TinyUSB:** isolate MCU-specific low-level driver code and OS integration from the portable core; use static memory and defer complex ISR work [[R14]](#ref-r14).
- **libhal:** strict interface/platform/device separation and composable adapters — but *not* its virtual-interface/reference-counted resource model or mandatory async architecture for this project [[R07]](#ref-r07).

The result is intentionally **not a universal model of every MCU register feature**. It is a portable contract layer plus target extensions.
## 2. Project goals, non-goals, and invariants

### 2.1 Goals

1. A portable device driver should be compilable unchanged on STM32H723, ESP32-S3, Linux integration backends, and the deterministic `sim` backend when the required capability exists.
2. Incompatible hardware capabilities should normally fail at compile time through concepts or target metadata, rather than through a runtime `supports_x()` branch.
3. The target family package, exact device/module profile, and BSP are selected at configure/build time; no target-selection branch exists in normal runtime code.
4. All normal HAL object storage is static, automatic, or explicitly caller-owned.
5. Peripheral use is deterministic: no hidden `new`, `malloc`, `std::vector` growth, reference counting, plugin loading, or virtual dispatch.
6. Public headers contain no vendor SDK types.
7. Target-specific “superpowers” remain accessible without polluting portable contracts.
8. Every portable contract has a written semantic specification and a conformance-test suite.
9. `sim` is a first-class deterministic test backend for unit tests, fault injection, and whole-application simulation; Linux is a distinct integration backend for real kernel/device APIs.
10. The architecture should scale from a few handwritten implementations to larger generated target support.
11. The FreeRTOS ECU template is the superproject that pins driver/target package versions; FreeRTOS remains above the HAL contracts and target driver object model.
12. Normal development uses CMake incremental builds; all disposable build output is contained under `.build/`, and embedded release builds emit `.elf`, `.bin`, and `.hex`.

### 2.2 Explicit non-goals

- Binary portability between unrelated MCUs.
- A single runtime object that can dynamically switch from STM32 to ESP32 behavior.
- Abstracting clock trees, DMA engines, interrupt controllers, and advanced timer blocks into one giant “universal MCU” interface.
- Guaranteeing that every target has every portable capability.
- Hiding important hardware semantics such as flash erase geometry, CAN-FD payload size, or I2C repeated starts.
- Introducing a generic host abstraction without a second concrete host operating system that needs it. Linux-specific kernel UAPIs and deterministic simulation are separate concerns.

### 2.3 Hard architectural invariants

These should eventually be enforced by build tests and code review:

```text
PORTABLE CONTRACT LAYER
  - no vendor headers
  - no target macros
  - no virtual
  - no RTTI dependency
  - no exceptions dependency
  - no global heap dependency
  - no RTOS dependency

PORTABLE DEVICE DRIVER LAYER
  - depends only on portable contracts + portable utilities
  - no target headers
  - no direct GPIO/pinmux/register access unless GPIO itself is the required contract

TARGET BACKEND LAYER
  - may include vendor/CMSIS/ESP-IDF low-level headers
  - may use target interrupts, DMA, cache maintenance, errata workarounds
  - must satisfy the public semantic contract exactly

BOARD/BSP LAYER
  - selects the exact MCU/module profile for the physical PCB
  - owns pins, alternate functions, external clock-source facts and physical wiring
  - does not contain portable product/application behavior

FIRMWARE COMPOSITION/POLICY
  - chooses operating points, whole-system resource assignments and driver composition
  - may use FreeRTOS queues/tasks to organize services without making drivers RTOS-aware
```
## 3. Analysis of the reviewed approaches

This section is intentionally comparative. A professional architecture is easier to justify when the rejected alternatives are understood rather than merely ignored.

### 3.1 Rust embedded-hal 1.0

**What it is.** embedded-hal defines traits intended primarily for platform-agnostic external-device drivers. Its 1.0 migration document states that the earlier dual goal of also standardizing complete MCU HAL APIs was dropped because hardware capabilities vary too much and capability fragmentation/customization points made generic drivers harder to write [[R01]](#ref-r01).

**Most important lessons for this project.**

- Define contracts around what portable *consumers* need, not around every peripheral mode an MCU exposes.
- Avoid excessive trait/concept fragmentation. Earlier fine-grained I2C/SPI traits caused interoperability problems; 1.0 unified them [[R01]](#ref-r01).
- I2C should have a transaction semantic that captures START/repeated-START/STOP behavior rather than vendor booleans such as `nostop` [[R03]](#ref-r03).
- SPI should distinguish a shared physical bus from a logical device transaction that owns/controls CS and bus exclusivity [[R04]](#ref-r04).
- Execution models should be separate. Blocking, async, and nonblocking contracts do not have to be forced into one API [[R01]](#ref-r01).
- A new portable contract should be demonstrated on multiple unrelated platforms and by a generic driver before standardization [[R02]](#ref-r02).

**Pros relative to this project.** Extremely close philosophical fit; static generic dispatch maps naturally to C++20 concepts; contracts are deliberately minimal and device-driver-centric; Linux is explicitly considered a useful second implementation target.

**Cons / differences.** Rust's type system, lifetimes, error traits, and ecosystem conventions cannot be copied mechanically into C++. Some embedded-hal abstractions were removed rather than standardized (including earlier ADC/timer/PWM/watchdog concepts), which means this project must design those cautiously rather than assuming a mature universal answer exists [[R01]](#ref-r01).

**Decision:** use embedded-hal as the primary semantic-design reference, not as a literal API translation.

### 3.2 modm

**What it is.** modm combines hardware descriptions (`modm-devices`), generated target-specific code, and the `lbuild` generator. It states that its HAL does not dynamically allocate; storage is static or explicitly supplied by the user [[R06]](#ref-r06).

**Pros relative to this project.**

- Excellent match for no-heap/static-target requirements.
- Code generation scales pin mappings and MCU variation much better than hand-maintained `#ifdef` trees.
- Compile-time hardware validation can reject impossible pin/peripheral combinations before firmware runs.
- Static functions/templates can inline down to register accesses.

**Cons / trade-offs.**

- A generator and hardware-description database become a product of their own: schema versioning, vendor-data ingestion, patches, tests, and reproducible generation all need maintenance.
- An all-static-class API can make dependency injection and stateful simulation less natural than ordinary concrete objects.
- Heavy template/generator use increases build-system complexity and can produce template code duplication.

**Decision:** copy the *generated target-facts + static validation* architecture, but use ordinary stateful concrete C++ objects at the runtime-object level where that improves ownership and testability.

### 3.3 libhal v5

**What it is.** libhal is a modern C++ HAL ecosystem built around interface types, managers/resources/adapters, user-controlled PMR allocation, and C++20 coroutine-based async operations [[R07]](#ref-r07).

**Pros relative to this project.**

- Very clean separation between platform implementation and portable consumers.
- Adapters are treated as first-class compositional elements.
- Allocation is explicit rather than silently using the global heap.
- Runtime-polymorphic interfaces give stable non-template device-driver types and excellent binary/module boundaries.

**Cons relative to the project's hard constraints.**

- Its interface mechanism is runtime polymorphism/pure virtual interfaces, which directly violates the no-vtable requirement.
- Reference-counted resource ownership adds state and verification surface that is unnecessary when the hardware object graph is fixed for the lifetime of firmware.
- Async-first coroutines and allocator-managed coroutine frames solve a different problem than the desired minimal synchronous/static V1.

**Decision:** borrow layering, adapters, and explicit ownership thinking; reject virtual interfaces, reference-counted HAL ownership, and mandatory async for the core profile.

### 3.4 Zephyr

**What it is.** Zephyr defines generic subsystem APIs and a device model. Its `device` structure separates immutable config, runtime data, and an API pointer; generic driver calls dispatch through function-pointer tables [[R08]](#ref-r08). Zephyr also permits device-specific APIs when generic subsystem APIs cannot represent special hardware features [[R08]](#ref-r08).

**Pros.**

- Proven under enormous board/MCU diversity.
- Excellent separation of build-time hardware description (Devicetree) from runtime driver state [[R09]](#ref-r09).
- Device-specific extensions provide an explicit escape hatch instead of corrupting the generic API.
- Very useful reference for what a mature subsystem contract must document.

**Cons for this project.**

- Runtime function tables are manual vtables and violate the static-dispatch goal.
- Many Zephyr APIs include OS/kernel expectations and runtime `-ENOTSUP` patterns that are unnecessary when the target is statically known.
- Devicetree is powerful but introduces another language, generator pipeline, and ecosystem that may be excessive for V1.

**Decision:** borrow the immutable-config/runtime-state split and hardware-description layering; use concepts and generated C++ instead of runtime API tables. Keep Devicetree as future interoperability input, not a core dependency.

### 3.5 CMSIS-Driver

**What it is.** CMSIS-Driver defines generic interfaces for common embedded peripherals and makes them available through per-instance Access Structs containing function pointers. It includes `GetCapabilities`, common start/stop conventions, templates, and a Driver Validation Suite [[R10]](#ref-r10) [[R11]](#ref-r11).

**Pros.**

- Strongly documented contracts and naming consistency.
- Explicit capability reporting demonstrates how mature systems deal with optional features.
- Multiple instances and middleware-facing interfaces are clearly separated.
- Validation-suite thinking is directly applicable.
- Device Family Packs may provide additional device-specific interfaces when the generic API is insufficient [[R10]](#ref-r10).

**Cons for this project.**

- The Access Struct is runtime indirect dispatch.
- Capability bits often move a compile-time fact into runtime because CMSIS has different binary-interoperability goals.
- Callback-centric asynchronous APIs couple events to function-pointer registration.

**Decision:** copy documentation discipline, generic error/capability vocabulary, and validation-suite practice. Replace runtime capability bits with concepts/static traits when the capability is build-known.

### 3.6 RIOT

RIOT's SPI API explicitly treats the SPI peripheral as a shared bus resource, with acquisition/configuration and release around transactions [[R13]](#ref-r13). This supports the same conclusion as embedded-hal's `SpiBus`/`SpiDevice`: external-device drivers should not casually reconfigure a shared bus or manually coordinate CS with unrelated drivers.

**Decision:** adopt transaction ownership at the `SpiDevice` layer; keep locking mechanism outside the low-level portable bus contract so bare-metal and RTOS builds can use different static adapters.

### 3.7 TinyUSB

TinyUSB isolates MCU-specific Device/Host Controller Drivers from a portable USB core and OS abstraction. Current architecture docs emphasize compile-time/static memory and deferred interrupt processing [[R14]](#ref-r14).

**Why it matters even though USB is not in V1.** It proves that highly diverse controller IP can be abstracted successfully when the abstraction represents protocol/controller semantics rather than vendor registers. It also validates the pattern `ISR -> bounded event state/queue -> non-ISR processing` for complex portable code.

**Decision:** use the same layering principle. Interrupt handlers stay target-specific and minimal; portable layers should not require arbitrary callbacks from ISR context.

### 3.8 C++20 concepts, customization mechanisms, and freestanding concerns

C++20 concepts provide compile-time requirements and are intended to represent semantic categories, not merely syntactic detection [[R15]](#ref-r15). That exactly matches the requirement that `I2cController<T>` means “T obeys this I2C contract,” not merely “T has a method named `transaction`.”

WG21 customization-point work catalogs alternatives such as ADL, CPOs, tag dispatch, `tag_invoke`, inheritance, and member customization [[R19]](#ref-r19). For this HAL, named member functions constrained by concepts are deliberately preferred because they are explicit, readable, debugger-friendly, and require less metaprogramming machinery.

Freestanding standard-library support has historically varied across C++ implementations [[R16]](#ref-r16). `std::span` is conceptually ideal [[R17]](#ref-r17), but the project should either establish a tested toolchain baseline or expose a tiny `hal::span` compatibility type. `std::expected` is C++23, not C++20 [[R18]](#ref-r18), so the baseline needs its own allocation-free `hal::result` or a project-controlled equivalent.

### 3.9 Safety/determinism implications

The no-heap/no-vtable policy is stricter than general embedded C++ requires, but it has advantages for a deterministic target: fixed storage, fixed call graph, static resource identity, easier whole-program inspection, and fewer lifetime modes. AUTOSAR's safety-oriented guidance highlights the problems dynamic allocation can introduce, including fragmentation and boundedness concerns [[R22]](#ref-r22).

This does **not** make the library automatically safety-certified. Functional safety also requires requirements traceability, hazard analysis, tests, tool qualification as applicable, coding rules, and verification. The architecture simply avoids several sources of nondeterminism and hidden control flow.
## 4. Professional architecture options and final selection

| Approach | Dispatch | Hidden heap required | Compile-time capability rejection | Device-driver type stability | MCU-scale maintainability | Fit |
|---|---|---:|---:|---:|---:|---|
| C++20 concepts + concrete objects | Static | No | Excellent | Template-specialized | Good | **Selected core** |
| Generated static HAL (modm-like) | Static | No | Excellent | Template/static | Excellent | **Selected target strategy** |
| CRTP | Static | No | Good | Template-specialized | Good | Use only for implementation reuse |
| CPO/ADL/tag-invoke | Static | No | Excellent | Template-specialized | Good | Defer; unnecessary V1 complexity |
| Build-selected global facade | Static | No | Moderate | Single build | Moderate | Board glue only |
| CMSIS/Zephyr function tables | Runtime indirect | No | Mostly runtime | Stable | Excellent | Rejected by no-vtable policy |
| libhal virtual resources | Runtime virtual | Controlled allocator | Mixed | Excellent | Good | Rejected by no-vtable policy |

### Why the selected combination wins

It gives the compiler the concrete target type everywhere that hardware-specific behavior matters, while preserving a single source-level API for portable consumers. It also avoids the scalability failure of a purely handwritten HAL by generating repetitive hardware facts. The critical distinction is:

```text
Generate facts, not semantics.
```

Generated data may say that `PA9` can route `USART1_TX` with alternate function X, or that `FDCAN1` uses a particular base address and IRQ. The hand-designed portable contract decides what `serial::Port` or `can::FdController` *means*.
## 5. Target hierarchy: family package, exact device/module, board, firmware

The target model is deliberately hierarchical. A reusable ECU firmware ecosystem must not collapse MCU family implementation, exact silicon/package facts, PCB wiring, and firmware policy into one "target configuration".

The architectural rule is:

```text
Target package:       what CAN this MCU family implementation support?
Exact device/module:  what physically EXISTS on this exact part/module?
Board/BSP:            what DID this PCB wire and populate?
Firmware composition: how WILL this firmware use those resources?
Application logic:    what should the PRODUCT do?
```

These layers may all participate in compile-time validation, but they have different owners and lifecycles.

### 5.1 STM32 hierarchy: family-level package, exact-device profile

For STM32, the repository/package boundary is the **STM32H7 family**, not one repository per exact H723 part:

```text
hal-target-stm32h7
        |
        +-- common STM32H7/Cortex-M7 implementation
        +-- reusable peripheral-IP implementations
        +-- exact-device profiles
                |
                +-- STM32H723VGT6
                +-- future H7 devices
```

The exact physical device currently selected is `STM32H723VGT6` (LQFP100) [[R23]](#ref-r23). STM32H7 devices are not all identical; ST itself separates H7 subfamilies across different reference manuals, and the target package must never assume one peripheral layout merely because two devices share the H7 marketing family [[R24]](#ref-r24). The reason for the family-level package is **reuse**, not homogenization.

The preferred internal organization is therefore:

```text
hal-target-stm32h7/
  include/
  src/                         # only if non-template target code exists
  peripherals/
    <shared peripheral-IP implementations>
  include/hal/stm32h7/device/
    stm32h723vgt6/
      device.hpp
      pins.hpp
      alternate_functions.hpp
      interrupts.hpp
      dma.hpp
      memory.hpp
      capabilities.hpp
```

The exact-device profile contains immutable facts such as:

- which peripheral instances exist;
- package-visible pins;
- legal alternate-function routes;
- IRQ mappings;
- DMA request mappings;
- memory regions and relevant DMA/cache accessibility facts;
- peripheral-IP revisions/capability traits where needed.

The backend may internally select reusable implementations based on those facts. For example, a future H743 profile and the H723 profile may share some implementation code while selecting different peripheral-IP traits where the hardware actually differs.

This structure mirrors the useful part of ST's own series-level component organization: STM32CubeH7 separates H7-series HAL/CMSIS components from BSP components rather than creating an independent HAL repository for every exact part [[R47]](#ref-r47). Our architecture goes further by making exact-device facts and board wiring separate compile-time layers.

### 5.2 ESP32 hierarchy: SoC target package, exact module profile

For ESP32-S3, the reusable target package is:

```text
hal-target-esp32s3
```

The selected physical module is:

```text
ESP32-S3-WROOM-1-N16R8
```

The module profile is more specific than the SoC because it fixes external flash/PSRAM population and constrains the module-visible pin set. The selected N16R8 module contains 16 MB flash and 8 MB PSRAM [[R28]](#ref-r28).

Recommended structure:

```text
hal-target-esp32s3/
  include/
  src/
  metadata/
  generated/
    soc/
      esp32s3/
    modules/
      esp32s3_wroom_1_n16r8/
```

The board selects the exact module profile. The target package owns the facts that are universally true for that SoC/module combination; the PCB/BSP owns the external wiring.

### 5.3 Linux and deterministic simulation

These remain two deliberately different implementation roles:

```text
hal-target-linux
        -> real Linux integration through SocketCAN, TAP/AF_PACKET,
           gpio chardev, spidev, i2c-dev, IIO, tty, RTC, watchdog, files

hal-sim
        -> deterministic software models controlled entirely by tests
```

Linux is a real host/integration backend. `sim` is the default backend for deterministic unit and application simulation. There is no generic POSIX target layer.

### 5.4 Board/BSP owns exact target selection and physical wiring

A PCB is physically built around an exact MCU/package or module, so the **board profile selects the exact device/module**.

For example, a future Sensor Management Unit BSP may declare conceptually:

```yaml
board: smu_rev_a

target:
  package: stm32h7
  device: stm32h723vgt6

clock_sources:
  hse_hz: 25000000
  lse_hz: 32768

sensor_i2c:
  peripheral: i2c2
  scl: PB10
  sda: PB11

vehicle_can:
  peripheral: fdcan1
  rx: PD0
  tx: PD1
  transceiver_enable: PE4
```

The BSP does **not** duplicate the STM32H723VGT6 pin database. Instead, its choices are validated against the exact-device facts owned by `hal-target-stm32h7`.

This separates statements such as:

```text
Target fact:   PB10 CAN be routed to a specific peripheral signal.
Board fact:    this PCB DOES connect PB10 to that function.
Firmware rule: this application WILL configure/use that resource at a chosen rate.
```

### 5.5 Firmware policy is not board hardware

Some configuration spans multiple layers and must remain intentionally split.

Examples:

| Concern | Target package owns | BSP owns | Firmware/composition owns |
|---|---|---|---|
| clocks | legal PLL/prescaler topology | physical HSE/LSE sources | selected operating frequency |
| DMA | legal requests/routes, memory constraints | physical external-memory facts if any | selected channel/resource allocation |
| FDCAN | instances, pins, clock capabilities | transceiver wiring/pins | bit rate, filters, queues/tasks |
| I2C | controller capability/pin routes | bus pins/pull-up topology | bus frequency/device composition |
| memory | on-chip regions/cache/DMA facts | external memory population | stack/buffer placement policy |

Do not create one enormous `stm32h723_config.hpp` that mixes all three categories.

## 6. Repository, package, BSP, and dependency architecture

The FreeRTOS ECU firmware template is the **superproject/composition root**. It owns the dependency versions and selects one BSP/application composition for a concrete firmware build.

This document is the authoritative architecture baseline for the new driver/template ecosystem. Older repository documents, CMake helpers, tests, or target lists that still assume POSIX as a target, nested backends, or legacy target sets are **migration artifacts**, not competing specifications. Milestone 0 is not complete until those files are removed, renamed, or reconciled with this model.

### 6.1 Recommended superproject layout

```text
ecu-firmware-template/
|
+-- application/
|   +-- services/
|   |   +-- can/
|   |   |   +-- can_service.hpp
|   |   |   +-- can_service.cpp
|   |   +-- sensing/
|   |   |   +-- sensors.hpp
|   |   |   +-- sensors.cpp
|   |   +-- logging/
|   |       +-- log_storage.hpp
|   |       +-- log_storage.cpp
|   |
|   +-- tasks/
|       +-- can/
|       |   +-- can_task.hpp
|       |   +-- can_task.cpp
|       |
|       +-- sensing/
|       |   +-- sensing_task.hpp
|       |   +-- sensing_task.cpp
|       |
|       +-- logging/
|           +-- logging_task.hpp
|           +-- logging_task.cpp
|
+-- composition/
|   +-- system.hpp
|   +-- system.cpp
|
+-- bsp/
|   +-- <board-name>/
|       +-- board.yaml
|       +-- board.hpp
|       +-- board.cpp
|
+-- config/
|   +-- firmware.hpp
|   +-- freertos/
|       +-- FreeRTOSConfig.h
|       +-- hooks.cpp
|
+-- lib/
|   +-- ecu-drivers/              [single git submodule/package]
|   +-- freertos/                 [git submodule/package]
|
+-- tests/
+-- cmake/
|   +-- toolchains/
|   +-- board.cmake
|   +-- target.cmake
|
+-- CMakePresets.json
+-- CMakeLists.txt
+-- main.cpp
+-- .build/                       [ignored build tree]
```

The driver layers are logical CMake packages inside one repository rather than separate HAL
submodules:

```text
ecu-drivers/
  include/                    portable contracts and core utilities
  targets/
    stm32f4/                  family implementation + exact profiles
    stm32h5/                  family implementation + exact profiles
    stm32h7/                  family implementation + exact profiles
    esp32s3/                  SoC implementation + module profiles
    linux/                    Linux userspace integration
  devices/                    portable external-device/protocol drivers
  sim/                        deterministic backend and device models
  third_party/                CMSIS and ESP-IDF target dependencies
```

The firmware superproject pins one compatible `ecu-drivers` revision and one FreeRTOS revision.
There are no separate `hal-core`, `hal-target-*`, `hal-sim`, or `hal-devices` submodules in the
firmware template. The `ecu-drivers` CMake package still exports separate logical targets such as
`hal::core`, `hal::stm32h723vgt6`, `hal::linux`, `hal::devices`, and `hal::sim`, so repository
consolidation does not collapse architectural boundaries or force unrelated targets into a build.

The exact MCU/module profiles remain inside their corresponding target package. The `devices/`
layer is reserved for reusable external-device/protocol drivers and never depends on an MCU target.
The BSP and composition remain in the firmware repository and select the concrete target/profile and
physical wiring.

### 6.2 Package responsibilities

#### `hal-core`

Owns portable infrastructure, contracts, and small capability-specific composition utilities:

```text
core/
one header per capability contract
```

It knows nothing about STM32, ESP32, Linux, FreeRTOS, or a particular board.

#### Target packages

`ecu-drivers/targets/` owns the MCU/OS implementations and exact-device/module profiles:

- `hal-target-stm32f4` — STM32F4 implementation and `STM32F446VET6` profile;
- `hal-target-stm32h5` — STM32H5 implementation and `STM32H563VIT6` profile;
- `hal-target-stm32h7` — STM32H7 implementation and `STM32H723VGT6` profile;
- `hal-target-esp32s3` — ESP32-S3 implementation and `ESP32-S3-WROOM-1-N16R8` profile;
- `hal-target-linux` — real Linux userspace integration.

#### `hal-sim`

Owns deterministic semantic models and fault-injection backends. It does not depend on Linux hardware UAPIs simply because tests execute on Linux.

#### `hal-devices`

Owns portable external-device/protocol drivers. It depends on `hal-core` contracts, never on an MCU target.

### 6.3 BSP belongs above the target packages

The BSP is not part of the reusable driver library. It belongs to the concrete firmware/product repository unless multiple independent firmware images need the same physical board support, in which case it may later be extracted into a reusable company BSP package.

The BSP owns:

- exact target device/module selection;
- PCB pin assignments;
- oscillator/external clock source facts;
- external transceivers/PHY/reset/power-enable wiring;
- chip-select lines and external peripheral attachment;
- board-specific electrical/topology facts.

Application logic must still not directly contain those pin numbers. Both `application/` and `bsp/` may live in the same firmware repository, but they are separate architectural layers.

### 6.4 Composition is the only place that should see both sides

The composition root binds concrete board resources to portable device drivers and application services. It is also the normal boundary where application task objects are constructed and supplied with their dependencies. Application task headers must not include STM32/ESP32/Linux target headers; target-specific types are confined to BSP/composition code.

For the ECU template, `application/` is intentionally organized **by feature/function**, not by `include/` and `src/` file type. Reusable library packages may still use `include/` to mark exported headers. Application code is split into two closely related roles:

```text
application/services/can/can_service.hpp + can_service.cpp
application/services/sensing/sensors.hpp + sensors.cpp
application/services/logging/log_storage.hpp + log_storage.cpp

application/tasks/can/can_task.hpp + can_task.cpp
application/tasks/sensing/sensing_task.hpp + sensing_task.cpp
application/tasks/logging/logging_task.hpp + logging_task.cpp
```

`services/` contains product-level semantic components that coordinate one or more portable device/HAL dependencies. `tasks/` contains the FreeRTOS execution/scheduling boundary. A task should normally depend on the **smallest meaningful semantic service** it needs rather than directly accumulating low-level devices. Direct device injection remains acceptable for genuinely small/simple tasks, but it is not the default once multiple related devices or product behavior are involved.

Recommended direction:

```text
BME280 + IMU + ADC channels -> Sensors   -> SensingTask
CAN controller             -> CanService -> CanTask
block/NV/filesystem        -> LogStorage -> LoggingTask
```

This keeps device/protocol knowledge out of task code, keeps task constructors small, and preserves explicit dependency injection/testability without falling back to hidden globals. A task may receive more than one dependency when they are genuinely independent semantic collaborators, but repeated growth of a task constructor is a signal to introduce or refine a service/component.

The composition root binds concrete board resources and portable capability utilities to device
drivers, application services, and tasks:

```text
hal-core <--- hal-target-*
    ^
    +------- hal-devices

hal-target-* + hal-devices + BSP
                         |
                         v
                    composition
                         |
                         v
                    application
```

Only BSP/composition code should routinely mention exact target types. Portable application logic receives semantic services/resources.

### 6.5 FreeRTOS is application/runtime policy, not a HAL dependency

This ECU template uses FreeRTOS, but **the drivers themselves do not depend on FreeRTOS**.

Do not put `QueueHandle_t`, `SemaphoreHandle_t`, task handles, FreeRTOS headers, or scheduler assumptions into `hal-core` contracts or the MCU target drivers. This keeps the same HAL usable in:

- bootloaders;
- factory-test images;
- single-threaded bring-up firmware;
- host simulation;
- Linux integration tools;
- the FreeRTOS ECU application.

FreeRTOS queues may be used **above** the driver boundary to transfer data/events between tasks and driver-owning logic. Likewise, task ownership or a queue-based service can serialize access to a peripheral without making the underlying driver itself RTOS-aware.

Example architecture:

```text
FreeRTOS tasks
     |
     +-- semantic dependencies --> Sensors / CanService / LogStorage
                                     |
                                     v
                         hal-devices
                                     |
                                     v
                                target HAL

queues/messages remain application/runtime mechanisms between tasks/services where needed
```

If the firmware uses FreeRTOS static-object APIs, the application can supply queue/task/timer storage explicitly; FreeRTOS documents static allocation support through `configSUPPORT_STATIC_ALLOCATION` and APIs such as `xQueueCreateStatic()` [[R57]](#ref-r57). This is an application/runtime decision and does not change the HAL contracts.

#### ISR-to-application bridge

The target driver owns the **hardware interrupt service**; BSP/composition owns the **OS/application notification**. A target ISR handler may update bounded, statically allocated driver state and return/latch an event indication. BSP/composition interrupt glue may then use `...FromISR()` FreeRTOS primitives to wake the owning task. `hal-core` and `hal-target-*` do not include FreeRTOS headers merely to notify application code.

Recommended flow:

```text
MCU interrupt
   -> target driver's bounded handle_irq()/ISR service
   -> static driver event/RX state
   -> BSP/composition ISR glue
   -> FreeRTOS task notification/queue if needed
   -> owning application task
```

The default ECU sharing policy is **single-task ownership of stateful peripherals**. Prefer one CAN task owning the CAN controller, one storage task owning writable block storage, etc., with queues/task notifications used between application functions. Internal driver mutexes are not the default. A shared synchronous bus may use an explicitly composed locking/transaction adapter only when real ownership cannot be made single-task.

### 6.6 CMake is the build/composition system

CMake owns target/BSP selection, toolchain selection, dependency wiring, code generation steps, optimized firmware builds, tests, and final artifact conversion.

Use checked-in `CMakePresets.json` for reproducible configure/build workflows. CMake Presets are specifically intended to share common project/CI configuration, and workflow presets can sequence configure/build/test steps [[R55]](#ref-r55).

Recommended presets include at least:

```text
<board>-release       optimized production firmware
<board>-debug         bring-up/debug build when needed
linux-release         Linux integration tools/tests
sim-debug             deterministic host tests with sanitizers where supported
```

Production firmware release presets enable the selected compiler's release optimizations (for example `-O2`/`-O3` or an explicitly selected size/performance profile), section garbage collection, and LTO only after it is validated for the target/toolchain. Optimization settings belong in version-controlled CMake/toolchain configuration, not in developer-local IDE state.

### 6.7 `.build/` is the complete disposable build tree

All CMake-generated build-system files and compiler/linker outputs must stay out of the source tree under `.build/`.

Recommended organization:

```text
.build/
  <preset-or-board>/
    CMakeCache.txt
    CMakeFiles/
    compile_commands.json
    generated/
      bsp/
      build_config/
    obj/
    lib/
    artifacts/
      ecu_firmware.elf
      ecu_firmware.bin
      ecu_firmware.hex
      ecu_firmware.map          # recommended diagnostic artifact
```

`.build/` is ignored by version control and can be deleted at any time. Reproducible board-derived headers should be generated during CMake configure/build into `.build/.../generated/` rather than written into the source BSP directory.

The STM32H7 exact-device headers are part of the target package's versioned source model under
`include/hal/stm32h7/device/<part-number>/`. Board-derived headers may still be generated into
`.build/.../generated/` because they are build outputs rather than device facts.

### 6.8 Incremental builds are the normal workflow

Preserve the configured binary tree and invoke:

```text
cmake --build <build-directory>
```

or the corresponding build preset. CMake's build mode operates on an existing generated binary tree [[R56]](#ref-r56); the underlying build tool (Ninja is recommended for this workflow) recompiles/relinks only what its dependency graph marks stale.

Do **not** run a clean/fresh configure before every normal build. Fresh builds remain useful for CI reproducibility checks, but day-to-day firmware development should be incremental.

The embedded release build must produce at least:

```text
.elf    primary linked/debug-symbol artifact
.bin    raw programming image where required
.hex    Intel HEX programming image
```

Create `.bin`/`.hex` from the linked `.elf` using the selected toolchain's `objcopy` as explicit CMake post-build/custom commands, with all outputs placed under `.build/.../artifacts/`.

### 6.9 Dependency invariants

The following are hard architectural rules:

```text
hal-core MUST NOT depend on any target, BSP, FreeRTOS, or vendor SDK.
hal-devices MUST NOT depend on target packages or FreeRTOS.
hal-target-* MAY depend on hal-core and private vendor definitions.
hal-sim depends on hal-core, not on an embedded target.
BSP depends on exactly the target package(s) needed by that physical board.
composition may depend on BSP + hal-devices + application services.
application logic MUST NOT know MCU pins/registers/vendor types.
```

A CI rule should compile portable headers/tests with STM32Cube and ESP-IDF absent from the include path. If a vendor type leaks upward, the layering has been violated.

## 7. Two-stage generated hardware/configuration model

Generation is retained, but the old single `generated/` concept is split into **target facts** and **board/build selections**.

### 7.1 Stage 1: target-package generation

modm demonstrates the scalability benefit of combining machine-readable hardware descriptions with generated target C++ [[R06]](#ref-r06). CMSIS-SVD provides machine-readable register/peripheral structure [[R21]](#ref-r21).

Target-package generation consumes only information that is universally true for the exact device/module:

```text
Vendor SVD / SDK metadata
        +
project-maintained corrections/overlays
        +
exact package/module metadata
        v
target generator
        v
hal-target-*/generated/<exact-device-or-module>/
```

For STM32H723VGT6, generated facts may look conceptually like:

```cpp
struct stm32h723vgt6_facts {
    static constexpr auto gpio_ports = /* ... */;
    static constexpr auto spi_instances = /* ... */;
    static constexpr auto fdcan_instances = /* ... */;
    static constexpr auto pin_routes = /* ... */;
    static constexpr auto dma_requests = /* ... */;
    static constexpr auto irq_map = /* ... */;
};
```

This output describes **what the exact target can do**. It does not contain the Sensor Management Unit's chosen pins or application rates.

### 7.2 Stage 2: BSP/build-time generation and validation

The firmware BSP provides only board-specific facts/selections. During CMake configure/build, the board generator/validator combines those selections with the exact target facts:

```text
hal-target-stm32h7 generated STM32H723VGT6 facts
        +
BSP board.yaml
        +
firmware-selected operating configuration
        v
validation / clock-resource solving
        v
.build/<preset>/generated/bsp/
```

If the BSP requests an illegal route such as an unsupported alternate function, configuration/generation fails before the firmware is compiled.

This stage may generate:

```text
pins.hpp
clock_config.hpp
peripheral_config.hpp
resource_assignments.hpp
memory_policy.hpp
```

These build-derived files are disposable and belong under `.build/`, not in `ecu-drivers` and not in the target package's immutable device database.

### 7.3 Generate facts and selections, never portable semantics

Neither generator decides what `I2cController::transaction`, `SpiDevice::transaction`, or `can::FdController` means. Portable semantics remain manually specified and tested in `ecu-drivers`.

Generation is used to make invalid physical/resource choices unrepresentable or to fail configuration/compilation early.

### 7.4 Board selection replaces a global HAL board namespace

The generic HAL does not own `HAL_BOARD` and does not expose a global `hal::selected::board`. The firmware superproject chooses a BSP through CMake/presets, and BSP/composition code selects the exact target profile.

Portable drivers receive concrete dependencies through constructors/template parameters and never query a global selected target.

### 7.5 BSP schema v0 and generation contract

Implementation begins with a deliberately small, project-owned BSP schema rather than a universal board-description language. The schema is versioned and validated. At minimum, BSP schema v0 must be able to describe:

- board identity/revision;
- target family plus exact device/module selection;
- physical oscillator sources and frequencies;
- named peripheral-instance selections;
- pin/signal routing;
- chip-select, reset, standby, enable and interrupt wiring for external devices;
- board-level external memory/transceiver/PHY facts required by startup.

Conceptual example:

```yaml
schema_version: 0
board: smu_rev_a
target:
  family: stm32h7
  device: stm32h723vgt6
oscillators:
  hse_hz: 25000000
  lse_hz: 32768
peripherals:
  vehicle_can:
    instance: fdcan1
    rx: PD0
    tx: PD1
    transceiver_standby: PE4
  sensor_i2c:
    instance: i2c2
    scl: PB10
    sda: PB11
```

The board generator validates this input against the exact-device target facts and emits only build-derived configuration under `.build/<preset>/generated/bsp/`. Generated source-tree MCU facts and generated build-tree board selections remain distinct artifacts with different owners. The schema should grow only when real boards require new facts.

## 8. Core C++ design rules

### 8.1 Concepts are contracts, not implementation inheritance

```cpp
template<class T>
concept OutputPin = requires(T& pin, gpio::level value) {
    typename T::error_type;
    { pin.write(value) }
        -> std::same_as<hal::result<void, typename T::error_type>>;
};
```

A target type satisfies the requirement structurally:

```cpp
class Stm32OutputPin { /* write(...) */ };
class Esp32OutputPin  { /* write(...) */ };
class SimOutputPin    { /* write(...) */ };

static_assert(hal::gpio::OutputPin<Stm32OutputPin>);
static_assert(hal::gpio::OutputPin<Esp32OutputPin>);
static_assert(hal::gpio::OutputPin<SimOutputPin>);
```

No common base class exists, and no vtable is emitted solely to implement the contract.

### 8.2 Normal objects, not only static classes

The target is static, but objects may still carry compile-time-sized state:

```cpp
Stm32I2c i2c1{ /* register instance/state */ };
Bme280 sensor{i2c1};
```

This gives dependency injection and multiple instances without runtime polymorphism.

### 8.3 Static allocation rule

- object members, globals/statics, stack/automatic storage, or caller-provided spans are allowed;
- no hidden heap allocation inside contracts or normal target operations;
- variable-size queues/buffers must have compile-time capacity or caller-supplied storage;
- no `std::function` in the core; use template callables or explicit bounded event state;
- no `shared_ptr`/reference counting;
- no driver-created threads/tasks.

### 8.4 Error model

Every driver family has:

1. a portable `error_kind` enum on which generic code may branch;
2. a backend-defined concrete `error_type` that maps to the portable kind and may retain native status/register information.

This mirrors embedded-hal's generic-error-kind philosophy while preserving target diagnostics [[R01]](#ref-r01).

### 8.5 No generic `not_supported` for core functionality

If a type claims to model a core concept, its required methods must work according to the contract. Optional capabilities are represented by **refinement concepts**. A backend may return a configuration error when a *parameter value* cannot be represented; it should not claim an interface and then routinely return `not_supported` for fundamental operations.

### 8.6 Configuration represents intent, not registers

Use names such as `max_frequency` when the caller means “do not exceed this device limit”. A backend then chooses the highest achievable safe rate. This avoids the semantic ambiguity of `set_frequency(10MHz)` when different clock-divider hardware rounds differently.

### 8.7 Concurrency baseline

Core V1 resources are single-owner and not implicitly thread-safe. If a peripheral is shared, use a
statically composed `hal::spi::StaticDevice8<Bus, ChipSelect, Delay, Lock>` or a board-owned device
transaction object. The contract documents whether methods may be called from ISR context; default
assumption is **task/main context only unless explicitly stated**.

### 8.8 Interrupt policy

Portable V1 interfaces avoid arbitrary callback registration because callbacks introduce lifetime/context questions. Target ISRs update statically allocated event state; portable refinements expose polling/latches or explicit bounded queues. Complex future async APIs may define a separate event/executor model.

### 8.9 Async policy

V1 is synchronous/polling-oriented at the portable contract boundary. An async namespace can be added later without changing the sync contracts, following embedded-hal's separate-execution-model lesson [[R01]](#ref-r01). DMA remains an implementation mechanism, not a portable API suffix.
### 8.10 Driver lifecycle and initialization policy

Hardware initialization is explicit and ordered. Constructors bind resources/store configuration; they should not perform long waits, enable application interrupts, or hide startup failure. Fallible hardware bring-up is performed by an explicit initialization operation or BSP/target startup function returning `hal::result`.

Default lifecycle:

```text
construct/bind static resources
        -> initialize clocks/core memory policy
        -> configure board pinmux/resource selections
        -> initialize peripheral/device drivers
        -> install/enable required interrupt routes
        -> construct FreeRTOS queues/tasks/services
        -> enable application event flow
        -> start scheduler
```

MCU peripheral objects are normally constructed once and live for the firmware lifetime. They are non-copyable; move support is provided only when it cannot invalidate ISR/static registrations. Portable `deinitialize()`/reinitialization is not required in V1 unless a real consumer needs it. Avoid hardware-dependent work from function-local/global static constructors.

### 8.11 Blocking and timeout policy

No operation whose progress depends on external hardware may wait forever. Every potentially blocking contract must define one of: an explicit deadline/timeout supplied by the call, a bounded operation determined by configuration/protocol, or a documented nonblocking/polling semantic. GPIO-like register operations need no artificial timeout.

For I2C, serial receive/waits, storage synchronization, external-device reset/probe, Ethernet/PHY bring-up and similar operations, the exact timeout/deadline policy must be frozen with the corresponding contract before implementation. A timeout must also define post-timeout state: whether the bus/peripheral is immediately reusable, aborted/reset, partially transferred, or requires recovery.

Do **not** mechanically add a timeout parameter to every method. The contract should expose a caller-supplied deadline/timeout only when caller control of worst-case execution time is part of the portable semantic requirement. Operations whose maximum duration is derivable from fixed configuration/protocol state may use a configured bound; naturally nonblocking operations use `try_`/polling semantics. Before a contract is declared V1-stable, its chosen progress model and WCET-relevant assumptions must be documented and covered by conformance tests.

### 8.12 Failure classification and fatal policy boundary

Keep three classes separate:

1. **normal runtime hardware outcomes** (`NACK`, bus-off, media removed, timeout) -> `hal::result<T,E>`;
2. **startup/configuration failures** -> returned to BSP/composition, which chooses retry, degraded mode, safe state, or reset;
3. **programmer/internal invariant violations** -> compile-time rejection where possible, otherwise the firmware-level assert/panic policy.

`hal-core` does not decide the product's reset/logging/safe-state behavior. The ECU template may provide a `firmware::panic(...)`/fatal policy, but target and device drivers report errors rather than silently resetting the system.

## 9. Canonical common C++20 primitives

The following baseline is intentionally small and allocation-free. `hal::result` exists because `std::expected` is C++23; `hal::span` keeps the public API usable on C++20 embedded libraries with incomplete freestanding library support. The reference header was syntax-checked with GCC in C++20 mode.

The authoritative common declarations live under `ecu-drivers/include/hal/`. Capability contracts are
in the focused `hal/<capability>.hpp` headers; shared primitives are in `hal/foundation/`. Their
architectural requirements are:

- `hal::result<T,E>` is allocation-free and provides explicit success/failure without exceptions;
- `hal::span<T>` is non-owning and never allocates. Array/`std::array` construction is preferred because extent/pointer validity is structural; raw pointer/length construction has the explicit precondition `size == 0 || data != nullptr`, exposed by `span::valid()` for boundary checks;
- malformed raw spans are programmer/precondition violations, not hardware failures. A backend or portable helper must not dereference caller-provided storage until the span invariant is established; checked/conformance builds should assert that invariant before access;
- strong units (`hertz`, `nanoseconds`, `microvolts`, `instant`, `utc_time`) prevent accidental unit mixing and remain tiny structural C++20 types.

The header is syntax/conformance checked directly by CI; this document deliberately does not duplicate those implementations.
## 10. Initial target capability overview

The matrix below is an architecture-level capability map, not a substitute for final package/pin validation. STM32H723VGT6 is the exact LQFP100 part. ST's current product data reports 17 16-bit timers, 4 32-bit timers, 5 SPI, 6 UART, 5 USART, 3 CAN-FD instances and 100-Mbit/s IEEE-1588-capable Ethernet for that ordering code; the family documentation also lists three ADCs, up to five I2C interfaces, two SD/SDIO/MMC interfaces, two watchdogs and a hardware-calendar RTC [[R23]](#ref-r23) [[R24]](#ref-r24). The ESP32-S3 module selected is specifically the N16R8 version with 16 MB flash and 8 MB Octal PSRAM [[R28]](#ref-r28).

| Capability | STM32H723VGT6 | ESP32-S3-WROOM-1-N16R8 | Linux integration | `sim` backend |
|---|---|---|---|---|
| GPIO | Native; exact package routing generated | Native; module exposes up to 36 GPIOs, with module/strapping restrictions | gpiochip character-device v2 on capable hosts | Deterministic in-memory lines, edges and fault injection |
| ADC latest sample | Native: 2x16-bit + 1x12-bit continuous acquisition resources | Native: two 12-bit SAR ADCs, up to 20 channels [[R29]](#ref-r29) [[R46]](#ref-r46) | Optional Linux buffered IIO adapter when real ADC hardware exists | Deterministic constant/scripted/waveform streams |
| ADC continuous | Native target extension, DMA capable | Native target extension; ESP-IDF continuous driver uses DMA [[R35]](#ref-r35) | Optional IIO buffered adapter | Deterministic sampled-stream extension |
| Classical CAN | FDCAN controllers also support classical frames | One TWAI controller, classical CAN only [[R30]](#ref-r30) | SocketCAN on real/virtual CAN interfaces [[R38]](#ref-r38) | Deterministic virtual CAN bus/queues |
| CAN FD | **Native**: 3 CAN-FD-capable controllers | **Not on-chip**; external SPI CAN-FD controller required [[R30]](#ref-r30) | SocketCAN CAN-FD where interface supports it [[R38]](#ref-r38) | Optional FD-capable virtual bus model |
| SD/MMC block storage | Native SDMMC interfaces | Two SDMMC slots [[R34]](#ref-r34) | File/block-device adapter using `pread`/`pwrite`/`fsync` | In-memory or file-image block model with fault injection |
| Ethernet MAC | Integrated 10/100 MAC + DMA; PHY is external | **No integrated Ethernet MAC** [[R33]](#ref-r33) | TAP/AF_PACKET can implement the higher-level frame-port contract; not a fake MCU MAC [[R39]](#ref-r39) | Deterministic Ethernet frame-port model |
| External Ethernet controller | Possible over SPI/etc. | Primary wired-Ethernet path (e.g. SPI Ethernet) [[R45]](#ref-r45) | Real USB/SPI/network integration is adapter-specific | Simulated external controller/device model where needed |
| I2C | Up to five family interfaces; exact package routes generated | Two SoC I2C controllers | `/dev/i2c-*` via Linux i2c-dev [[R50]](#ref-r50) | Deterministic bus plus attachable/scripted target models |
| SPI | Product table reports 5 SPI for VGT6; exact routes generated | Two general-purpose SPI controllers in addition to memory SPI blocks | `spidev` where appropriate [[R49]](#ref-r49) | Deterministic bus/device transaction model |
| PWM | Timer channels; advanced modes target-specific | LEDC + MCPWM, with advanced motor control target-specific | Optional Linux PWM userspace integration when exposed by the system | Deterministic state/time model |
| RTC wall clock | Hardware calendar + subsecond RTC, VBAT support | RTC/system-time source persists sleep/warm reset but not power-on reset [[R31]](#ref-r31) | `CLOCK_REALTIME` and optional `/dev/rtc*` adapter | Manually controlled wall clock with reset/persistence scenarios |
| Watchdog | IWDG/WWDG-style hardware resources; product family lists 2 | Multiple hardware/system watchdog roles [[R32]](#ref-r32) | Explicit integration-only `/dev/watchdog*` adapter [[R41]](#ref-r41) | Safe deterministic expiry/reset-intent model; never reboots host |
| Monotonic timer/alarm | Many timers/SysTick/DWT choices | 52-bit system timer + four 54-bit general-purpose timers [[R29]](#ref-r29) | `CLOCK_MONOTONIC`, `clock_nanosleep`, `timerfd` as appropriate | Manual deterministic clock/event scheduler |
| Raw internal nonvolatile | MCU flash; erase/program geometry must be respected | 16 MB module SPI flash, partitioned | File-backed integration adapter if desired | In-memory/file-backed NOR model enforcing erase/program rules |

### 10.1 STM32H723-specific architectural warning: Cortex-M7 caches and DMA

STM32H72x/H73x devices have separate 32-KiB I-cache and D-cache [[R26]](#ref-r26). ST's cache documentation explains that CPU D-cache and independent DMA masters require explicit coherency strategy for cacheable buffers [[R27]](#ref-r27). This matters directly for Ethernet, SDMMC, ADC DMA, serial DMA, and potentially other high-throughput backends.

Therefore the STM32 backend must have a **central DMA buffer policy**, not ad-hoc cache calls scattered across every driver. Recommended options:

- dedicated non-cacheable MPU region for DMA descriptors/small control buffers where appropriate;
- cache-line-aligned caller buffers plus clean/invalidate operations at well-defined ownership transitions;
- compile-time traits describing whether a memory region is DMA-accessible and cacheable;
- conformance tests that run with D-cache enabled.

Do not expose cache maintenance in the portable peripheral contracts. It is an STM32 backend responsibility unless the application explicitly opts into a zero-copy DMA-specific extension.

### 10.2 ESP32-S3-specific architectural warning: ESP-IDF high-level resource allocation

Some modern ESP-IDF drivers use dynamically created handles. For example, the GPTimer API's `gptimer_new_timer()` can fail with `ESP_ERR_NO_MEM` [[R37]](#ref-r37); NVS documents a heap RAM footprint that grows with partition size and key count [[R36]](#ref-r36). Those APIs are useful, but they do not automatically satisfy this project's strict static-allocation profile.

The ESP target therefore has two possible implementation strata:

1. **Strict HAL profile (core target):** use Espressif SoC/HAL/LL facilities or project-owned static resource structures; no dynamic driver creation after startup.
2. **Optional ESP-IDF hosted adapter profile:** permit selected IDF high-level components behind the same semantic contracts when an application explicitly relaxes the no-heap rule.

The strict profile is the profile that **will** be used for conformance to this document. Until Milestone 0 implements the build distinction and CI/link checks, this is an architectural requirement rather than a proven implementation guarantee. Milestone 0 must make it mechanical: CI audits the `hal-target-esp32s3` strict-profile objects/link map for forbidden allocation symbols and maintains an explicit deny-list for selected high-level ESP-IDF handle/resource-creation APIs known to allocate dynamically. Any permitted vendor call in the strict backend must have its allocation/lifetime behavior reviewed and covered by a regression test. The optional hosted profile is built/tested separately and must never be mistaken for strict-profile conformance.
## 11. Time and timers — no universal `Timer` interface

A generic MCU `Timer` is intentionally **not** part of the portable API. The STM32H723 exposes a large and heterogeneous timer set — general-purpose, low-power, PWM/motor-control, input-capture, encoder, watchdog and system timing roles — while the ESP32-S3 exposes general-purpose timers, a system timer, LEDC, MCPWM and other timing blocks [[R23]](#ref-r23) [[R29]](#ref-r29). A single `Timer` object would either be the lowest common denominator or a capability bitmap masquerading as an interface.

Instead the portable layer models **purposes**: monotonic time, delays, and one-shot alarms. PWM is its own contract; future input-capture, quadrature encoder, periodic ticker, pulse counter and waveform-generator contracts should be added independently only when real consumers justify them. This is also consistent with the warning from embedded-hal's redesign, where generic timer/capture abstractions from older versions did not survive into 1.0 [[R01]](#ref-r01).

### Contract decisions

- `MonotonicClock::now()` never moves backwards within one boot/session.
- `Delay::delay_for()` is a simple synchronous primitive; an async sleep belongs in a future async layer.
- `OneShotAlarm` is an optional active capability. The portable object exposes arm/cancel/expired, not vendor callback/IRQ registration.
- Timer allocation/configuration occurs in the board/target layer. A `pwm::Output` or `OneShotAlarm` may internally use the same hardware timer only if the board's resource model guarantees there is no conflict.

### Target mapping

| Target | Mapping |
|---|---|
| STM32H723 | Select SysTick/DWT/general-purpose/LPTIM according to accuracy/sleep needs. Board generator reserves the chosen hardware resource. |
| ESP32-S3 | System timer or a statically owned general-purpose timer; avoid allocation-heavy high-level GPTimer in strict profile unless statically wrapped. |
| Linux | `CLOCK_MONOTONIC` for monotonic time; `clock_nanosleep`/`timerfd` may back integration alarms/delays [[R42]](#ref-r42). |
| `sim` | `sim::ManualClock` and deterministic event scheduler. Tests advance time explicitly; no real sleeping. |

### Authoritative C++20 declaration

The normative C++ declarations live in the focused `ecu-drivers/include/hal/<capability>.hpp` headers.
This document defines system semantics and target mapping without duplicating complete declarations.
## 12. GPIO

GPIO is intentionally split into small capability concepts instead of a universal mutable pin object. This prevents a portable sensor/display driver from reconfiguring pinmux, drive strength, analog mode or alternate functions behind the board's back. embedded-hal uses separate digital input/output traits for similar reasons, while Zephyr demonstrates how richer target GPIO configuration can remain subsystem- or device-specific [[R01]](#ref-r01) [[R08]](#ref-r08).

### Contract decisions

- Pinmux, pull resistor, output type, drive strength, slew rate, alternate-function routing and initial mode are board/target configuration.
- `OutputPin` promises logical writes only.
- `InputPin` promises logical reads only.
- `StatefulOutputPin` adds latch readback when that semantic can be provided reliably.
- `EdgeInput` is a refinement using a statically owned **notification latch**. It intentionally does not standardize callbacks or ISR registration.
- The latch is deliberately coalescing: one or more qualifying hardware events may be represented by one pending notification. It is therefore suitable for "wake and inspect current state" semantics, not lossless pulse counting.
- Consuming the latch is one atomic/data-race-free operation with respect to ISR/event production; V1 must not use a separate `event_pending()` + `clear_event()` pair that can erase an event arriving between the two calls.
- Consumers requiring exact edge multiplicity, timestamps, or an ordered event stream must use a future `EdgeCounter`/capture/queued-event refinement or target-specific hardware capability rather than assuming `EdgeInput` provides those guarantees.
- Active-low board semantics can use the `hal::gpio::ActiveLowPin` adapter from `hal/gpio/active_low.hpp`. It borrows the physical pin and preserves the applicable `OutputPin`, `InputPin`, `StatefulOutputPin`, and `EdgeInput` contracts. A separate `hal-adapters` package may retain its `InvertedOutputPin`/`InvertedInputPin` composition names; those are adapters over the current core concepts, not additional core interfaces.

### Target mapping

| Target | Mapping / features |
|---|---|
| STM32H723VGT6 | Native GPIO/EXTI. Exact port/pin/AF set is generated from LQFP100 metadata. Product table reports 80 high-current I/Os for VGT6 [[R23]](#ref-r23). |
| ESP32-S3-WROOM-1-N16R8 | Native GPIO/GPIO matrix; module datasheet exposes up to 36 GPIOs and module/strapping restrictions must be encoded [[R28]](#ref-r28). |
| Linux | gpiochip character-device v2 adapter for real Linux-exposed GPIO [[R40]](#ref-r40). |
| `sim` | Pure in-memory lines with deterministic level/edge control and optional fault injection. |

### Authoritative C++20 declaration

The normative declaration lives in its focused `ecu-drivers/include/hal/` capability header. This
section defines semantics and target mapping without duplicating the C++ declaration.
## 13. I2C

The core primitive is one **transaction** containing read/write operations. This is chosen directly from the embedded-hal 1.0 contract, whose transaction semantics specify START before the first operation, no STOP/repeated START between adjacent operations of the same direction, repeated START when direction changes, and STOP after the final operation [[R03]](#ref-r03). This correctly represents register-address-write followed by repeated-start read without leaking vendor flags. The I2C protocol itself should be consulted when adding controller/target/multi-controller features [[R44]](#ref-r44).

### Contract decisions

- V1 is controller/master mode only. Target/slave mode is a separate future concept.
- 7-bit addressing is the normal core concept. 10-bit addressing is a separate refinement (`Controller10`).
- `write`, `read`, and `write_read` are convenience functions implemented on top of `transaction`.
- A backend that models `Controller7` must faithfully implement repeated-start semantics. If a DMA engine cannot do that for a particular sequence, the backend must fall back to another mechanism rather than silently change bus semantics.
- Bus speed is configured in target/board construction. Runtime reconfiguration is not part of the current `hal-core` I2C concepts; a future refinement may be added only if portable consumers truly need it.
- DMA is private implementation detail.
- `i2c::operation` is an immutable, non-aggregate descriptor created only through `operation::write(...)` or `operation::read(...)`; callers cannot independently set direction/pointers/lengths into contradictory states. Transactions accept `span<const operation>`.
- Operation factories inherit the `hal::span` storage invariant. Before dereferencing operation buffers, backends must establish `operation::valid()`/span validity; malformed raw pointer/length spans are programmer/precondition failures and must never become unchecked memory access.
- The final V1 `transaction()` signature is not frozen until Milestone 2 chooses and tests the bus progress policy. No implementation may wait indefinitely for externally controlled progress such as clock stretching. Whether the bound is caller-supplied or construction/configuration state must be identical at the semantic level across conforming backends.

### Target mapping

| Target | Mapping / differences |
|---|---|
| STM32H723 | Hardware I2C, family advertises up to 5 Fm+ interfaces. Exact instances/pins in VGT6 generated from device/package data. SMBus/PMBus features remain target extensions. |
| ESP32-S3 | Two hardware I2C controllers. Standard generic controller operations map naturally; ESP-specific filter/timing/power features remain target-side. |
| Linux | `/dev/i2c-*` via i2c-dev for real Linux I2C integration [[R50]](#ref-r50). |
| `sim` | Deterministic I2C bus with scripted expectations and attachable stateful target-device models. This is the main CI backend. |

### Authoritative C++20 declaration

The normative declaration lives in its focused `ecu-drivers/include/hal/` capability header. This
section defines semantics and target mapping without duplicating the C++ declaration.
## 14. SPI

SPI uses two related contracts: `Bus8` for exclusive access to the physical SPI bus and `Device8` for a transaction against one logical slave. embedded-hal explains why this split is essential on shared buses: the device layer owns/selects CS and guarantees that no competing device transaction interleaves with it [[R04]](#ref-r04). RIOT independently models SPI as a shared acquired/configured resource [[R13]](#ref-r13).

### Contract decisions

- V1 portable word size is 8 bits. Wider word/frame modes are refinements, not assumptions baked into every driver.
- `config8::max_frequency` means the backend must choose an achievable clock **not exceeding** the requested maximum. `configure()` returns the actual clock.
- Mode 0..3 and MSB-first form the baseline. LSB-first is an optional refinement.
- `read_fill` defines what is shifted on MOSI when the caller is reading.
- `transfer(tx, rx)` may have different TX/RX lengths. The operation lasts `max(tx_size, rx_size)` bytes; extra incoming bytes are discarded after RX is full, and `read_fill` is transmitted after TX is exhausted. This follows a proven portable SPI semantic [[R04]](#ref-r04).
- `Device8::transaction()` is responsible for bus exclusivity, applying the device configuration, asserting CS, performing all operations, flushing, deasserting CS, and releasing the bus. On errors it should make a best effort to deassert CS.
- The `Device8` concept deliberately exposes only `transaction()`. A concrete/static device adapter owns or references the bus, CS mechanism, and per-device configuration; those implementation data members are not requirements of the concept.
- `operation8` is an immutable, non-aggregate descriptor created through the named factories (`write`, `read`, `transfer`, `delay_for`). The transaction receives `span<const operation8>`, preventing callers from fabricating contradictory operation-kind/pointer/length combinations. Operation-buffer span invariants are checked before dereference.
- `operation8::delay_for()` means a delay **inside an active transaction while CS remains asserted**. CS-assert-to-first-clock setup time and last-clock-to-CS-deassert hold time are device configuration properties, not transaction delay operations; a standard `StaticSpiDevice` adapter should carry them alongside the bus configuration and use hardware-managed timing when available.
- The locking mechanism is a statically composed implementation detail; bare metal may use a critical-section/resource guard, while application/RTOS-level composition may provide a fixed lock when sharing cannot be avoided.
- SPI transfers are normally bounded by operation sizes and configured clock rate; any backend-specific wait for BUSY/flush completion must itself have a documented bounded abort/timeout policy rather than spin forever.

### Target mapping

| Target | Mapping / differences |
|---|---|
| STM32H723VGT6 | Product table reports five SPI instances for the exact part [[R23]](#ref-r23); additional synchronous USART modes are not treated as ordinary SPI instances unless a target adapter deliberately exposes them. DMA/FIFO modes are backend details. |
| ESP32-S3 | Two general-purpose SPI controllers; the other SPI blocks are used for attached memory. SoC hardware supports richer Dual/Quad/QPI and half-duplex modes, which remain ESP-specific/refined APIs [[R29]](#ref-r29). |
| Linux | `spidev` integration adapter where Linux can preserve the required transaction semantics [[R49]](#ref-r49). |
| `sim` | Deterministic SPI bus/device model with exact transaction tracing, CS semantics and fault injection. |

### Authoritative C++20 declaration

The normative declaration lives in its focused `ecu-drivers/include/hal/` capability header. This
section defines semantics and target mapping without duplicating the C++ declaration.
## 15. ADC

ADC is deliberately conservative. embedded-hal 1.0 did not carry forward its older ADC traits, a useful warning that ADC hardware is difficult to standardize across resolution, attenuation/reference, scan sequences, triggers, oversampling, differential modes and DMA [[R01]](#ref-r01). The current portable baseline is a configured `Channel` whose nonblocking reads observe the latest value published by an independently running acquisition engine.

### Contract decisions

- Channel configuration (resolution, attenuation/reference, sample time, differential/single-ended choice) is board/target construction state and may be encoded in the concrete channel type.
- `Channel::read_raw()` observes the most recently published `hal::adc::raw_sample`; it never starts or waits for a conversion. Before the first conversion is published, it returns `not_ready`.
- `Channel::read_voltage()` observes the corresponding `hal::adc::voltage_sample` in integer microvolts. The target/BSP owns the calibration and conversion meaning needed to provide this view.
- Every `hal::adc::sample<Value>` includes its `value`, capture timestamp, and monotonically increasing sequence number so consumers can detect freshness without comparing signal values.
- `properties()` returns `hal::adc::characteristics`: resolution bits and nominal minimum/maximum voltage.
- `ContinuousChannel` is the loss-aware refinement for consumers such as software filters that must observe every published raw sample. `try_read()` copies currently available `raw_sample` values and reports overwritten samples through `stream_read::dropped`; `stream_read::count` reports how many were copied. Each concrete channel owns one advancing consumer cursor unless it explicitly provides a stronger target-specific model.
- Acquisition start/stop, hardware scan sequences, trigger routing, DMA ownership, hardware oversampling, threshold monitors and injected groups remain target/BSP configuration or target extensions.
- Hardware oversampling remains integer-domain ADC behavior. Software block and moving averages are portable utilities in `hal/adc/averaging.hpp`; raw codes remain integers and the baseline calibrated view uses integer microvolts rather than floating point.
- The board's analog low-pass/anti-alias filter is a BSP electrical fact. It is documented alongside sample-rate assumptions but does not become a software interface method.

### Target mapping

| Target | Mapping / differences |
|---|---|
| STM32H723 | Family has two 16-bit ADCs and one 12-bit ADC with high-rate sequencing, hardware oversampling and DMA features [[R23]](#ref-r23) [[R24]](#ref-r24). Target acquisition publishes into statically owned channel buffers. |
| ESP32-S3 | Two 12-bit SAR ADCs, up to 20 analog channels [[R29]](#ref-r29) [[R46]](#ref-r46). A continuous DMA acquisition can back `Channel` and `ContinuousChannel`; target/BSP calibration provides the `voltage_sample` view [[R35]](#ref-r35). |
| Linux | Optional buffered IIO adapter for actual Linux-exposed ADC hardware; the kernel buffer acquires independently and `/dev/iio:deviceX` is drained without triggering conversion [[R51]](#ref-r51). |
| `sim` | Deterministic constant, scripted, waveform, or explicitly test-controlled sample source. |

### Authoritative C++20 declaration

The normative declaration lives in its focused `ecu-drivers/include/hal/` capability header. This
section defines semantics and target mapping without duplicating the C++ declaration.
## 16. CAN and CAN FD

Classical CAN and CAN FD must not be one giant interface with `supports_fd()`. The selected targets prove why: STM32H723 has CAN-FD-capable controllers, while ESP32-S3's on-chip TWAI controller is classical CAN only and explicitly treats FD frames as errors [[R23]](#ref-r23) [[R30]](#ref-r30). Linux SocketCAN supports both classical and CAN-FD frames [[R38]](#ref-r38).

### Contract decisions

- `can::ClassicController` is the classical CAN baseline: 11/29-bit identifiers, up to 8 data bytes, polling try-send/try-receive.
- `can::FdController` refines `ClassicController` and adds CAN-FD frame operations up to 64 bytes plus BRS/ESI fields.
- A CAN-FD controller must also satisfy the classical controller concept.
- Nonblocking `try_*` is used to avoid unspecified wait time. `try_send` returns success/failure plus a boolean indicating whether the frame was accepted now.
- Hardware filtering is a refinement (`FilterableController`), because filter quantity/layout differs substantially. A software-filter adapter can provide generic filtering if needed.
- Bit timing, transceiver standby pins, automatic retransmission, loopback/listen-only and detailed bus-state diagnostics are board/target configuration or target extensions until a portable consumer demonstrates a common requirement.

### Target mapping

| Target | Classical CAN | CAN FD | Notes |
|---|---|---|---|
| STM32H723VGT6 | Yes | **Yes** | Product data reports 3 CAN-FD controllers [[R23]](#ref-r23). Use FDCAN backend; preserve detailed protocol/bus-off diagnostics in concrete error. |
| ESP32-S3-WROOM-1-N16R8 | **Yes**, one TWAI controller | **No on-chip** | Espressif explicitly states S3 TWAI is not FD-compatible [[R30]](#ref-r30). An external MCP2518FD-like SPI device can implement `FdController` as a portable device adapter. |
| Linux | Yes | Yes | SocketCAN provides classical CAN and CAN-FD support where enabled by the interface; `vcan` is useful for Linux integration tests [[R38]](#ref-r38). |
| `sim` | Yes | Configurable | Deterministic virtual bus with explicit arbitration/order/fault policy; can model classical-only or FD-capable networks. |

### Frame and filter validity

Portable CAN objects must not rely on comments alone for protocol validity.

- Classical data-frame payload length is 0..8 bytes. Remote frames carry a requested DLC/length but no data payload.
- CAN-FD payload length must map to a legal DLC: 0..8, 12, 16, 20, 24, 32, 48, or 64 bytes. `fd_frame::dlc()` exposes the corresponding encoded DLC so backends do not duplicate conversion rules.
- Public frame construction uses validated factories. A successfully constructed `frame`/`fd_frame` is therefore valid by construction; target backends do not need to defend against arbitrary public `size` fields.
- Portable filter matching is defined independently of controller register layout. A filter matches only identifiers of the same format (standard vs extended), and then matches when `(candidate.value & mask) == (filter.id.value & mask)`. Mask bits outside the selected identifier width are invalid.
- Hardware filter capacity/shape is not hidden. `set_filters()` may return `configuration` when a valid portable filter set cannot be represented because of target resource/capacity limits; software post-filtering is an adapter decision, not silently assumed by the core contract.

### Authoritative C++20 declaration

The normative declaration lives in its focused `ecu-drivers/include/hal/` capability header. This
section defines semantics and target mapping without duplicating the C++ declaration.
## 17. Block device / SD storage

A filesystem generally wants a **logical block device**, not an SDMMC peripheral. Therefore the common contract is media-oriented: block geometry, read blocks, write blocks, sync, and optional trim. SD card protocol/host initialization is an implementation/adaptation layer below this contract.

CMSIS-Driver similarly distinguishes storage/memory-card interfaces from unrelated peripheral APIs [[R10]](#ref-r10). The design intentionally does not expose SD command numbers, bus width, card-detect GPIO, DMA descriptors or filesystem concepts in `block::Device`.

### Contract decisions

- `logical_block_size` and `logical_block_count` are runtime geometry because removable media capacity is discovered at runtime.
- Buffers passed to `read_blocks`/`write_blocks` must contain an integral number of logical blocks; target backends return `not_aligned` otherwise.
- `sync()` means all writes accepted by the object have reached the durability level promised by that backend/media. It is required before removable-media ejection or test assertions about persistence.
- `TrimDevice` is optional because discard/erase support varies.
- Filesystems, caching, partition tables, FAT/LittleFS, and wear-leveling do not belong in this low-level contract.

### Target mapping

| Target | Mapping |
|---|---|
| STM32H723 | SDMMC host + SD/eMMC protocol driver produces `block::Device`. Family lists two SD/SDIO/MMC interfaces [[R23]](#ref-r23). STM32 M7 DMA/cache policy is critical for data buffers. |
| ESP32-S3 | SDMMC host has two slots and supports SD/eMMC bus widths/speed modes [[R34]](#ref-r34). Strict static backend must audit any ESP-IDF handle/buffer allocation or implement lower-level static ownership. |
| Linux | Fixed file or actual block-device descriptor using `pread`/`pwrite` and `fsync` for integration [[R43]](#ref-r43). |
| `sim` | In-memory or file-image block model with deterministic short-write, media-removal and torn-write/power-loss injection. |

### Authoritative C++20 declaration

The normative declaration lives in its focused `ecu-drivers/include/hal/` capability header. This
section defines semantics and target mapping without duplicating the C++ declaration.
## 18. Ethernet — MAC, PHY, and application-facing frame port

Ethernet needs **two abstraction levels** because the selected targets have fundamentally different topology. STM32H723 has an integrated 10/100 Ethernet MAC and needs an external PHY connected through MII/RMII-style signals; ST documents STM32 10/100 MAC support and IEEE 1588 [[R23]](#ref-r23) [[R25]](#ref-r25). ESP32-S3 explicitly has no integrated Ethernet MAC and uses an external Ethernet interface such as SPI-Ethernet [[R33]](#ref-r33) [[R45]](#ref-r45). Pretending both are “an MCU MAC + PHY” would make the ESP adapter artificial.

### Contract decisions

- Low-level composition contracts: `MdioBus`, `Phy`, and `Mac` support integrated-MAC designs.
- Preferred application/network-stack dependency: `FramePort`, representing a complete Ethernet L2 frame interface with MAC address and link state.
- STM32 adapter composes `Stm32Mac + ExternalPhy` into `FramePort`.
- ESP external chips such as W5500/DM9051/KSZ8851-like controllers may implement `FramePort` directly because MAC+PHY may be inside the external chip.
- Linux TAP naturally implements `FramePort` because userspace reads/writes Ethernet frames [[R39]](#ref-r39).
- PTP hardware timestamping is a refinement (`PtpFramePort`) because STM32 supports IEEE-1588-relevant hardware while external ESP controllers and TAP configurations may differ.
- TCP/IP, DHCP, ARP, sockets, TLS, and network-stack ownership are above this layer.

### Target mapping

| Target | Mapping / features |
|---|---|
| STM32H723 | Native MAC+DMA; external PHY via board hardware; `MdioBus` + concrete PHY + `Mac` -> composed `FramePort`. IEEE-1588/PTP can satisfy refinement where implemented. |
| ESP32-S3 | No integrated Ethernet MAC. External SPI Ethernet chip -> `FramePort`; whether it internally contains MAC+PHY is irrelevant to the portable network stack. |
| Linux | TAP is preferred for userspace frame-port integration; AF_PACKET is another option when binding to an existing interface [[R39]](#ref-r39). This implements the higher-level `FramePort`, not a fictitious MCU MAC/PHY. |
| `sim` | Deterministic Ethernet frame queues/link-state model for protocol/application tests. |

### Authoritative C++20 declaration

The normative declaration lives in its focused `ecu-drivers/include/hal/` capability header. This
section defines semantics and target mapping without duplicating the C++ declaration.
## 19. Nonvolatile memory and emulated EEPROM

Do **not** expose internal flash as if it were byte-overwritable EEPROM. Raw NOR flash and logical EEPROM-like storage have different invariants. embedded-storage's `NorFlash` contract explicitly models write/erase sizes and the fact that erased state and power-loss behavior matter [[R05]](#ref-r05).

The portable design therefore defines two layers: `NorFlash` for physical flash semantics, and `Memory` for logical overwriteable nonvolatile bytes. A wear-levelled journal adapter converts the former into the latter.

### Contract decisions

#### Raw `NorFlash`

- fixed capacity/read/program/erase geometry;
- `program` must respect erased-state/program rules;
- `erase` is aligned to erase geometry;
- `sync` makes pending backend operations durable;
- power loss during erase/program may leave the affected region invalid according to the concrete backend's documented guarantees.

#### Logical `Memory`

- arbitrary bytes can be overwritten without caller-managed erase;
- the object is single-owner/not implicitly thread-safe, consistent with the V1 resource model;
- a concrete journal-backed implementation performs mount/recovery during its explicit initialization and is exposed as `Memory` only after recovery succeeds;
- `max_atomic_write_size` is an explicit property. A `write()` larger than this limit fails with `too_large` rather than silently weakening atomicity;
- each accepted `write()` is a logical all-or-nothing update with respect to recovery. After a power interruption, a write is observed either in its previous state or its complete new state, never as a torn logical byte range;
- successful writes are ordered. Before `sync()`, a sudden power loss may discard a suffix of otherwise successful unsynchronized writes, but recovery must produce a valid state corresponding to a prefix of the accepted write order rather than arbitrary corruption;
- successful `sync()` is the durability boundary: all successful writes completed before that `sync()` must be recoverable after the documented power-loss/restart model of the backend;
- interrupted garbage collection/compaction must preserve the last committed generation; recovery ignores incomplete destination generations and resumes from a valid committed state;
- V1 does not promise space reservation. `write()` may return `no_space` when bounded static journal capacity/garbage-collection constraints cannot satisfy the request;
- implementation provides journaling/wear leveling/recovery internally; key/value settings or transactions spanning multiple independent writes belong in a higher-level store unless real consumers justify a portable refinement.

#### Proposed static journal adapter

A first professional implementation can reserve two or more erase regions and use append-only records:

```text
record header:
  magic/version
  logical offset or key
  payload length
  monotonically increasing sequence
  payload CRC
  header CRC
  commit state / commit marker

write:
  1. append uncommitted header + payload
  2. verify/write CRC as required
  3. program commit marker last
  4. old record remains valid until new record is committed

mount/recovery:
  scan regions
  ignore torn/uncommitted/CRC-invalid records
  select newest committed record for each logical range/key

garbage collection:
  copy latest live records to fresh erase region
  commit destination generation
  erase obsolete region only after destination is valid
```

All indexing/scratch storage must have compile-time capacity or be caller-supplied. If the use case is small configuration data, consider a key/value settings layer *above* `Memory`; do not force key/value semantics into the byte-addressed contract.

### Target mapping

| Target | Raw storage | Logical NV strategy |
|---|---|---|
| STM32H723 | Internal flash; RM0468 documents large user erase sectors and flash-specific program geometry. Reserve dedicated sectors in linker layout. | `JournalMemory<Stm32NorFlash, StaticScratch...>`. Avoid frequent rewrite patterns; M7 cache/execution-from-flash interactions must be tested. |
| ESP32-S3 N16R8 | Dedicated partition in 16 MB module SPI flash [[R28]](#ref-r28). | Own static journal for strict profile. ESP NVS is feature-rich and wear-levelled but documents heap RAM consumption, so it belongs in an optional relaxed adapter profile [[R36]](#ref-r36). |
| Linux | Optional fixed-file raw-storage integration adapter. | Linux file-backed logical `Memory` adapter when persistence across host runs is useful. |
| `sim` | Memory/file-image NOR model that enforces erase/program geometry and one-way bit programming. | Journal `Memory` over simulated NOR with deterministic torn writes is the primary recovery-test backend. |

### Authoritative C++20 declaration

The normative declaration lives in its focused `ecu-drivers/include/hal/` capability header. This
section defines semantics and target mapping without duplicating the C++ declaration.
## 20. PWM

PWM is modeled by its function — period/frequency and pulse width — rather than by a generic timer peripheral. This is necessary because STM32 timer channels and ESP32-S3 LEDC/MCPWM have different resource-sharing and advanced-feature models. Advanced motor-control functionality is a refinement/target API, not pollution of the baseline.

### Contract decisions

- `configure()` accepts a requested period and an allowed error in ppm, and returns the actual achieved period.
- `set_pulse_width()` uses a time quantity instead of raw timer counts or a floating-point duty ratio; this preserves exactness and avoids silently changing meaning when period changes.
- Backends reject pulse widths outside `[0, actual_period]`.
- The output is safe/disabled after construction and initialization. `configure()` changes timing parameters but does **not** implicitly drive the physical PWM output.
- `enable_output()` explicitly starts driving the configured waveform; `disable_output()` explicitly returns the output to the backend/BSP-defined safe disabled state. `disable_output()` is idempotent, and enabling before successful configuration reports `not_configured`.
- `shared_period_conflict` exists because some hardware channels share a timer/base period.
- `ComplementaryOutput` is an example refinement for dead-time-capable motor-control hardware. Additional refinements such as phase alignment, break input, center-aligned mode, sync groups and fault shutdown should be justified by actual portable motor-control consumers.

### Target mapping

| Target | Mapping |
|---|---|
| STM32H723 | Timer channel output; rich timer/motor-control features remain target refinements. Resource generator must track channels that share a timer period. |
| ESP32-S3 | LEDC for ordinary PWM; MCPWM for complementary/motor-control features. The existence of these two very different blocks is exactly why `pwm::Output` should express purpose rather than expose a “timer”. |
| Linux | Optional `/sys/class/pwm` integration where the host kernel exposes PWM; Linux documents period, duty-cycle, polarity and enable controls [[R52]](#ref-r52). |
| `sim` | Deterministic PWM state model tied to `sim::ManualClock`; no host timing dependence. |

### Authoritative C++20 declaration

The normative declaration lives in its focused `ecu-drivers/include/hal/` capability header. This
section defines semantics and target mapping without duplicating the C++ declaration.
## 21. RTC / wall-clock time

The portable RTC contract represents **wall-clock UTC time**, not calendar-register layout. STM32H723 has a hardware calendar RTC with subsecond support and backup-domain supply features [[R23]](#ref-r23). ESP32-S3 system time can use an RTC timer that persists through sleep and resets except power-on reset, but it is not equivalent to a battery-backed calendar RTC [[R31]](#ref-r31). Those persistence differences are exposed as static characteristics rather than hidden.

### Contract decisions

- `utc_time` uses signed epoch seconds plus nanoseconds. Calendar conversion belongs in a utility layer.
- `Clock::persistence_characteristics()` lets generic policy code know whether time survives warm reset, deep sleep, and main-power loss with backup supply.
- These static characteristics belong to the **fully configured concrete clock type**, not to the bare MCU family in isolation. Clock-source policy, reset behavior, module configuration and board backup-power topology must therefore be reflected in the selected concrete type/traits before it claims a static persistence profile. If persistence truly varies after construction for a future implementation, that implementation must use a separate runtime-status refinement rather than publishing a false static fact.
- `Alarm` is a refinement; not every wall clock needs to support a hardware wake alarm through the same interface.
- Calibration, oscillator selection, backup-register access, tamper detection, timezone/DST, SNTP discipline and leap-second policy are separate concerns.

### Target mapping

| Target | Mapping |
|---|---|
| STM32H723 | Hardware RTC/calendar; can survive main VDD loss when the backup domain is correctly powered/configured. |
| ESP32-S3 | System time backed by RTC timer/high-resolution timer. RTC timer survives sleep and non-power-on reset; power-on reset clears it, so `can_survive_main_power_loss_with_backup_supply = false` for the module alone. External RTC can provide a stronger implementation. |
| Linux | `CLOCK_REALTIME` system-clock adapter and, when physical RTC semantics are required, optional `/dev/rtcN` integration [[R53]](#ref-r53). |
| `sim` | Manually controlled wall clock with configurable reset/deep-sleep/main-power-loss persistence behavior. |

### Authoritative C++20 declaration

The normative declaration lives in its focused `ecu-drivers/include/hal/` capability header. This
section defines semantics and target mapping without duplicating the C++ declaration.
## 22. Serial / UART byte stream

Portable consumers such as GNSS modules, modems and console-style protocols normally need a byte stream, not STM32 USART register modes or ESP UART handles. The core therefore models TX/RX byte-stream semantics. Runtime serial reconfiguration is a refinement rather than a requirement for every consumer.

### Contract decisions

- `try_write()` accepts as many bytes as can be accepted immediately into hardware/static buffering and returns that count. Zero is backpressure, not an error.
- `try_read()` returns currently available bytes; zero is normal.
- `flush()` waits until previously accepted TX data reaches the contract's physical-completion boundary.
- Framing/baud configuration normally happens once in board construction. `ConfigurablePort` exists for applications that genuinely need runtime changes.
- Fixed-size RX/TX ring buffers may be members of the concrete backend or caller-supplied template capacity; no dynamic queue growth.
- RS-485 direction control, LIN, synchronous USART/SPI mode, IrDA and modem control are separate adapters/refinements.

### Higher-level text adapter

The portable serial contract remains byte-oriented because UART/serial transports are also used for
binary protocols. Application code that wants to write human-readable text should normally use the
reusable `TextWriter` utility from `hal/serial/text_writer.hpp` above `serial::Tx`. It is not a new
hardware capability and does not replace `serial::Tx`.

Conceptual usage:

```cpp
hal::serial::TextWriter console{board.debug_uart};

auto written = console.try_write("Hello Serial");
```

The adapter accepts `std::string_view`/string literals and internally presents their bytes to `serial::Tx::try_write()`. Its baseline operation remains nonblocking and returns the number of text bytes accepted, preserving the underlying backpressure semantics. It must not silently spin until all text is accepted. If an application needs complete-message delivery, buffering/retry belongs in a higher-level logger/service with an explicit bounded progress policy.

Recommended layering:

```text
application/logger/service
        |
        v
hal::serial::TextWriter
        |
        v
hal::serial::Tx
        |
        v
STM32 UART / ESP UART / Linux tty / sim byte port
```

`TextWriter` should remain formatting-agnostic. Severity levels, timestamps, integer/float formatting, log records, and queueing belong in a logging layer above it. This keeps the core serial abstraction usable for both text and binary protocols.

### Target mapping

| Target | Mapping |
|---|---|
| STM32H723VGT6 | Product table reports multiple UART/USART resources (6 UART, 5 USART entries in the product table); exact package routes and overlapping alternate functions are generated [[R23]](#ref-r23). |
| ESP32-S3 | Three UART controllers per SoC feature set; strict backend can use static FIFOs/interrupt state. |
| Linux | Real `/dev/tty*`/PTY integration using termios; `tcgetattr`/`tcsetattr` and baud/framing controls provide the host mapping [[R54]](#ref-r54). |
| `sim` | Deterministic byte queues with framing/error injection; no OS scheduler dependence. |

### Authoritative C++20 declaration

The normative declaration lives in its focused `ecu-drivers/include/hal/` capability header. This
section defines semantics and target mapping without duplicating the C++ declaration.
## 23. Watchdog

The most portable safety-relevant watchdog operation is **feeding an already configured watchdog**. Configuration/disable semantics differ considerably between MCU watchdogs, and some safety designs intentionally make watchdog disable impossible after startup. ESP32-S3 also distinguishes hardware watchdog timers from system-level interrupt/task watchdog services [[R32]](#ref-r32).

### Contract decisions

- Board/target startup configures and starts the watchdog.
- Portable code receives a `Feeder` only after the watchdog is running.
- There is deliberately no portable `disable()`.
- Static characteristics expose minimum/maximum allowed feed interval so a window watchdog can be represented.
- Higher-level “task watchdog” aggregation belongs above this contract: multiple subsystems can report liveness to a supervisor, and only the supervisor owns the hardware `Feeder`.
- Tests should never reboot the host. The simulator transitions into an explicit expired/fault state and lets the test assert that a reset would have occurred.

### Target mapping

| Target | Mapping |
|---|---|
| STM32H723 | Hardware independent/window watchdog resources; family product page lists two watchdogs [[R23]](#ref-r23). Board selects one and fixes configuration. |
| ESP32-S3 | Raw MWDT/RWDT mechanism may back `Feeder`; ESP task/interrupt watchdogs are system/RTOS services and should not be conflated with the low-level portable feeder [[R32]](#ref-r32). |
| Linux | `/dev/watchdog*` adapter only for explicitly enabled integration environments; never instantiate it in ordinary developer/CI tests [[R41]](#ref-r41). |
| `sim` | Safe watchdog model transitions to an expired/reset-intent state so tests can assert reset behavior without rebooting the host. |

### Authoritative C++20 declaration

The normative declaration lives in its focused `ecu-drivers/include/hal/` capability header. This
section defines semantics and target mapping without duplicating the C++ declaration.
## 24. Cross-cutting implementation policy by target

### 24.1 `hal-target-stm32h7` backend with STM32H723VGT6 exact-device profile

Recommended implementation strategy:

1. Use CMSIS device definitions and direct-register access for simple/hot peripherals where the code remains understandable.
2. It is acceptable to use STM32Cube HAL privately as a bootstrap for complex blocks such as FDCAN, Ethernet or SDMMC, provided:
   - vendor types never leak through public contracts;
   - all allocation/lifetime behavior is audited;
   - global Cube handles are wrapped in target-owned objects rather than becoming application globals;
   - the portable contract's semantics take precedence over HAL quirks;
   - cache/DMA coherency is centrally managed.
3. Keep the repository/package boundary at STM32H7 while building reusable code around actual peripheral-IP revisions where hardware differences require it.
4. Keep exact `STM32H723VGT6` package pin routing, alternate functions, IRQs, DMA facts and memory traits inside the target package's exact-device headers.
5. The BSP selects the exact device and physical wiring. Firmware composition selects operating policy such as clock rate and whole-system DMA/resource assignments.
6. Put startup clock tree, MPU/cache policy and pinmux in BSP/target initialization, never in portable device drivers.
7. Keep errata references adjacent to each workaround and unit/HIL tests.

### 24.2 ESP32-S3-WROOM-1-N16R8 backend

Recommended strategy:

1. Treat `ESP32-S3` SoC and `WROOM-1-N16R8` module as separate metadata layers.
2. Prefer Espressif HAL/LL/SOC-level facilities or project-owned static resource objects in the strict profile.
3. High-level ESP-IDF components may be used only after auditing allocation and lifecycle. Components documented to allocate handles/heap should live behind an optional relaxed profile unless a static configuration path is proven.
4. Do not make the HAL contracts or target driver object model depend on FreeRTOS. Where high-level ESP-IDF facilities inherently impose scheduler/heap/lifecycle assumptions, prefer lower-level HAL/LL/SOC facilities for the strict driver layer or isolate that integration above the portable contracts.
5. External Ethernet and external CAN-FD devices are **portable device drivers/adapters over SPI**, not fake on-chip peripherals.
6. N16R8's external flash/PSRAM topology must be treated as module infrastructure, not free user SPI resources.

### 24.3 Linux integration backend

Linux is a **real host integration target**, not the deterministic simulator. It should map portable contracts to Linux kernel/userspace facilities when the mapping is semantically honest:

- `linux::Can` / `linux::CanFd` -> SocketCAN;
- `linux::EthernetFramePort` -> TAP or AF_PACKET depending the test/integration level;
- `linux::GpioLine` -> GPIO character-device v2;
- `linux::SpiDevice` -> spidev where the transaction semantics can be preserved;
- `linux::I2cController` -> i2c-dev;
- `linux::SerialPort` -> tty/termios;
- `linux::BlockDevice` -> fixed file or real block-device descriptor;
- `linux::Rtc` -> `CLOCK_REALTIME` and optionally `/dev/rtc*`;
- `linux::MonotonicClock` -> `CLOCK_MONOTONIC`;
- `linux::Watchdog` -> `/dev/watchdog*` only in explicit integration environments;
- `linux::AdcChannel` -> optional IIO adapter;
- GPIO/PWM/IIO adapters are optional because availability depends on the host kernel and attached hardware.

Linux is valuable because the same portable driver can communicate with **real hardware** from a PC/SBC and because SocketCAN/vcan/TAP provide excellent integration-test infrastructure. Linux syscalls/kernel internals may allocate; the project invariant forbids hidden allocation in the library object model under project control, not inside the host kernel.

### 24.4 Deterministic `sim` backend

`sim` is mandatory for CI and unit/application simulation. It does **not** call Linux hardware APIs to implement peripheral semantics. Its state is completely test-controlled:

- manually advanced monotonic and wall clocks;
- in-memory GPIO with deterministic edge events;
- I2C/SPI buses with attachable device models and scripted fault injection;
- ADC constant/scripted/waveform sources;
- PWM state models;
- deterministic classical CAN/CAN-FD virtual buses;
- in-memory/file-image block devices and NOR-flash models with torn-write/power-loss injection;
- safe watchdog expiry state that reports that reset would have occurred;
- deterministic Ethernet frame queues.

The simulator should support two test styles:

1. **Expectation mocks** for protocol-level tests (for example, assert the exact SPI byte sequence sent by a W25Q driver).
2. **Stateful device models** for system tests (for example, a W25Q model that enforces write-enable, page-program, busy and erase semantics).

A simulated hour must be able to execute instantly by advancing `sim::ManualClock`; tests must not depend on `sleep_for()` or host wall time.

### 24.5 Startup, linker, and memory ownership

Startup/linker policy is a cross-cutting build/BSP concern, not a portable HAL API. Ownership is split as follows:

| Concern | Owner |
|---|---|
| physical on-chip memory regions, DMA accessibility, cache traits | exact target profile |
| external memories physically populated on the PCB | BSP |
| bootloader/application flash offsets, stack placement, `.noinit`, DMA-buffer sections, task stacks, application memory reservations | firmware/build policy |
| linker script generation/selection and startup object wiring | CMake/toolchain + BSP/target startup |

For STM32H723VGT6, startup must explicitly establish the core/MPU/cache strategy before DMA-capable peripherals are allowed to use shared memory. The target package provides the exact memory facts; the firmware selects placement policy. Do not encode product-specific boot offsets or task-stack placement as immutable MCU metadata.

Heap use is not required by the strict HAL. If the wider FreeRTOS application chooses any heap-based runtime facility, that allocation policy is a firmware decision and must not cause hidden allocation inside `hal-core`/strict target drivers.

## 25. Resource ownership and composition

### 25.1 Peripheral objects are unique resources

Do not casually copy a type that represents a hardware peripheral. Prefer non-copyable concrete backends:

```cpp
class Stm32Spi1 {
public:
    Stm32Spi1(const Stm32Spi1&) = delete;
    Stm32Spi1& operator=(const Stm32Spi1&) = delete;
    // movable only if move semantics do not invalidate ISR/static registration
};
```

For most MCU peripheral types, even move may be unnecessary. The board can construct them once in static/automatic startup scope and keep them for program lifetime.

### 25.2 Dependency injection remains static

```cpp
template<hal::i2c::Controller7 Bus>
class Bme280 {
public:
    explicit Bme280(Bus& bus) noexcept : bus_{bus} {}
private:
    Bus& bus_;
};
```

The reference itself does not introduce polymorphic dispatch: the static template instantiation knows `Bus`.

### 25.3 Shared SPI

A board should build one physical bus object and one statically typed device adapter per chip-select/configuration:

```cpp
Stm32Spi1 spi1;
GpioOutput display_cs;
GpioOutput flash_cs;

SpiDevice<Stm32Spi1, GpioOutput, display_cfg> display_spi{spi1, display_cs};
SpiDevice<Stm32Spi1, GpioOutput, flash_cfg>   flash_spi{spi1, flash_cs};
```

`SpiDevice` is where mutual exclusion lives if the system can access multiple devices concurrently.

### 25.4 Ownership of pinmux and clocks

A sensor driver must never decide that `PB6` should become I2C SCL. The board layer provides an already configured bus. This makes portable drivers portable and prevents two dependencies from silently fighting over the same pin/timer/DMA channel.
## 26. Capability taxonomy

Use exactly four mechanisms for feature diversity.

### Level 1 — mandatory core concept

If the type satisfies it, every operation works according to the contract.

Examples: `gpio::OutputPin`, `i2c::Controller7`, `spi::Device8`, `can::ClassicController`.

### Level 2 — portable refinement concept

Represents genuinely optional but reusable semantics.

Examples: `can::FdController`, `i2c::Controller10`, `adc::ContinuousChannel`, `ethernet::PtpFramePort`, `rtc::Alarm`, `pwm::ComplementaryOutput`.

### Level 3 — portable adapter/emulation

Build a stronger abstraction from simpler contracts when semantics can be faithfully reproduced:

```text
GPIO + clock -> SoftwareI2c -> Controller7
GPIO + clock -> SoftwareSpi -> Bus8
NorFlash     -> JournalMemory -> nv::Memory
SPI + ext CAN-FD chip -> can::FdController
SPI + ext Ethernet chip -> ethernet::FramePort
```

### Level 4 — target-specific extension

Features too target-specific to justify portable standardization remain under target namespaces:

```cpp
hal::stm32h7::advanced_timer<device::stm32h723vgt6>
hal::stm32h7::adc_sequence<device::stm32h723vgt6>
hal::esp32s3::mcpwm_operator
hal::esp32s3::rmt_channel
```

This is not an abstraction failure. Both CMSIS-Driver and Zephyr explicitly support device-specific extensions beyond common subsystem APIs.
## 27. API versioning and stability policy

Portable contracts will be expensive to change once device drivers depend on them. Before declaring a V1 contract stable:

1. implement it through `hal-target-stm32h7` on the exact `STM32H723VGT6` device profile;
2. implement it on ESP32-S3 where capability exists;
3. implement the deterministic `sim` backend and, independently, the Linux integration backend where a real Linux mapping exists;
4. write at least one real portable consumer;
5. run contract conformance tests;
6. review semantics for blocking, ISR safety, buffer lifetime, errors and post-failure state;
7. inspect generated assembly for hot paths;
8. measure flash/RAM impact on both embedded targets.

A contract should not be added because a single target datasheet exposes a feature. This directly adopts the multi-platform proof rule recommended by embedded-hal.

Breaking changes before V1 should be encouraged when evidence reveals a poor semantic boundary. After V1, use semantic versioning and keep target-specific APIs separately versionable where practical.
## 28. Testing strategy

### 28.1 Compile-time conformance

Every concrete backend has static assertions:

```cpp
static_assert(hal::i2c::Controller7<stm32h7::I2c<stm32h7::device::stm32h723vgt6, 1>>);
static_assert(hal::i2c::Controller7<esp32s3::I2c0>);
static_assert(hal::i2c::Controller7<sim::I2cController>);
static_assert(hal::i2c::Controller7<linux::I2cController>); // integration build only
```

This proves shape, not semantics.

### 28.2 Portable semantic conformance suites

Write reusable test algorithms for every contract. Examples:

- GPIO output toggles only the target line and logical readback behaves as documented.
- I2C operation sequence generates required repeated starts.
- SPI device transaction never releases CS between operations and always flushes before deassertion.
- CAN-FD payload lengths and classical fallback are correct.
- block-device out-of-range/alignment errors are deterministic.
- NV recovery chooses only committed records after injected power loss.
- RTC monotonic/persistence claims match reset tests.

### 28.3 Host tests

Compile portable drivers natively with aggressive warnings and sanitizers. The simulator should support fault injection:

```text
I2C NACK on operation N
SPI timeout after byte N
CAN bus-off
SD media removed
flash power loss during commit
Ethernet link down
serial framing error
watchdog expiration
```

### 28.4 Hardware-in-the-loop

At minimum:

- GPIO output wired to input;
- UART loopback;
- known I2C target;
- known SPI target/loopback arrangement;
- CAN transceiver and peer/interface;
- SD card known test image;
- Ethernet PHY loop/network test;
- RTC reset/deep-sleep tests;
- watchdog reset-reason verification;
- STM32 D-cache enabled during DMA-heavy tests.

### 28.5 Static analysis/build matrix

Recommended CI configurations:

```text
GCC ARM embedded: -O0, -Og, -O2, -Os
Clang host:       ASan + UBSan
GCC/Clang host:   warnings-as-errors
-fno-exceptions -fno-rtti
LTO build and non-LTO build
STM32 cache enabled
ESP strict static profile
```

Use map files and `size` checks to detect RAM/flash regressions. Add a link-time test that no forbidden global allocation symbols are referenced from the strict HAL library.
## 29. Build and coding policy

The canonical build path is the CMake preset/workflow described in Section 6. Production firmware is built from an optimized release preset into a persistent `.build/` binary tree so normal rebuilds remain incremental. `.elf`, `.bin`, `.hex` and recommended `.map` outputs remain inside that build tree. Debug/sanitizer presets are separate workflows and do not change the portable HAL API.

### 29.1 Supported toolchain/dependency baseline must be pinned

The architecture requires C++20, but reproducibility requires exact tested versions rather than an abstract language label. Before Milestone 1 becomes authoritative CI, the superproject must pin/version-control at least:

- CMake and preferred generator/Ninja baseline;
- ARM embedded compiler/binutils and C++ runtime/library baseline for STM32;
- STM32 CMSIS device definitions and any Cube HAL/LL component versions actually used;
- ESP-IDF/toolchain version for the ESP32-S3 backend;
- Linux host GCC/Clang baseline used by `hal-target-linux` and `hal-sim`;
- FreeRTOS kernel version used by the ECU template.

Store these choices in version-controlled presets/toolchain files/submodule commits (and optionally a human-readable dependency manifest). CI must build the pinned configuration. Upgrades are explicit changes with conformance/regression testing. Do not guess feature availability from “C++20” alone because freestanding standard-library support differs by toolchain.

Recommended strict-target compile profile (toolchain-specific flags need verification):

```text
C++20
-fno-exceptions
-fno-rtti
-Wall -Wextra -Wpedantic
-Wconversion -Wsign-conversion (adopt gradually with clean baseline)
-ffunction-sections -fdata-sections
linker --gc-sections
```

The startup policy is now explicit, but do not globally add flags such as `-fno-threadsafe-statics` merely as a style preference. Enable such flags only if the pinned toolchain/runtime and all code paths are verified not to require guarded function-local static initialization. Critical hardware startup should use explicit BSP/composition initialization rather than hidden function-local static guards.

Coding rules:

- use fixed-width integers at hardware boundaries;
- no implicit unit conversions; use strong wrappers (`hertz`, `nanoseconds`, etc.);
- `noexcept` for operations that must not throw;
- no ownership by raw pointer; raw references/pointers are non-owning and documented;
- no destructor that performs an unbounded hardware wait;
- no blocking operation without documented maximum/timeout policy;
- portable device drivers and application code do not access target registers directly; target backends and BSPs necessarily do;
- every target erratum workaround links to its source document.
## 30. Open design questions intentionally deferred

These are **not** unresolved mistakes; they are areas where freezing an API before real consumers would be premature.

1. **Async API:** coroutine, explicit poll/future, callback-free event handles, or RTOS adapters. Keep separate from sync V1.
2. **Advanced ADC acquisition:** multi-channel scan ownership, zero-copy DMA blocks, synchronized triggers and threshold events remain target-specific until concrete consumers justify stronger portable refinements.
3. **CAN timestamps:** standardize only if time-synchronization consumers need them.
4. **Ethernet zero-copy:** requires explicit DMA/cache buffer ownership and is target-sensitive.
5. **Serial packet/DMA API:** byte stream first; add framed/async layers later.
6. **Input capture / encoder / pulse counting:** separate purpose concepts, not generic Timer methods.
7. **Power management:** clock gating/sleep wake capabilities need a broader system-resource architecture and should not be hidden inside peripheral calls.
8. **Multicore ESP32-S3 ownership:** core affinity/critical-section policy belongs to target/runtime integration, not the semantic peripheral contract.
9. **Memory regions:** a future compile-time `dma_buffer`/memory-region policy may be worthwhile, especially on Cortex-M7.
10. **BSP schema evolution:** a project-owned `board.yaml`/equivalent descriptor is selected initially; keep the schema small and migrate only if real boards prove that a richer format (for example Devicetree) provides enough benefit.
## 31. Implementation roadmap

### Milestone 0 — architecture enforcement and implementation-readiness gate

- establish the superproject/package structure: one `ecu-drivers` package containing the portable
  contracts, target packages, external-device drivers, and `hal-sim`, plus the BSP, composition,
  application `services/` + `tasks/`, and tests;
- retire/reconcile legacy architecture documents, CMake helpers and tests that still encode POSIX/old target sets or nested backend ownership so there is one authoritative package/target model;
- establish checked-in CMake presets/toolchains, strict C++20/no-RTTI/no-exception builds, optimized release workflow and `.build/` output policy;
- pin the exact toolchain, FreeRTOS, CMSIS/vendor-component and host-compiler baselines used by CI;
- establish BSP schema v0 plus exact-device-to-BSP validation/generation into `.build/.../generated/`;
- establish the explicit startup/lifetime sequence and target/BSP initialization API;
- establish ISR-service -> BSP/composition -> FreeRTOS notification integration without FreeRTOS dependencies in the HAL;
- establish firmware/linker/memory ownership rules and the first STM32H723VGT6 linker/startup configuration;
- establish project-wide timeout/failure/fatal-policy conventions;
- implement the ESP32-S3 `strict` versus optional `idf-hosted` build profiles and CI/link deny-list so the strict no-allocation claim becomes mechanically enforceable;
- establish `hal::span`, `hal::result`, units;
- add “portable headers compile without vendor SDK” CI job.

### Milestone 1 — GPIO + time

- `hal-target-stm32h7` exact `STM32H723VGT6` device headers and pin profile;
- ESP32-S3 N16R8 module metadata;
- deterministic `sim` backend plus Linux host-integration skeleton;
- `OutputPin`, `InputPin`, monotonic clock;
- when `EdgeInput` is added, verify atomic `take_event()` consume semantics under ISR/task races and explicitly test coalescing behavior;
- conformance tests.

### Milestone 2 — I2C + first portable sensor

- freeze the I2C progress/timeout policy and post-timeout bus-recovery semantics before declaring `Controller7` V1-stable;
- transaction semantics on STM32H7/STM32H723VGT6, ESP32-S3, `sim`, and Linux i2c-dev integration where available;
- BME280 or similar real sensor driver;
- scripted/stateful `sim` I2C tests; Linux i2c-dev integration test separately;
- repeated-start HIL verification.

### Milestone 3 — SPI + two shared devices

- `hal::spi::Bus8` plus the statically composed `hal::spi::StaticDevice8` implementation in `hal-core`;
- freeze bounded BUSY/flush progress semantics and document any configured timeout/abort policy;
- one display + one flash device on same bus with different configs and explicit CS setup/hold timing where required;
- validate that `operation8::delay_for()` keeps CS asserted and is not confused with CS setup/hold timing;
- validate no interleaving and frequency rounding semantics.

### Milestone 4 — serial, CAN, CAN FD

- freeze serial wait/progress semantics before V1 stabilization;
  - implement/test the generic nonblocking `hal::serial::TextWriter` in `hal-core`; keep
  formatting/logging policy above it;
- establish `CanService` as the normal ECU application-facing owner/coordinator of the CAN controller, with `CanTask` depending on that semantic service rather than target HAL types;
- validate CAN/CAN-FD frame construction/DLC rules and portable filter matching semantics before backend optimization;
- STM32 FDCAN and ESP TWAI backends;
- Linux SocketCAN/vcan integration tests plus deterministic `sim` CAN tests;
- compile-time proof that ESP on-chip controller satisfies `ClassicController` but not `FdController`.

### Milestone 5 — storage/NV

- freeze block-device and NV synchronization/progress bounds before V1 stabilization;
- `block::Device` on file and SD;
- raw `NorFlash` backend using the intentionally simple uniform V1 geometry; add heterogeneous-region refinements only if a real supported device cannot provide a useful uniform view;
- power-loss-test `hal-core` `JournalMemory` against the documented atomic-write, ordered-prefix
  recovery, mount and `sync()` durability guarantees;
- fuzz/fault-injection recovery tests including interrupted compaction and writes at/above `max_atomic_write_size`.

### Milestone 6 — ADC/PWM/RTC/watchdog

- implement `Channel::read_raw()`/`read_voltage()` plus the optional loss-aware `ContinuousChannel` refinement;
- implement allocation-free `BlockAverage` and `MovingAverage` utilities while keeping hardware oversampling in target/BSP configuration;
- verify PWM safe-disabled initialization plus explicit enable/disable lifecycle;
- verify each concrete RTC type's static persistence claims against its selected clock policy and BSP backup-power topology;
- freeze any hardware-dependent ADC conversion/progress bound before V1 stabilization;
- keep complex scanning/motor-control/task-watchdog features target-specific until consumers prove portable requirements.

### Milestone 7 — Ethernet

- Linux TAP `FramePort` first;
- STM32 MAC + chosen external PHY;
- ESP external SPI Ethernet controller;
- common network stack test over both.

### Milestone 8 — generator hardening

- SVD ingestion/validation;
- package/module overlays;
- generated docs/capability reports;
- reproducibility checks and source hashes.
## 32. Example of the intended final usage

The board composition is target-specific; the application and portable drivers are not.

```cpp
int main()
{
    auto board = board::initialize();

    Bme280 sensor{board.sensor_i2c};
    W25q128 flash{board.flash_spi};
    DataLogger logger{board.data_storage};

    while (true) {
        auto sample = sensor.read();
        if (sample) {
            auto logged = logger.append(sample.value());
            if (!logged) {
                // Application policy: report/degrade/retry as appropriate.
            }
        }

        board.delay.delay_for(hal::nanoseconds{100'000'000});

        auto fed = board.watchdog.feed();
        if (!fed) {
            // Product-level policy decides whether this is recoverable or fatal.
        }
    }
}
```

Conceptually, for different builds:

```text
STM32 build:
  board.sensor_i2c  -> stm32h7::I2c<device::stm32h723vgt6, 1, ...>
  board.flash_spi   -> StaticSpiDevice<stm32h7::Spi<device::stm32h723vgt6, 2>, ...>

ESP build:
  board.sensor_i2c  -> esp32s3::I2c<0,...>
  board.flash_spi   -> StaticSpiDevice<esp32s3::GpSpi<2>, ...>

Simulation test:
  board.sensor_i2c  -> sim::I2cController
  board.flash_spi   -> sim::SpiDevice

Linux integration build:
  board.sensor_i2c  -> linux::I2cController
  board.flash_spi   -> linux::SpiDevice
```

The portable source does not contain an `#ifdef STM32`, `#ifdef ESP32`, virtual call, or heap allocation to select those behaviors.
## 33. Authoritative portable contract headers

The focused `ecu-drivers/include/hal/<capability>.hpp` headers are the source of truth for portable C++20
contract declarations. They are versioned and compiled directly; this architecture document does not
embed a second complete copy.

Rules:

1. semantic changes are first described/reviewed in the relevant architecture section;
2. the actual C++ declaration is changed once, in its owning capability header;
3. conformance tests compile against the owning header;
4. documentation may contain small illustrative usage snippets or pseudocode, but no second canonical contract implementation;
5. CI must syntax-check the header with the pinned C++20 embedded/host baselines and `-fno-exceptions -fno-rtti` where applicable.

This removes a demonstrated source-drift failure mode: the architecture previously duplicated the same contracts in individual sections, in this section, and in the standalone header.

## 34. Final architectural rules to put in `CONTRIBUTING.md`

1. **Portable APIs describe semantics, not registers.**
2. **No portable contract is accepted based on one MCU.** Require two materially different implementations, Linux/fake when possible, and at least one generic consumer.
3. **No virtual functions or manual runtime API tables in the strict HAL profile.**
4. **No hidden allocation.** Variable-capacity memory is caller-supplied or statically sized.
5. **No vendor types in portable headers.**
6. **No `#ifdef TARGET` in portable device drivers.**
7. **A type that satisfies a core concept must actually implement the complete semantic contract.**
8. **Optional hardware capability becomes a refinement concept, adapter, or target extension — not a forest of `supports_x()` runtime checks.**
9. **Board code owns pinmux, clocks, DMA-resource assignment and physical wiring.**
10. **DMA and interrupts are mechanisms, not automatically public abstractions.**
11. **No generic Timer.** Expose purposes: clock, alarm, PWM, capture, encoder, etc.
12. **SPI device drivers depend on `hal::spi::Device8`, normally supplied by `hal::spi::StaticDevice8`, not raw bus + manually controlled CS, unless the device genuinely has unusual signaling.**
13. **I2C transactions preserve repeated-start semantics exactly.**
14. **Raw flash is not EEPROM.** Keep `NorFlash` and logical `Memory` separate.
15. **Ethernet application code consumes complete L2 frame ports; MAC/PHY are lower-level composition contracts.**
16. **CAN FD is a refinement of classical CAN.** ESP32-S3 on-chip TWAI must fail the FD concept at compile time.
17. **Every driver contract documents blocking behavior, ISR/thread rules, buffer lifetime, timeout behavior, configuration rounding, error meaning and post-failure state.**
18. **Every DMA-capable STM32H7 backend used by the STM32H723VGT6 profile is tested with D-cache enabled.**
19. **Every target-specific workaround cites a vendor erratum/manual.**
20. **Optimize only after inspecting generated code and measuring.** Static dispatch permits zero-cost paths, but abstraction correctness comes first.
21. **FreeRTOS stays above the HAL.** Queues/tasks may organize application services and driver ownership, but portable contracts and target driver object models do not depend on FreeRTOS.
22. **The firmware superproject owns the `ecu-drivers` submodule version.** Core contracts, target
    packages, and external-device packages are logical components inside that dependency.
23. **Family target packages own exact-device/module facts; BSPs own PCB wiring.** Firmware policy owns operating/resource choices.
24. **All disposable build output lives under `.build/`.** Release embedded builds emit at least `.elf`, `.bin`, and `.hex`; normal development builds are incremental.
25. **Each `ecu-drivers/include/hal/<capability>.hpp` file owns its C++ contract.** Architecture prose
    defines semantics; do not duplicate complete declarations in Markdown.
26. **Transaction descriptors are factory-built and immutable.** I2C/SPI callers do not construct raw direction/kind/pointer/length aggregates; span invariants are established before dereference.
27. **ESP strict-static conformance is not a prose claim.** It is valid only when the strict build profile and CI/link allocation checks pass.
28. **There is one authoritative architecture baseline.** Legacy POSIX/old-target/nested-backend documents and build assumptions must be removed or reconciled; they do not define a parallel architecture.
29. **Initialization is explicit and ordered.** Constructors bind resources; fallible hardware bring-up reports errors to BSP/composition. Avoid hidden hardware startup from global/function-local static constructors.
30. **Target drivers service hardware interrupts; BSP/composition bridges events to FreeRTOS.** HAL packages do not call FreeRTOS simply to notify tasks.
31. **No externally dependent operation may wait forever.** Every blocking contract defines a bounded timeout/deadline/progress policy and post-timeout state.
32. **Runtime error, startup failure and invariant violation are separate categories.** Drivers report; product-level reset/safe-state policy stays above the HAL.
33. **Target facts, BSP physical facts and firmware memory/linker policy remain separate.** Product boot offsets and task/buffer placement are not immutable MCU metadata.
34. **The supported toolchain/dependency matrix is pinned and version-controlled.** CI builds the pinned baseline; upgrades are explicit validated changes.
35. **BSP schema v0 is small, versioned and validated against exact target facts.** Grow it from real boards, not speculative universality.
36. **Interrupt notification semantics are explicit.** `EdgeInput` is a coalescing notification latch with atomic consume semantics; exact edge counting/timestamps require a stronger refinement.
37. **Portable CAN frames are valid by construction and filter matching has one software-defined semantic.** Target register filter layouts must not leak into the portable model.
38. **SPI device state belongs to the concrete adapter, not the concept.** In-transaction delay is distinct from CS setup/hold timing.
39. **Logical NV durability claims are testable.** Document atomic write size, mount/recovery, ordered power-loss behavior and the precise `sync()` durability boundary.
40. **PWM configuration does not implicitly energize an output.** Safe-disabled startup and explicit enable/disable are part of the portable lifecycle.
41. **Static RTC persistence is a fact of the fully configured concrete type/BSP composition, not a generic MCU-family promise.**
42. **ESP32 strict-static conformance is mechanically audited.** Allocation-prone IDF APIs are forbidden or explicitly reviewed; hosted-IDF builds are a separate profile.
## 35. Primary reference index

The architecture above is grounded primarily in the following project/vendor/kernel documentation. Versioned vendor reference manuals and errata should be pinned in the repository when implementation begins.

<a id="ref-r01"></a>**[R01] embedded-hal 0.2 -> 1.0 migration/design rationale.** <https://github.com/rust-embedded/embedded-hal/blob/master/docs/migrating-from-0.2-to-1.0.md>

<a id="ref-r02"></a>**[R02] embedded-hal: how to add a new trait.** <https://github.com/rust-embedded/embedded-hal/blob/master/docs/how-to-add-a-new-trait.md>

<a id="ref-r03"></a>**[R03] embedded-hal I2C contract.** <https://docs.rs/embedded-hal/latest/embedded_hal/i2c/trait.I2c.html>

<a id="ref-r04"></a>**[R04] embedded-hal SPI architecture and contracts.** <https://docs.rs/embedded-hal/latest/embedded_hal/spi/index.html>

<a id="ref-r05"></a>**[R05] embedded-storage NorFlash contract.** <https://docs.rs/embedded-storage/latest/embedded_storage/nor_flash/trait.NorFlash.html>

<a id="ref-r06"></a>**[R06] modm: How modm works.** <https://modm.io/how-modm-works/>

<a id="ref-r07"></a>**[R07] libhal v5 Fundamentals.** <https://libhal.github.io/5.0/user_guide/fundamentals/>

<a id="ref-r08"></a>**[R08] Zephyr Device Driver Model.** <https://docs.zephyrproject.org/latest/kernel/drivers/index.html>

<a id="ref-r09"></a>**[R09] Zephyr Devicetree.** <https://docs.zephyrproject.org/latest/build/dts/index.html>

<a id="ref-r10"></a>**[R10] CMSIS-Driver overview.** <https://arm-software.github.io/CMSIS_6/main/Driver/index.html>

<a id="ref-r11"></a>**[R11] CMSIS-Driver Theory of Operation.** <https://arm-software.github.io/CMSIS_6/v6.0.0/Driver/theoryOperation.html>

<a id="ref-r12"></a>**[R12] CMSIS-Driver CAN interface/capabilities.** <https://arm-software.github.io/CMSIS_6/main/Driver/group__can__interface__gr.html>

<a id="ref-r13"></a>**[R13] RIOT SPI peripheral API.** <https://api.riot-os.org/group__drivers__periph__spi.html>

<a id="ref-r14"></a>**[R14] TinyUSB Architecture.** <https://docs.tinyusb.org/en/latest/reference/architecture.html>

<a id="ref-r15"></a>**[R15] C++20 constraints and concepts.** <https://en.cppreference.com/cpp/language/constraints>

<a id="ref-r16"></a>**[R16] C++ freestanding implementations.** <https://en.cppreference.com/cpp/freestanding>

<a id="ref-r17"></a>**[R17] std::span reference.** <https://en.cppreference.com/w/cpp/container/span>

<a id="ref-r18"></a>**[R18] std::expected reference (C++23).** <https://en.cppreference.com/w/cpp/utility/expected>

<a id="ref-r19"></a>**[R19] WG21 P2279R0: C++ customization mechanisms.** <https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2021/p2279r0.html>

<a id="ref-r20"></a>**[R20] WG21 P2300R10.** <https://www9.open-std.org/JTC1/SC22/WG21/docs/papers/2024/p2300r10.html>

<a id="ref-r21"></a>**[R21] Open-CMSIS-SVD specification.** <https://open-cmsis-pack.github.io/svd-spec/latest/index.html>

<a id="ref-r22"></a>**[R22] AUTOSAR C++14 safety guidelines.** <https://www.autosar.org/fileadmin/standards/R22-11/AP/AUTOSAR_RS_CPP14Guidelines.pdf>

<a id="ref-r23"></a>**[R23] STM32H723VG product page (includes STM32H723VGT6).** <https://www.st.com/en/microcontrollers-microprocessors/stm32h723vg>

<a id="ref-r24"></a>**[R24] STM32H723/733 documentation hub: RM0468, DS13313, errata.** <https://www.st.com/en/microcontrollers-microprocessors/stm32h723-733/documentation.html>

<a id="ref-r25"></a>**[R25] ST Ethernet overview: STM32 10/100 MAC, MII/RMII, IEEE 1588.** <https://www.st.com/en/applications/connectivity/ethernet.html>

<a id="ref-r26"></a>**[R26] ST AN4891 / STM32H72x-H73x system architecture and caches.** <https://www.st.com/resource/en/application_note/dm00306681-stm32h74x-and-stm32h75x-system-architecture-and-performance-expansion-package-for-stm32cube-stmicroelectronics.pdf>

<a id="ref-r27"></a>**[R27] ST AN4839: Cortex-M7 L1 cache and DMA coherency.** <https://www.st.com/resource/en/application_note/an4839-level-1-cache-on-stm32f7-series-and-stm32h7-series-stmicroelectronics.pdf>

<a id="ref-r28"></a>**[R28] ESP32-S3-WROOM-1/WROOM-1U datasheet.** <https://www.espressif.com/sites/default/files/documentation/esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf>

<a id="ref-r29"></a>**[R29] ESP32-S3 Series datasheet.** <https://documentation.espressif.com/esp32_s3_datasheet_en.pdf>

<a id="ref-r30"></a>**[R30] ESP-IDF ESP32-S3 TWAI documentation.** <https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/twai.html>

<a id="ref-r31"></a>**[R31] ESP-IDF ESP32-S3 System Time.** <https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/system/system_time.html>

<a id="ref-r32"></a>**[R32] ESP-IDF ESP32-S3 Watchdogs.** <https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/system/wdts.html>

<a id="ref-r33"></a>**[R33] ESP-IDF ESP32-S3 system API: no integrated Ethernet MAC.** <https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/misc_system_api.html>

<a id="ref-r34"></a>**[R34] ESP-IDF ESP32-S3 SDMMC host.** <https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/sdmmc_host.html>

<a id="ref-r35"></a>**[R35] ESP-IDF ESP32-S3 ADC continuous mode.** <https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/adc/adc_continuous.html>

<a id="ref-r36"></a>**[R36] ESP-IDF NVS documentation and RAM footprint.** <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/storage/nvs_flash.html>

<a id="ref-r37"></a>**[R37] ESP-IDF ESP32-S3 GPTimer.** <https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/gptimer.html>

<a id="ref-r38"></a>**[R38] Linux SocketCAN documentation.** <https://docs.kernel.org/networking/can.html>

<a id="ref-r39"></a>**[R39] Linux TUN/TAP documentation.** <https://docs.kernel.org/networking/tuntap.html>

<a id="ref-r40"></a>**[R40] Linux GPIO character-device v2 API.** <https://docs.kernel.org/userspace-api/gpio/chardev.html>

<a id="ref-r41"></a>**[R41] Linux watchdog userspace API.** <https://docs.kernel.org/watchdog/watchdog-api.html>

<a id="ref-r42"></a>**[R42] Linux `clock_gettime(2)` manual.** <https://man7.org/linux/man-pages/man2/clock_gettime.2.html>

<a id="ref-r43"></a>**[R43] Linux pread/pwrite manual.** <https://man7.org/linux/man-pages/man2/pread.2.html>

<a id="ref-r44"></a>**[R44] NXP UM10204 I2C-bus specification and user manual.** <https://www.nxp.com/docs/en/user-guide/UM10204.pdf>

<a id="ref-r45"></a>**[R45] ESP-IDF Ethernet documentation.** <https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/network/esp_eth.html>

<a id="ref-r46"></a>**[R46] ESP32-S3 ADC/GPIO hardware-design guidance.** <https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32s3/schematic-checklist.html>

<a id="ref-r47"></a>**[R47] STM32H7 Cube repository.** <https://github.com/STMicroelectronics/STM32CubeH7>

<a id="ref-r48"></a>**[R48] ESP-IDF repository.** <https://github.com/espressif/esp-idf>

<a id="ref-r49"></a>**[R49] Linux spidev userspace API.** <https://docs.kernel.org/spi/spidev.html>

<a id="ref-r50"></a>**[R50] Linux I2C userspace interface.** <https://docs.kernel.org/i2c/dev-interface.html>

<a id="ref-r51"></a>**[R51] Linux Industrial I/O buffered userspace interface.** <https://docs.kernel.org/iio/iio_devbuf.html>

<a id="ref-r52"></a>**[R52] Linux PWM interface, including userspace sysfs controls.** <https://docs.kernel.org/6.8/driver-api/pwm.html>

<a id="ref-r53"></a>**[R53] Linux RTC userspace interfaces (`/dev/rtcN`, sysfs).** <https://docs.kernel.org/6.5/admin-guide/rtc.html>

<a id="ref-r54"></a>**[R54] Linux `termios(3)` serial/tty interface.** <https://man7.org/linux/man-pages/man3/termios.3.html>

<a id="ref-r55"></a>**[R55] CMake Presets manual.** <https://cmake.org/cmake/help/latest/manual/cmake-presets.7.html>

<a id="ref-r56"></a>**[R56] CMake command-line build/workflow manual (`cmake --build`, `cmake --workflow`).** <https://cmake.org/cmake/help/latest/manual/cmake.1.html>

<a id="ref-r57"></a>**[R57] FreeRTOS Reference Manual — static allocation and static queue APIs.** <https://www.freertos.org/media/2018/FreeRTOS_Reference_Manual_V10.0.0.pdf>

---

### Document maintenance policy

This file should evolve as an Architecture Decision Record (ADR-like specification), not as a static essay. When a real backend or portable driver proves a contract wrong, update the contract, record the evidence, update the conformance tests, and preserve the rationale in version control. Before the first stable release, correctness across materially different targets is more valuable than preserving an elegant but untested abstraction.
