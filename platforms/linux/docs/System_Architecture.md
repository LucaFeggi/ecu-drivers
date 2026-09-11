# Linux target architecture

This package is the host integration backend for the portable `hal-core`
contracts. It is header-only and uses Linux kernel UAPIs directly; it does not
emulate MCU registers and it does not provide an event loop or an RTOS layer.
The CMake target is `hal::linux` (`platforms/linux/CMakeLists.txt`).

Objects own their file descriptors through the target's small RAII `owned_fd`
helper. Configuration is explicit, storage is caller-owned or bounded inside a
driver, and all operations return `hal::result` with a portable error kind plus
the native `errno` and operation detail. No target driver allocates from the
heap, creates threads, or hides blocking policy.

## Driver implementations

- **GPIO — `include/hal/gpio.hpp`.** `OutputLine`, `InputLine`, and `EdgeLine`
  open Linux GPIO character devices and request GPIO v2 lines. Output writes,
  input reads, active-low configuration, and rising/falling/both-edge event
  reads are exposed through the portable GPIO concepts. Edge events are
  consumed from the line file descriptor; line ownership is released by RAII.
- **I2C — `include/hal/i2c.hpp`.** `Controller` uses `i2c-dev` ioctl
  transactions, checks adapter functionality, and supports both seven- and
  ten-bit addresses. A bounded operation array preserves write/read/repeated-
  start/stop transaction semantics without allocating.
- **SPI — `include/hal/spi.hpp`.** `transfer_engine` configures `spidev` mode,
  bit order, word size, frequency, and per-operation delays. `Bus` is the
  physical 8-bit bus; `Device` binds one configuration and uses bounded
  staging for output-only operations. `hal::spi::StaticDevice8` remains the
  portable adapter when an external chip-select pin is needed.
- **CAN — `include/hal/can.hpp`.** `Controller<false>` exposes SocketCAN
  classic CAN and `Controller<true>` additionally accepts CAN-FD frames. Raw
  SocketCAN identifiers and DLC/length encoding are translated to the
  portable frame types; bounded software filters and nonblocking receive are
  supported.
- **Ethernet — `include/hal/ethernet.hpp`.** `TapFramePort` provides a TAP
  interface and `PacketFramePort` uses an AF_PACKET raw socket. Both expose
  bounded frame transmit/receive and link-state configuration; neither is a
  simulated MAC or PHY.
- **Serial — `include/hal/serial.hpp`.** `Port` configures a nonblocking
  `termios` device, supports the portable data/parity/stop/flow-control
  settings, and implements bounded `try_read`, `try_write`, and `flush`.
- **Block storage — `include/hal/block.hpp`.** `Device` maps regular files and
  block devices to the block contract, validates block geometry and ranges, and
  implements read/write/sync/trim with portable media and protection errors.
- **NOR storage — `include/hal/nv.hpp`.** `FileNorFlash` gives a file-backed
  flash model with compile-time capacity/read/program/erase geometry checks. It
  validates erase/program alignment and enforces flash 1-to-0 programming
  semantics rather than treating the file as arbitrary RAM.
- **ADC — `include/hal/adc.hpp`.** `BufferedChannel` enables an IIO buffered
  channel through sysfs, reads a bounded scan buffer, decodes bit width,
  shift, endianness, scale, and offset, and publishes timestamped raw/voltage
  samples without a background thread.
- **PWM — `include/hal/pwm.hpp`.** `Output` controls a sysfs PWM channel,
  including export ownership, period, duty cycle, polarity, and enable state.
  The implementation verifies that requested nanosecond values fit the exposed
  sysfs attributes.
- **RTC — `include/hal/rtc.hpp`.** `SystemClock` maps UTC time to
  `clock_gettime`/`clock_settime`, `SystemClockAlarm` uses `timerfd`, and
  `HardwareClock` uses Linux RTC ioctls for persistent clock/alarm semantics.
  Civil-date conversion is implemented locally without heap or locale state.
- **Time — `include/hal/time.hpp`.** `MonotonicClock` uses
  `CLOCK_MONOTONIC`, `Delay` uses absolute `clock_nanosleep` deadlines, and
  `OneShotAlarm` uses a nonblocking `timerfd`.
- **Watchdog — `include/hal/watchdog.hpp`.** `Feeder` opens a Linux watchdog,
  sets and validates its timeout, and feeds it with `WDIOC_KEEPALIVE`; optional
  magic-close behavior is explicit in the configuration.

The `detail/` headers centralize descriptor ownership, bounded I/O helpers, and
native-error mapping. They are implementation support, not an additional
portable abstraction layer.

## Ownership and validation

`hal_linux_contract_tests` checks the concepts, while file-device and PTY tests
exercise real Linux integration. The backend deliberately does not claim MCU
MAC/PHY/MDIO behavior, DMA, cache coherency, or hardware timing equivalence.
Linux is an integration backend for real kernel interfaces, not an MCU
peripheral simulator.
