# ESP32-S3 target architecture

This package is the direct-register ESP32-S3 target. Its family headers live
under `platforms/esp32/s3/include/hal/`; register bindings and package facts
are supplied separately by the selected chip profile.

ESP-IDF is consumed for raw SoC register structures, addresses, and startup
integration only. The public drivers do not use ESP-IDF peripheral drivers,
handles, FreeRTOS types, or hidden allocation. Clock/reset gates, CPU interrupt
vector installation, GPIO matrix policy, board wiring, and DMA-visible memory
placement remain BSP/startup responsibilities. The selected package profile
supplies flash, PSRAM, GPIO, and memory facts.

## Driver implementations

- **Register/resource support — `register_access.hpp`, `support.hpp`, and the
  chip profile.** `bindings.hpp` maps the ESP-IDF register structs to fixed SoC
  addresses. `clock::Gate` explicitly enables/resets peripheral resources;
  drivers never silently change another peripheral's clock state. The profile
  supplies GPIO validity/output capability, DMA descriptor encoding, interrupt
  source names, and static device counts.
- **DMA — `dma/channel.hpp`.** The shared GDMA `Channel` resets and programs RX
  and TX links, peripheral selection, descriptor addresses, start/stop, and
  interrupt flags. Descriptors are 32-bit-address, caller-owned objects; the
  BSP must provide DMA-visible internal storage when zero-copy behavior is
  required.
- **GPIO — `gpio/pin.hpp`, `gpio/matrix.hpp`, and `gpio/edge_input.hpp`.**
  `OutputPin` and `InputPin` configure GPIO mode, pull, drive, and level.
  Matrix helpers route peripheral signals through the ESP32-S3 GPIO matrix;
  `EdgeInput` adds edge selection and pending-event state. Invalid or
  module-reserved pins are rejected by the exact profile.
- **ADC — `adc/adc.hpp` and `adc/stream.hpp`.** `ChannelReader` performs
  bounded one-shot SAR conversions for ADC unit 1 or 2 and maps raw samples to
  the portable voltage view. `Continuous` configures the digital controller,
  channel/attenuation pattern, sampling rate, GDMA descriptors, and a
  caller-owned sample area; `Stream` publishes bounded timestamped samples for
  a nonblocking consumer. Continuous mode uses DMA; one-shot mode is polled.
- **I2C — `i2c/controller.hpp`.** `Controller` translates portable operations
  into the ESP32-S3 I2C command/FIFO engine, including address phase, repeated
  starts, read/write direction, ACK policy, stop, and bounded completion/error
  polling. The two SoC controllers are selected by the BSP binding.
- **SPI — `spi/dma_bus.hpp`.** `DmaBus8` programs GPSPI2/GPSPI3 clock mode,
  bit order, frequency, and full-duplex exchange. It disables hardware CS so
  `StaticDevice8` owns chip select and transaction boundaries. TX/RX GDMA
  descriptors and aligned scratch buffers are caller-owned; transfers are
  chunked to the bounded staging capacity.
- **Serial — `serial/port.hpp` and `serial/dma_port.hpp`.** `Port` configures
  UART baud, framing, flow control, and FIFO polling. `DmaPort` adds GDMA TX
  transfers with bounded internal staging while retaining FIFO-safe RX
  behavior. UART FIFO overrun, timeout, and configuration failures are mapped
  to the serial contract.
- **CAN — `can/twai.hpp`.** `Controller` configures the TWAI classic-CAN
  timing, acceptance filter, TX mailbox, and RX FIFO. The ESP32-S3 has no
  integrated CAN-FD controller, so only the classic CAN contract is exposed.
- **Ethernet — `ethernet/unsupported.hpp`.** `Unsupported` deliberately
  returns the portable unsupported error. The ESP32-S3 has no integrated
  Ethernet MAC; an external controller belongs in `devices/` over a selected
  bus.
- **PWM — `pwm/ledc.hpp`.** `Output` solves the LEDC timer divider/resolution
  for a requested period, configures one of four timers and eight channels, and
  updates duty/polarity with bounded register operations.
- **RTC — `rtc/rtc.hpp`.** `Clock` uses the RTC counter and slow-clock
  calibration/configuration to provide UTC-style time and alarms. The slow
  clock frequency is a compile-time parameter and alarm clearing is explicit.
- **Time — `time/monotonic_clock.hpp`.** `MonotonicClock` reads SYSTIMER,
  `Delay` performs bounded busy-wait delays against that clock, and
  `OneShotAlarm` programs a SYSTIMER alarm. No task or timer thread is created.
- **Nonvolatile storage — `nv/flash.hpp`.** `Flash` uses the SPI-memory
  controller's mapped flash/read, page-program, sector-erase, and status/wait
  operations. Capacity, 4-byte program size, and 4 KiB erase geometry are
  compile-time checked; writes and erases validate alignment/range.
- **Block storage — `block/sdmmc.hpp`.** `Sdmmc` performs SD-card command
  initialization, CSD geometry decoding, block read/write, sync, and trim over
  the SDMMC host. Data transfers use the controller's internal IDMAC and
  caller-owned buffers/descriptors.
- **Watchdog — `watchdog/timer_group.hpp`.** `Feeder` selects a timer-group
  watchdog prescaler for the requested timeout, handles write protection, and
  exposes start/feed/stop through the watchdog contract.

The device-profile test instantiates every target driver and validates the
exact profile at compile time; it does not dereference the ESP32-S3 address map
on the host. HIL remains required for clock/reset sequencing, GPIO matrix
routing, ADC calibration, GDMA placement/cache policy, flash/PSRAM behavior,
SD-card wiring, and interrupt installation.
