# STM32F4 target architecture

This package is the direct-register STM32F4 family layer. It provides the
selected concrete STM32F4 device target. Reusable driver logic is under
`platforms/stm32/f4/include/hal/`; CMSIS register types and exact package facts
are selected through a separate chip profile.

Only CMSIS device/core declarations are required at the binding boundary. The
drivers do not use STM32Cube HAL/LL, an RTOS, virtual dispatch, exceptions, or
hidden allocation. The BSP owns clocks, alternate-function pin setup, DMA
stream/channel routing, interrupt dispatch, external transceivers, and storage
wiring. STM32F4 has no Ethernet MAC in this package; its CAN implementation is
the classic bxCAN peripheral.

## Driver implementations

- **GPIO/EXTI — `include/hal/gpio.hpp`.** `OutputPin` and `InputPin` program
  STM32 GPIO mode, pull, speed, output type, and level. `EdgeInput` configures
  SYSCFG/EXTI routing and rising/falling/both-edge pending state. Port and pin
  objects are register references selected by the BSP.
- **ADC — `include/hal/adc.hpp`.** `AdcDma` configures one 12-bit ADC channel,
  sample time, circular legacy DMA stream, and continuous conversion into a
  caller-owned `uint16_t` buffer. `ContinuousChannel` publishes the samples
  from the DMA interrupt and exposes nonblocking raw/voltage reads.
- **I2C — `include/hal/i2c.hpp`.** `Controller` implements the STM32F4 I2C
  event/error state machine for 7/10-bit address transactions. TX/RX legacy
  DMA streams are optional template parameters; the synchronous operation
  waits with a bounded poll budget and aborts DMA on failure.
- **SPI — `include/hal/spi.hpp`.** `PollingBus8` handles register/FIFO
  exchange and `DmaBus8` adds paired legacy DMA streams with caller-owned
  scratch storage. Both provide 8-bit bus configuration, full-duplex
  transfer, read fill, flush, and bounded timeout/error reporting.
- **Serial — `include/hal/serial.hpp`.** `PollingPort` configures USART framing
  and performs bounded FIFO/register I/O. `DmaPort` adds a caller-owned TX/RX
  buffer path using the F4 stream model; DMA completion is delivered through
  the BSP's interrupt call-in methods.
- **CAN — `include/hal/can.hpp`.** `Controller` configures classic bxCAN bit
  timing, filters, mailboxes, and RX FIFOs. It translates standard/extended
  identifiers and bounded classic frames; CAN-FD and Ethernet are not exposed.
- **Block storage — `include/hal/block.hpp`.** `SdmmcDevice` drives the F4 SDIO
  command/data protocol, initializes SDHC/SDSC cards, decodes CSD geometry,
  and implements block read/write/sync/trim. Data movement is polling or uses
  an optional legacy DMA stream; buffers must be in DMA-accessible SRAM.
- **NOR flash — `include/hal/nv.hpp`.** `NorFlash` unlocks the internal flash,
  programs 32-bit words, erases the STM32F4 16 KiB-sector geometry, polls
  completion, and maps status failures. Base/capacity are compile-time
  parameters and all program/erase operations validate alignment and range.
- **PWM — `include/hal/pwm.hpp`.** `Output` calculates timer ARR/compare values
  for a requested period and pulse width, supports channels 1..4, polarity,
  complementary output/dead-time policy, and start/stop state.
- **RTC — `include/hal/rtc.hpp`.** `Clock` reads/writes the calendar in BCD,
  validates UTC values, configures alarms, and applies a selectable backup
  domain persistence policy. Clock-source and backup-domain setup remain BSP
  policy.
- **Time — `include/hal/time.hpp`.** `TimerClock` configures a free-running
  timer tick, extends its counter through an atomic overflow value, and
  provides monotonic time and delay. `MicrosecondDelay` adapts that clock for
  peripheral startup/calibration waits.
- **Watchdog — `include/hal/watchdog.hpp`.** `Feeder` computes the IWDG
  prescaler/reload from the BSP-supplied LSI frequency, starts and feeds the
  watchdog, and returns bounded hardware/configuration errors.

The selected package profile supplies flash/SRAM geometry, peripheral counts,
legacy DMA accessibility, and interrupt facts. The device-profile test is a
host register-model/concept check; board clocking, pin mux, DMA routing, SDIO,
flash execution policy, and physical CAN wiring still require HIL validation.
