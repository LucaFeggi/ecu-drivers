# STM32G4 target architecture

This package is the direct-register STM32G4 family layer. It provides the
selected concrete STM32G4 device target. Family implementations are under
`platforms/stm32/g4/include/hal/`; CMSIS bindings and exact package facts are
provided by a separate chip profile.

The target uses CMSIS register declarations only; it does not use STM32Cube
HAL/LL, an RTOS, heap allocation, or runtime polymorphism. Clock trees, GPIO
alternate functions, DMA/DMAMUX request allocation, interrupt dispatch,
FDCAN message-RAM mapping, and board wiring are BSP responsibilities. The
G484CE profile intentionally exposes neither Ethernet MAC nor SDMMC because
the part has neither integrated controller.

## Driver implementations

- **GPIO/EXTI — `include/hal/gpio.hpp`.** `OutputPin` and `InputPin` program
  GPIO mode, pull, speed, output type, and levels. `EdgeInput` owns the
  register-level EXTI routing and pending-edge behavior; the BSP still chooses
  the pin and interrupt vector.
- **ADC — `include/hal/adc.hpp`.** `AdcDma` performs regulator startup,
  calibration, resolution/sample-time setup, optional hardware oversampling,
  timer external triggering, and circular DMA through a G4 DMA channel and
  DMAMUX request. Half/complete interrupts publish into
  `ContinuousChannel`; the acquisition path validates sample period, timer
  limits, ADC clock, and DMA configuration.
- **I2C — `include/hal/i2c.hpp`.** `Controller` implements the G4 I2C transfer
  protocol with 7/10-bit addressing, repeated starts, bounded status/error
  polling, and optional TX/RX DMA channels plus DMAMUX channels. Synchronous
  calls do not return until the programmed operation is complete.
- **SPI — `include/hal/spi.hpp`.** `PollingBus8` and `DmaBus8` implement
  8-bit SPI configuration and full-duplex operations. The DMA variant uses
  paired G4 channels/DMAMUX requests and caller-owned scratch buffers, then
  waits for end-of-transfer before returning.
- **Serial — `include/hal/serial.hpp`.** `PollingPort` provides USART/UART
  framing, baud configuration, bounded FIFO/register reads/writes, and flush.
  `DmaPort` adds circular RX and bounded TX/RX buffers using G4 DMA channels,
  DMAMUX routing, atomic state/error publication, and explicit ISR call-ins.
- **CAN — `include/hal/can.hpp`.** `Controller` configures G4 FDCAN timing,
  filters, TX FIFO, and RX FIFO/message RAM. It supports classic CAN and
  CAN-FD frame contracts, while the BSP supplies the selected instance and its
  fixed message-RAM window.
- **NOR flash — `include/hal/nv.hpp`.** `NorFlash` implements G4 flash page
  programming, status/error clearing, polling, and erase operations. Program
  size, page size, base, and capacity are compile-time parameters; range and
  alignment are checked before modifying flash.
- **PWM — `include/hal/pwm.hpp`.** `Output` derives timer period/compare
  values, supports 16- or 32-bit timers, polarity, complementary channels,
  and optional dead time through a compare accessor. Unrepresentable periods
  and unconfigured operations return explicit errors.
- **RTC — `include/hal/rtc.hpp`.** `Clock` provides BCD calendar read/write,
  UTC validation, alarm configuration/clearing, and backup-domain persistence
  policy. LSE/LSI and backup-domain enabling remain outside the driver.
- **Time — `include/hal/time.hpp`.** `TimerClock` creates a monotonic free-
  running timer using a configured prescaler/tick rate, extends counter
  overflow atomically, and supplies delay; `MicrosecondDelay` is the startup
  adapter used by drivers such as ADC.
- **Watchdog — `include/hal/watchdog.hpp`.** `Feeder` computes and programs
  IWDG prescaler/reload from the supplied LSI frequency, starts/feeds it, and
  reports timeout or hardware faults through the watchdog contract.

The selected package profile supplies package dimensions, memory regions,
peripheral counts, DMA/DMAMUX requests, interrupt numbers, and pin facts. Host
validation uses register models and compile-time profile tests, while hardware
validation remains required for clocking, analog calibration, DMA requests,
FDCAN wiring, flash timing, and interrupt routing.
