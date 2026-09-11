# STM32H5 target architecture

This package is the direct-register STM32H5 family layer. It provides the
selected concrete STM32H5 device target. Reusable code is under
`platforms/stm32/h5/include/hal/`; CMSIS bindings and exact package facts are
provided by a separate chip profile.

CMSIS supplies register/core declarations only. No STM32Cube HAL/LL, RTOS,
heap allocation, exceptions, or virtual dispatch are part of the target. The
BSP owns clock/reset sequencing, alternate functions, GPDMA request/channel
selection, interrupt routing, cache/MPU policy, external PHY/transceiver
wiring, and the placement/coherency of all DMA buffers and message RAM.

## Driver implementations

- **GPIO/EXTI — `include/hal/gpio.hpp`.** `OutputPin` and `InputPin` configure
  H5 GPIO modes, pull, speed, output type, and level. `EdgeInput` configures
  EXTI edge selection and pending state; SYSCFG/pin and vector policy remains
  with the BSP.
- **ADC — `include/hal/adc.hpp`.** `Adc12Gpdma` performs ADC regulator startup,
  calibration, sample-time/resolution/oversampling setup, timer-triggered
  circular sampling, and GPDMA channel movement into caller-owned storage.
  DMA interrupts publish half/complete data through `ContinuousChannel` and
  preserve bounded error/fault reporting.
- **I2C — `include/hal/i2c.hpp`.** `Controller` implements the H5 I2C
  transaction protocol with 7/10-bit addresses, repeated starts, status/error
  handling, and optional GPDMA data movement. A synchronous call waits within
  the configured poll budget; the BSP supplies request routing and channel
  ownership.
- **SPI — `include/hal/spi.hpp`.** `PollingBus8` implements direct register
  exchange, while `DmaBus8` uses paired GPDMA channels for bounded caller-owned
  scratch buffers. Both expose mode, frequency, bit order, read fill, and
  full-duplex bus operations and wait for completion before returning.
- **Serial — `include/hal/serial.hpp`.** `PollingPort` handles USART/UART
  framing and FIFO/register I/O. `DmaPort` adds GPDMA TX/RX buffers, circular
  receive publication, transfer-error state, and ISR service methods without
  introducing an RTOS queue.
- **CAN — `include/hal/can.hpp`.** `Controller` configures H5 FDCAN timing,
  filters, TX/RX FIFOs, and the reduced fixed message-RAM layout. It supports
  classic and CAN-FD frames. This message RAM model is intentionally distinct
  from the STM32H7 implementation and is selected by the BSP-provided window.
- **Ethernet — `include/hal/ethernet.hpp`.** `MdioBus` performs bounded MDIO
  clause transactions; `Phy` composes it with a caller-supplied status decoder.
  `Mac` owns no descriptor storage: the BSP supplies aligned TX/RX descriptor
  rings and buffers. `FramePort` connects MAC link state and PHY state to the
  portable frame contract. Generic PHY speed/duplex decoding is not claimed.
- **Block storage — `include/hal/block.hpp`.** `SdmmcDevice` initializes SD
  cards, decodes CSD geometry, supports block read/write/sync/trim, and can use
  the controller's internal IDMA or an optional DMA stream/DMAMUX boundary.
  Alignment, media, protection, range, and timeout errors are explicit.
- **NOR flash — `include/hal/nv.hpp`.** `NorFlash` unlocks, programs, erases,
  polls, and locks internal H5 flash with compile-time base/capacity/page
  geometry. It validates alignment/ranges and synchronizes instruction/data
  visibility after writes and erases.
- **PWM — `include/hal/pwm.hpp`.** `Output` computes timer period/compare
  values, supports channels, polarity, complementary output/dead time through
  a compare accessor, and reports unrepresentable configurations.
- **RTC — `include/hal/rtc.hpp`.** `Clock` implements BCD calendar access,
  UTC range checks, alarm set/clear, and backup-domain persistence policy.
  Clock-source selection and backup-domain power are BSP-owned.
- **Time — `include/hal/time.hpp`.** `TimerClock` builds a monotonic timer
  from a configured clock/tick relationship, extends overflows atomically, and
  supplies delays. `MicrosecondDelay` is the small calibration/startup adapter.
- **Watchdog — `include/hal/watchdog.hpp`.** `Feeder` computes IWDG
  prescaler/reload from the H5 LSI frequency, starts and feeds the watchdog,
  and maps invalid timing or hardware faults to the portable contract.

The selected package profile supplies memory regions, peripheral counts, DMA
requests, interrupt numbers, pin facts, and cache-line details. Cache
maintenance or MPU configuration must be performed by the BSP. Device-profile
tests are host register-model checks; clock/reset, DMA request, cache, PHY,
SD-card, flash, and board-wiring HIL validation remains required.
