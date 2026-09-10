# STM32H7 target architecture

This package is MCU-family support, not a BSP. Reusable register-level implementations live under
`include/hal/stm32h7/`; exact-part facts live under
`include/hal/stm32h7/device/<part-number>/`. A firmware BSP selects the exact device, pins,
oscillators, peripheral instances, DMA resources, interrupts, and memory placement.

Only CMSIS register/core declarations are consumed from `third_party/`. `USE_HAL_DRIVER` is never
defined and STM32Cube HAL functions are not used. Drivers remain FreeRTOS-independent.

The ADC acquisition boundary is `timer trigger -> ADC1/2 -> circular DMA -> DMA half/complete ISR ->
published channel state`. A concrete channel exposes the current `hal-core` `Channel` contract through
nonblocking `read_raw()` and `read_voltage()` operations; a continuous channel additionally exposes
`ContinuousChannel::try_read()` over caller-provided `hal::adc::raw_sample` storage. The acquisition
engine owns the required ADC, timer, DMAMUX, and DMA register protocol. The BSP selects the instances,
pin/channel, operating rate, interrupt, and caller-owned DMA storage. It also makes that storage
coherent by MPU placement or explicit cache policy. The publisher supports one ISR producer, any
number of channel-read observers, and one serialized stream consumer.

Hardware oversampling remains in the target engine. Physical anti-alias filtering is a board/circuit
fact. Portable `BlockAverage` and `MovingAverage` filters are composed from
`hal/adc/averaging.hpp` after `ContinuousChannel::try_read()`. ADC1/2 calibration is performed before
acquisition; it corrects chip-specific offset and linearity error and is distinct from the target/BSP
conversion that supplies the `Channel::read_voltage()` `hal::adc::voltage_sample` view.

The target also provides polling and DMA data paths for SPI/USART, DMA-capable I2C, FDCAN message-RAM
FIFO access, direct PWM/RTC/IWDG/flash/SDMMC-IDMA implementations, and direct H7 Ethernet MAC/MDIO
descriptor access. DMA storage and descriptor rings are borrowed from the BSP so cacheability and
placement remain explicit. Synchronous `hal-core` operations use DMA for the payload and a bounded
completion check before returning; continuous ADC and receive-ring paths use DMA completion
notifications. A free-running 32-bit timer plus overflow ISR supplies a monotonic clock. The code stays
FreeRTOS-independent; the firmware owns scheduler configuration and IRQ priorities.

The host conformance build covers the target concepts. Hardware-in-the-loop validation remains required
for the selected clock tree, reset behavior, DMA request routing, cache/MPU attributes, external PHY,
SD card, flash execution policy, and board wiring. PTP and generic PHY speed/duplex decoding are not
claimed by the family package; those are explicit extensions or BSP-provided policies.
