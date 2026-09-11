# STM32F4 platform

Direct-register STM32F4 target drivers. Exact device bindings and package facts
are supplied separately by the selected chip profile.

The family layer provides GPIO/EXTI, timers, PWM, RTC, watchdog, ADC
acquisition, USART, SPI, I2C, classic bxCAN, SDIO, and internal flash. DMA
paths use the STM32F4 legacy DMA1/DMA2 stream and channel model; no DMAMUX or
FDCAN/Ethernet assumptions are made.

DMA buffers remain caller/BSP-owned. The BSP supplies clock gates, pin
alternate functions, DMA stream/channel routing, interrupt dispatch, and
board-level transceiver/storage wiring. No AliExpress-board BSP is included
because the linked listings do not provide a reliable schematic or complete
pin map.

The exact device profile is LQFP100 with 512 KB flash and 128 KB SRAM. CCM
RAM is represented separately because it is not DMA-accessible.

See [System Architecture](docs/System_Architecture.md) for the driver-level
implementation and ownership model.
