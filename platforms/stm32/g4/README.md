# STM32G4 platform

This package contains the direct-register STM32G4 implementation. Exact
device and package facts are supplied separately by the selected chip profile.

The implementation uses CMSIS device definitions only at the exact binding
boundary. Clock-tree setup, GPIO alternate-function selection, DMA channel
allocation, interrupt dispatch, and DMA-buffer placement remain BSP policy.

Implemented capabilities include GPIO, timer time, USART/UART, SPI, I2C,
ADC acquisition, classic/FD CAN, internal flash, timer PWM, RTC, and IWDG.
The selected STM32G4 package profile determines whether Ethernet or SDMMC are
available; this family implementation does not assume either controller.

DMA uses the G4 channel/DMAMUX architecture. DMA buffers must be placed in
DMA-accessible SRAM; CCM SRAM is CPU-only.

See [System Architecture](docs/System_Architecture.md) for the driver-level
implementation and ownership model.
