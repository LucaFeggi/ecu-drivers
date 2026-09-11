# hal-target-stm32h5

Direct-register STM32H5 target drivers. Exact device and package facts are
supplied separately by the selected chip profile.

The family layer provides GPIO/EXTI, timers, PWM, RTC, watchdog, ADC
acquisition, USART, SPI, I2C, reduced-IP FDCAN, SDMMC, Ethernet MAC/MDIO, and
internal flash. USART, SPI, I2C, and ADC expose GPDMA paths where the hardware
can move the payload without changing the peripheral transaction boundary.

DMA buffers and message RAM are caller/BSP-owned. The BSP remains responsible
for clock gates, pin alternate functions, GPDMA request routing, interrupt
dispatch, cache maintenance/MPU attributes, and transceiver wiring. No
AliExpress-board BSP is included because the linked listings do not provide a
reliable schematic or complete pin map.

The exact device profile is LQFP100 with 2 MB flash, 640 KB SRAM, two ADCs,
four I2C instances, six SPI instances, two FDCAN instances, two GPDMA
controllers, SDMMC, and Ethernet MAC support. STM32H5 FDCAN uses the reduced
fixed message-RAM layout; it is not configured like STM32H7 FDCAN.

See [System Architecture](docs/System_Architecture.md) for the driver-level
implementation and ownership model.
