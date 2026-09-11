# External-device drivers

This folder contains portable drivers for specific external devices, such as
sensors, memories, displays, transceivers, and external controllers. These
drivers use the generic contracts in `include/hal` (for example, I2C, SPI, or
GPIO) to communicate with a particular chip or module.

They do not implement MCU peripherals, select board wiring, or depend on a
target package or RTOS. MCU-specific implementations belong in `platforms/`,
while board-specific connections belong in the BSP/composition layer.
