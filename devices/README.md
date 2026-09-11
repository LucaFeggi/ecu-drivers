# External-device drivers

This folder contains portable drivers for specific external devices, such as
sensors, memories, displays, transceivers, and external controllers. These
drivers use the generic contracts in `include/hal` (for example, I2C, SPI, or
GPIO) to communicate with a particular chip or module.

They do not implement MCU peripherals, select board wiring, or depend on a
target package or RTOS. MCU-specific implementations belong in `platforms/`,
while board-specific connections belong in the BSP/composition layer.

## ST7735

`<hal/devices/st7735.hpp>` provides the
`hal::devices::st7735::St7735` controller driver. It consumes an 8-bit SPI
bus, chip-select pin, D/C pin, and delay provider. Command, parameter, and
pixel phases stay under one chip-select assertion, and the driver does not
allocate memory.

`weact_mini_096_hannstar` and `weact_mini_096_configuration` describe the
0.96-inch 160x80 HannStar panel supplied with the WeAct MiniSTM32H723 V1.2.
