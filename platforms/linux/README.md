# hal-target-linux

Linux UAPI-backed drivers for the portable HAL contracts in `ecu-drivers`.

The target is a header-only, statically allocated C++20 interface. It uses
caller-owned buffers and RAII file-descriptor ownership; it does not allocate
from the heap, create threads, or hide an event loop inside a driver.

Implemented capabilities:

- GPIO character-device v2 lines and edge events
- `i2c-dev` transactions
- `spidev` buses and devices
- SocketCAN classic CAN and CAN FD
- TAP and AF_PACKET Ethernet frame ports
- non-blocking `termios` serial ports
- regular-file and block-device storage
- file-backed NOR-flash semantics
- IIO buffered ADC channels
- sysfs PWM outputs
- system and hardware RTC clocks/alarms
- monotonic time, delay, and one-shot alarms
- Linux watchdog feeding

Include only the driver headers required by each translation unit, for
example `<hal/serial.hpp>` or `<hal/block.hpp>`.

The Linux target deliberately does not provide MCU-only MAC/PHY/MDIO wrappers.

See [System Architecture](docs/System_Architecture.md) for the driver-level
implementation and ownership model.
