#include <hal/adc.hpp>
#include <hal/can.hpp>
#include <hal/gpio.hpp>
#include <hal/i2c.hpp>
#include <hal/nv.hpp>
#include <hal/pwm.hpp>
#include <hal/rtc.hpp>
#include <hal/serial.hpp>
#include <hal/spi.hpp>
#include <hal/time.hpp>
#include <hal/watchdog.hpp>

#if defined(ECU_CONSUMER_LINUX)
#include <hal/block.hpp>
#include <hal/ethernet.hpp>
#elif defined(ECU_CONSUMER_STM32F4)
#include <hal/block.hpp>
#include <hal/mcu/stm32f446vet6/device.hpp>
#elif defined(ECU_CONSUMER_STM32G4)
#include <hal/mcu/stm32g484ce/device.hpp>
#elif defined(ECU_CONSUMER_STM32H5)
#include <hal/block.hpp>
#include <hal/ethernet.hpp>
#include <hal/mcu/stm32h563vit6/device.hpp>
#elif defined(ECU_CONSUMER_STM32H7)
#include <hal/block.hpp>
#include <hal/ethernet.hpp>
#include <hal/mcu/stm32h723vgt6/device.hpp>
#elif defined(ECU_CONSUMER_ESP32S3)
#include <hal/block.hpp>
#include <hal/clock.hpp>
#include <hal/ethernet.hpp>
#include <hal/mcu/esp32s3_wroom_1_n16r8/device.hpp>
#include <hal/memory.hpp>
#else
#error "No installed consumer target was selected"
#endif

int main() { return 0; }
