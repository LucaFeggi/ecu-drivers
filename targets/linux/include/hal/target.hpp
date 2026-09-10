#pragma once

// Single include for the Linux HAL target. Individual headers remain available
// for builds that want to keep compile time and dependency surface minimal.
#include <hal/linux/adc.hpp>
#include <hal/linux/block.hpp>
#include <hal/linux/can.hpp>
#include <hal/linux/ethernet.hpp>
#include <hal/linux/gpio.hpp>
#include <hal/linux/i2c.hpp>
#include <hal/linux/nv.hpp>
#include <hal/linux/pwm.hpp>
#include <hal/linux/rtc.hpp>
#include <hal/linux/serial.hpp>
#include <hal/linux/spi.hpp>
#include <hal/linux/support.hpp>
#include <hal/linux/time.hpp>
#include <hal/linux/watchdog.hpp>
