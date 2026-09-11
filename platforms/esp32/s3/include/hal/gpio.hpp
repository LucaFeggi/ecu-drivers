#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/gpio.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif
#include <hal/gpio/edge_input.hpp>
#include <hal/gpio/matrix.hpp>
#include <hal/gpio/pin.hpp>

namespace hal::esp32s3_wroom_1_n16r8::gpio {

using StatefulOutputPin = OutputPin;

}  // namespace hal::esp32s3_wroom_1_n16r8::gpio
