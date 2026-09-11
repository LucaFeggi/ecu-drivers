#pragma once

#include <cstdint>
#include <hal/mcu/esp32s3_wroom_1_n16r8/device.hpp>

namespace hal::esp32s3_wroom_1_n16r8::device {

[[nodiscard]] constexpr bool is_valid_gpio(std::uint8_t pin) noexcept {
  return pin < gpio_count && ((gpio_valid_mask >> pin) & 1U) != 0U;
}

[[nodiscard]] constexpr bool is_output_capable(std::uint8_t pin) noexcept {
  return is_valid_gpio(pin) &&
         ((gpio_output_capable_mask >> pin) & 1U) != 0U;
}

[[nodiscard]] constexpr bool is_input_only(std::uint8_t pin) noexcept {
  return is_valid_gpio(pin) &&
         ((gpio_input_only_mask >> pin) & 1U) != 0U;
}

}  // namespace hal::esp32s3_wroom_1_n16r8::device
