#pragma once

#include <cstdint>
#include <hal/gpio/pin_capabilities.hpp>
#include <hal/mcu/esp32s3_wroom_1_n16r8/device.hpp>

namespace hal::esp32s3_wroom_1_n16r8::device {

inline constexpr std::uint8_t gpio_first_unavailable{22U};
inline constexpr std::uint8_t gpio_last_unavailable{25U};

constexpr std::uint64_t bit_range(std::uint8_t first,
                                  std::uint8_t last) noexcept {
  return hal::esp32s3::gpio::bit_range(first, last);
}

// N16R8 uses octal PSRAM. GPIO35..GPIO37 are connected internally and are
// deliberately excluded from the board-visible GPIO contract.
inline constexpr std::uint64_t gpio_valid_mask =
    bit_range(0U, 21U) | bit_range(26U, 34U) | bit_range(38U, 48U);
inline constexpr std::uint64_t gpio_output_capable_mask =
    gpio_valid_mask & ~(std::uint64_t{1U} << 46U);
inline constexpr std::uint64_t gpio_input_only_mask = std::uint64_t{1U} << 46U;
inline constexpr auto pin_capabilities =
    hal::esp32s3::gpio::supported_module_pins;

[[nodiscard]] constexpr bool is_valid_gpio(std::uint8_t pin) noexcept {
  return pin_capabilities.valid(pin);
}

[[nodiscard]] constexpr bool is_output_capable(std::uint8_t pin) noexcept {
  return pin_capabilities.output_capable(pin);
}

[[nodiscard]] constexpr bool is_input_only(std::uint8_t pin) noexcept {
  return is_valid_gpio(pin) && ((gpio_input_only_mask >> pin) & 1U) != 0U;
}

}  // namespace hal::esp32s3_wroom_1_n16r8::device
