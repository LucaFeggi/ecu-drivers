#pragma once

#include <cstdint>

namespace hal::esp32s3::gpio {

struct pin_capabilities {
  std::uint8_t gpio_count{};
  std::uint64_t valid_mask{};
  std::uint64_t output_mask{};

  [[nodiscard]] constexpr bool valid(std::uint8_t pin) const noexcept {
    return pin < gpio_count && ((valid_mask >> pin) & 1U) != 0U;
  }
  [[nodiscard]] constexpr bool output_capable(std::uint8_t pin) const noexcept {
    return valid(pin) && ((output_mask >> pin) & 1U) != 0U;
  }
};

[[nodiscard]] constexpr std::uint64_t bit_range(std::uint8_t first,
                                                std::uint8_t last) noexcept {
  std::uint64_t result = 0U;
  for (std::uint8_t pin = first; pin <= last; ++pin) {
    result |= std::uint64_t{1U} << pin;
  }
  return result;
}

// Conservative default for the currently shipped N16R8 profile: GPIO35..37
// are occupied by octal PSRAM and GPIO46 is input-only. A future exact module
// profile can pass a different value without modifying the family driver.
inline constexpr std::uint64_t supported_valid_mask =
    bit_range(0U, 21U) | bit_range(26U, 34U) | bit_range(38U, 48U);
inline constexpr pin_capabilities supported_module_pins{
    49U, supported_valid_mask,
    supported_valid_mask & ~(std::uint64_t{1U} << 46U)};

} // namespace hal::esp32s3::gpio
