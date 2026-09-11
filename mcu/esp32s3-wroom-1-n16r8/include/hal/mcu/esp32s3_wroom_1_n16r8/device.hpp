#pragma once

#include <cstdint>

namespace hal::esp32s3_wroom_1_n16r8::device {

inline constexpr const char name[] = "esp32s3_wroom_1_n16r8";
inline constexpr std::uint32_t flash_bytes = 16U * 1024U * 1024U;
inline constexpr std::uint32_t psram_bytes = 8U * 1024U * 1024U;
inline constexpr std::uint8_t gpio_count = 49U;
inline constexpr std::uint8_t gpio_first_unavailable = 22U;
inline constexpr std::uint8_t gpio_last_unavailable = 25U;

constexpr std::uint64_t bit_range(std::uint8_t first,
                                  std::uint8_t last) noexcept {
  std::uint64_t result = 0U;
  for (std::uint8_t pin = first; pin <= last; ++pin) {
    result |= std::uint64_t{1U} << pin;
  }
  return result;
}

// N16R8 uses octal PSRAM. GPIO35..GPIO37 are connected internally and are
// deliberately excluded from the board-visible GPIO contract.
inline constexpr std::uint64_t gpio_valid_mask =
    bit_range(0U, 21U) | bit_range(26U, 34U) | bit_range(38U, 48U);
inline constexpr std::uint64_t gpio_output_capable_mask =
    gpio_valid_mask & ~(std::uint64_t{1U} << 46U);
inline constexpr std::uint64_t gpio_input_only_mask =
    std::uint64_t{1U} << 46U;

}  // namespace hal::esp32s3_wroom_1_n16r8::device
