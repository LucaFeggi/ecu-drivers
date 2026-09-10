#pragma once

#include <cstdint>
#include <hal/foundation/assert.hpp>
#include <cstddef>
#include <hal/foundation/span.hpp>

namespace hal {

// IEEE CRC-32 (polynomial 0xEDB88320), implemented without lookup tables so it
// has fixed, small storage on all targets.
[[nodiscard]] constexpr std::uint32_t crc32(span<const std::byte> data) noexcept {
  HAL_CORE_ASSERT(data.valid());
  if (!data.valid()) {
    return 0U;
  }

  std::uint32_t crc = 0xFFFFFFFFU;
  for (const std::byte value : data) {
    crc ^= static_cast<std::uint32_t>(std::to_integer<std::uint8_t>(value));
    for (std::uint8_t bit = 0U; bit < 8U; ++bit) {
      const std::uint32_t mask = 0U - (crc & 1U);
      crc = (crc >> 1U) ^ (0xEDB88320U & mask);
    }
  }
  return ~crc;
}

}  // namespace hal
