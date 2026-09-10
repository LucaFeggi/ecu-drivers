#pragma once

#include <cstdint>

namespace hal::stm32f4::device::stm32f446vet6 {

struct dma_stream {
  std::uint8_t controller{};
  std::uint8_t stream{};
  std::uint8_t channel{};
  std::int32_t interrupt{-1};
};

inline constexpr unsigned data_cache_line_bytes{0U};

} // namespace hal::stm32f4::device::stm32f446vet6
