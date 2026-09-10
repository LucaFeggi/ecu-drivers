#pragma once

#include <cstdint>

namespace hal::stm32h5::device::stm32h563vit6 {

// GPDMA request numbers are deliberately supplied by the BSP. The request
// selector is part of the board/instance routing decision, not a property of
// the reusable family driver.
struct dma_channel {
  std::uint8_t controller{};
  std::uint8_t channel{};
  std::int32_t interrupt{-1};
};

inline constexpr unsigned data_cache_line_bytes{32U};

} // namespace hal::stm32h5::device::stm32h563vit6
