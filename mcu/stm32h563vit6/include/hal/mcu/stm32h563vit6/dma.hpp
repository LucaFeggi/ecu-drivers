#pragma once

#include <cstddef>
#include <cstdint>

namespace hal::stm32h5::device::stm32h563vit6 {

// GPDMA request numbers are deliberately supplied by the BSP. The request
// selector is part of the board/instance routing decision, not a property of
// the reusable family driver.
struct dma_channel {
  std::uint8_t controller{};
  std::uint8_t channel{};
};

inline constexpr std::size_t gpdma_controller_count{2U};
inline constexpr std::size_t gpdma_channels_per_controller{8U};
inline constexpr std::size_t gpdma_channel_count{16U};

}  // namespace hal::stm32h5::device::stm32h563vit6
