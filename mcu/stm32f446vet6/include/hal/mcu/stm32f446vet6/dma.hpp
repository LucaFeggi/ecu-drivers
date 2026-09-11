#pragma once

#include <cstddef>
#include <cstdint>

namespace hal::stm32f4::device::stm32f446vet6 {

// STM32F4 DMA controllers expose eight streams each. The stream's channel
// field selects the peripheral request according to the device routing table.
struct dma_stream {
  std::uint8_t controller{};
  std::uint8_t stream{};
  std::uint8_t channel{};
};

inline constexpr std::size_t dma_controller_count{2U};
inline constexpr std::size_t dma_streams_per_controller{8U};

}  // namespace hal::stm32f4::device::stm32f446vet6
