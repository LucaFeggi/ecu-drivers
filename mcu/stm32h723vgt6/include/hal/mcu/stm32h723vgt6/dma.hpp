#pragma once

#include <cstddef>
#include <cstdint>

namespace hal::stm32h7::device::stm32h723vgt6 {

// DMA1 and DMA2 use the legacy stream register model with request selection
// handled by DMAMUX1. Stream/channel assignment remains BSP policy.
struct dma_stream {
  std::uint8_t controller{};
  std::uint8_t stream{};
};

inline constexpr std::size_t dma_controller_count{2U};
inline constexpr std::size_t dma_streams_per_controller{8U};
inline constexpr std::size_t dmamux1_channel_count{16U};

// Only the request IDs consumed by the reusable platform drivers are
// published here. Other routes remain BSP configuration choices.
namespace dmamux1_request {
inline constexpr std::uint8_t adc1{9U};
inline constexpr std::uint8_t adc2{10U};
inline constexpr std::uint8_t i2c1_rx{33U};
inline constexpr std::uint8_t i2c1_tx{34U};
inline constexpr std::uint8_t spi1_rx{37U};
inline constexpr std::uint8_t spi1_tx{38U};
inline constexpr std::uint8_t usart1_rx{41U};
inline constexpr std::uint8_t usart1_tx{42U};
}  // namespace dmamux1_request

}  // namespace hal::stm32h7::device::stm32h723vgt6
