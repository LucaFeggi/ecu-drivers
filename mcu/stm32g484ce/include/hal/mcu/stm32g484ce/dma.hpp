#pragma once

#include <cstddef>
#include <cstdint>

namespace hal::stm32g4::device::stm32g484ce {

struct dma_channel {
  std::uint8_t controller{};
  std::uint8_t channel{};
};

inline constexpr std::size_t dma_controller_count{2U};
inline constexpr std::size_t dma_channels_per_controller{8U};
inline constexpr std::size_t dma_channel_count{16U};
inline constexpr std::size_t dmamux_channel_count{16U};

// These are immutable peripheral request IDs. The BSP still selects the DMA
// controller and channel for a concrete board configuration.
namespace dmamux_request {
inline constexpr std::uint8_t adc1{5U};
inline constexpr std::uint8_t adc2{36U};
inline constexpr std::uint8_t adc3{37U};
inline constexpr std::uint8_t adc4{38U};
inline constexpr std::uint8_t adc5{39U};
inline constexpr std::uint8_t spi1_rx{10U};
inline constexpr std::uint8_t spi1_tx{11U};
inline constexpr std::uint8_t spi2_rx{12U};
inline constexpr std::uint8_t spi2_tx{13U};
inline constexpr std::uint8_t spi3_rx{14U};
inline constexpr std::uint8_t spi3_tx{15U};
inline constexpr std::uint8_t spi4_rx{44U};
inline constexpr std::uint8_t spi4_tx{45U};
inline constexpr std::uint8_t i2c1_rx{16U};
inline constexpr std::uint8_t i2c1_tx{17U};
inline constexpr std::uint8_t i2c2_rx{18U};
inline constexpr std::uint8_t i2c2_tx{19U};
inline constexpr std::uint8_t i2c3_rx{20U};
inline constexpr std::uint8_t i2c3_tx{21U};
inline constexpr std::uint8_t i2c4_rx{22U};
inline constexpr std::uint8_t i2c4_tx{23U};
inline constexpr std::uint8_t usart1_rx{24U};
inline constexpr std::uint8_t usart1_tx{25U};
inline constexpr std::uint8_t usart2_rx{26U};
inline constexpr std::uint8_t usart2_tx{27U};
inline constexpr std::uint8_t usart3_rx{28U};
inline constexpr std::uint8_t usart3_tx{29U};
inline constexpr std::uint8_t uart4_rx{30U};
inline constexpr std::uint8_t uart4_tx{31U};
inline constexpr std::uint8_t uart5_rx{32U};
inline constexpr std::uint8_t uart5_tx{33U};
inline constexpr std::uint8_t lpuart1_rx{34U};
inline constexpr std::uint8_t lpuart1_tx{35U};
inline constexpr std::uint8_t tim6_up{8U};
inline constexpr std::uint8_t tim7_up{9U};
}  // namespace dmamux_request

}  // namespace hal::stm32g4::device::stm32g484ce
