#pragma once

#include <cstdint>

namespace hal::stm32h7::device::stm32h723vgt6 {

namespace dmamux1_request {
inline constexpr std::uint8_t adc1{9U};
inline constexpr std::uint8_t adc2{10U};
inline constexpr std::uint8_t i2c1_rx{33U};
inline constexpr std::uint8_t i2c1_tx{34U};
inline constexpr std::uint8_t spi1_rx{37U};
inline constexpr std::uint8_t spi1_tx{38U};
inline constexpr std::uint8_t usart1_rx{41U};
inline constexpr std::uint8_t usart1_tx{42U};
} // namespace dmamux1_request

inline constexpr unsigned data_cache_line_bytes{32U};

} // namespace hal::stm32h7::device::stm32h723vgt6
