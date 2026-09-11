#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace hal::stm32g4::device::stm32g484ce {

inline constexpr std::string_view name{"STM32G484CE"};
inline constexpr std::string_view package{"LQFP48"};
inline constexpr std::size_t package_pin_count{48U};

struct capabilities {
  static constexpr std::uint8_t adc_instances{5U};
  static constexpr std::uint8_t i2c_instances{4U};
  static constexpr std::uint8_t spi_instances{4U};
  static constexpr std::uint8_t fdcan_instances{3U};
  static constexpr std::uint8_t usart_instances{3U};
  static constexpr std::uint8_t uart_instances{2U};
  static constexpr bool lpuart{true};
  static constexpr bool ethernet_mac{false};
  static constexpr std::uint8_t sdmmc_instances{0U};
  static constexpr bool hrtim{true};
};

}  // namespace hal::stm32g4::device::stm32g484ce
