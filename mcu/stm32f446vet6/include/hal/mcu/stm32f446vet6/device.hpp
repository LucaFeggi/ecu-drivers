#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace hal::stm32f4::device::stm32f446vet6 {

inline constexpr std::string_view name{"STM32F446VET6"};
inline constexpr std::string_view package{"LQFP100"};
inline constexpr std::size_t package_pin_count{100U};

struct capabilities {
  static constexpr std::uint8_t adc_instances{3U};
  static constexpr std::uint8_t i2c_instances{3U};
  static constexpr std::uint8_t spi_instances{4U};
  static constexpr std::uint8_t can_instances{2U};
  static constexpr bool ethernet_mac{false};
  static constexpr std::uint8_t sdmmc_instances{1U};
  static constexpr std::uint8_t dma_controllers{2U};
  static constexpr bool adc_timer_triggered_dma{true};
  static constexpr std::size_t flash_bytes{512U * 1024U};
  static constexpr std::size_t sram_bytes{128U * 1024U};
};

} // namespace hal::stm32f4::device::stm32f446vet6
