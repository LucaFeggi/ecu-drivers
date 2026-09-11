#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace hal::stm32h5::device::stm32h563vit6 {

inline constexpr std::string_view name{"STM32H563VIT6"};
inline constexpr std::string_view package{"LQFP100"};
inline constexpr std::size_t package_pin_count{100U};

struct capabilities {
  static constexpr std::uint8_t adc_instances{2U};
  static constexpr std::uint8_t i2c_instances{4U};
  static constexpr std::uint8_t spi_instances{6U};
  static constexpr std::uint8_t fdcan_instances{2U};
  static constexpr bool ethernet_mac{true};
  static constexpr std::uint8_t sdmmc_instances{2U};
  static constexpr bool adc_timer_triggered_dma{true};
  static constexpr std::uint8_t flash_wait_state_limit{8U};
};

}  // namespace hal::stm32h5::device::stm32h563vit6
