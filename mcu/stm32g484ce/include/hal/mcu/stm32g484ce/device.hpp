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
  static constexpr std::uint8_t dma_controllers{2U};
  static constexpr std::uint8_t dma_channels{16U};
  static constexpr bool data_cache{false};
  static constexpr bool hrtim{true};
};

// The G4 FDCAN instances use fixed, separate message-RAM windows. The
// controller driver still accepts the window pointer explicitly so the BSP
// remains responsible for choosing the instance and mapping policy.
namespace fdcan_message_ram {
inline constexpr std::uintptr_t base{0x4000A400U};
inline constexpr std::size_t words{212U};
inline constexpr std::size_t instance_stride_words{words};

[[nodiscard]] constexpr std::uintptr_t base_for(unsigned instance) noexcept {
  return instance >= 1U && instance <= 3U
             ? base + (static_cast<std::uintptr_t>(instance - 1U) *
                       instance_stride_words * sizeof(std::uint32_t))
             : 0U;
}
} // namespace fdcan_message_ram

} // namespace hal::stm32g4::device::stm32g484ce
