#pragma once

#include <cstddef>
#include <cstdint>

namespace hal::stm32g4::device::stm32g484ce {

struct memory_region {
  std::uintptr_t origin{};
  std::size_t size{};
  bool dma_accessible{};
};

inline constexpr memory_region flash{0x08000000U, 512U * 1024U, false};
inline constexpr memory_region sram1{0x20000000U, 80U * 1024U, true};
inline constexpr memory_region sram2{0x20014000U, 16U * 1024U, true};
inline constexpr memory_region ccm_sram{0x10000000U, 32U * 1024U, false};
inline constexpr std::size_t data_cache_line_bytes{0U};

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
}  // namespace fdcan_message_ram

[[nodiscard]] constexpr bool dma_accessible(std::uintptr_t address,
                                            std::size_t size) noexcept {
  const auto in = [address, size](memory_region region) constexpr {
    return address >= region.origin && size <= region.size &&
           address - region.origin <= region.size - size &&
           region.dma_accessible;
  };
  return in(sram1) || in(sram2);
}

}  // namespace hal::stm32g4::device::stm32g484ce
