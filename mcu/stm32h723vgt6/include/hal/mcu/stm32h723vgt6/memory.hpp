#pragma once

#include <cstddef>
#include <cstdint>

namespace hal::stm32h7::device::stm32h723vgt6 {

struct memory_region {
  std::uintptr_t origin{};
  std::size_t size{};
  bool dma_accessible{};
};

inline constexpr memory_region flash{0x08000000U, 1024U * 1024U, false};
inline constexpr memory_region dtcm{0x20000000U, 128U * 1024U, false};
inline constexpr memory_region axi_sram{0x24000000U, 320U * 1024U, true};
inline constexpr memory_region sram1{0x30000000U, 16U * 1024U, true};
inline constexpr memory_region sram2{0x30004000U, 16U * 1024U, true};
inline constexpr memory_region sram4{0x38000000U, 16U * 1024U, false};
inline constexpr std::size_t data_cache_line_bytes{32U};

// This property describes access from DMA1/DMA2, which are the DMA engines
// used by the platform drivers. BDMA has a separate memory-access domain.
[[nodiscard]] constexpr bool dma_accessible(std::uintptr_t address,
                                            std::size_t size) noexcept {
  const auto in = [address, size](memory_region region) constexpr {
    return address >= region.origin && size <= region.size &&
           address - region.origin <= region.size - size &&
           region.dma_accessible;
  };
  return in(axi_sram) || in(sram1) || in(sram2);
}
}  // namespace hal::stm32h7::device::stm32h723vgt6
