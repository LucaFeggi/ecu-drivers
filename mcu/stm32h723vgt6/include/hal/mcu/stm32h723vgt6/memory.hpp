#pragma once

#include <cstddef>
#include <cstdint>

namespace hal::stm32h7::device::stm32h723vgt6 {

struct memory_region {
  std::uintptr_t origin{};
  std::size_t size{};
  bool dma1_accessible{};
};

inline constexpr memory_region flash{0x08000000U, 1024U * 1024U, false};
inline constexpr memory_region dtcm{0x20000000U, 128U * 1024U, false};
inline constexpr memory_region axi_sram{0x24000000U, 320U * 1024U, true};
inline constexpr memory_region sram1{0x30000000U, 16U * 1024U, true};
inline constexpr memory_region sram2{0x30004000U, 16U * 1024U, true};
inline constexpr memory_region sram4{0x38000000U, 16U * 1024U, false};
} // namespace hal::stm32h7::device::stm32h723vgt6
