#pragma once

#include <cstddef>
#include <cstdint>

namespace hal::stm32f4::device::stm32f446vet6 {

struct memory_region {
  std::uintptr_t origin{};
  std::size_t size{};
  bool dma_accessible{};
};

inline constexpr memory_region flash{0x08000000U, 512U * 1024U, false};
inline constexpr memory_region sram{0x20000000U, 128U * 1024U, true};
inline constexpr memory_region ccm{0x10000000U, 64U * 1024U, false};

} // namespace hal::stm32f4::device::stm32f446vet6
