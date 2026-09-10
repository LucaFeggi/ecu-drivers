#pragma once

#include <cstddef>
#include <cstdint>

namespace hal::stm32h5::device::stm32h563vit6 {

struct memory_region {
  std::uintptr_t origin{};
  std::size_t size{};
  bool gpdma_accessible{};
};

inline constexpr memory_region flash{0x08000000U, 2U * 1024U * 1024U, false};
inline constexpr memory_region sram1{0x20000000U, 256U * 1024U, true};
inline constexpr memory_region sram2{0x20040000U, 64U * 1024U, true};
inline constexpr memory_region sram3{0x20050000U, 320U * 1024U, true};
inline constexpr memory_region backup_sram{0x38800000U, 4U * 1024U, false};

} // namespace hal::stm32h5::device::stm32h563vit6
