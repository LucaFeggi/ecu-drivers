#pragma once

#include <cstdint>

namespace hal::stm32g4::register_access {

[[nodiscard]] inline std::uint32_t read(volatile std::uint32_t &reg) noexcept {
  return reg;
}

inline void write(volatile std::uint32_t &reg, std::uint32_t value) noexcept {
  reg = value;
}

inline void set(volatile std::uint32_t &reg, std::uint32_t mask) noexcept {
  reg = reg | mask;
}

inline void clear(volatile std::uint32_t &reg, std::uint32_t mask) noexcept {
  reg = reg & ~mask;
}

inline void replace(volatile std::uint32_t &reg, std::uint32_t mask,
                    std::uint32_t value) noexcept {
  reg = (reg & ~mask) | (value & mask);
}

} // namespace hal::stm32g4::register_access
