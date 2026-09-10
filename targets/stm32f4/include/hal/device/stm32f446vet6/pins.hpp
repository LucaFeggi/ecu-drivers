#pragma once

#include <cstdint>

namespace hal::stm32f4::device::stm32f446vet6 {

enum class port : std::uint8_t { a, b, c, d, e, f, g, h, i };

struct pin {
  port gpio_port{};
  std::uint8_t number{};

  [[nodiscard]] constexpr bool valid() const noexcept { return number < 16U; }
};

} // namespace hal::stm32f4::device::stm32f446vet6
