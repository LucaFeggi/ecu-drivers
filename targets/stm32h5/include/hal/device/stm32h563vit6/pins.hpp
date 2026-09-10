#pragma once

#include <cstdint>

namespace hal::stm32h5::device::stm32h563vit6 {

enum class port : std::uint8_t { a, b, c, d, e, f, g, h, i };

struct pin {
  port gpio_port{};
  std::uint8_t number{};

  [[nodiscard]] constexpr bool valid() const noexcept { return number < 16U; }
};

} // namespace hal::stm32h5::device::stm32h563vit6
