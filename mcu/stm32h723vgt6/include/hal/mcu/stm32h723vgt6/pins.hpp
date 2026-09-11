#pragma once

#include <cstdint>

namespace hal::stm32h7::device::stm32h723vgt6 {

enum class port : std::uint8_t { a, b, c, d, e, h };

struct pin {
  port gpio_port{};
  std::uint8_t number{};

  [[nodiscard]] constexpr bool valid() const noexcept { return number < 16U; }
};

namespace adc1_pin {
inline constexpr pin inp4{port::c, 4U};
inline constexpr std::uint8_t inp4_channel{4U};
}  // namespace adc1_pin

}  // namespace hal::stm32h7::device::stm32h723vgt6
