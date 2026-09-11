#pragma once

#include <cstdint>

namespace hal::stm32g4::device::stm32g484ce {

enum class port : std::uint8_t { a, b, c, d, f };

struct pin {
  port gpio_port{};
  std::uint8_t number{};

  [[nodiscard]] constexpr bool valid() const noexcept { return number < 16U; }
};

namespace adc1_pin {
inline constexpr pin inp1{port::a, 0U};
inline constexpr std::uint8_t inp1_channel{1U};
inline constexpr pin inp2{port::a, 1U};
inline constexpr std::uint8_t inp2_channel{2U};
inline constexpr pin inp3{port::a, 2U};
inline constexpr std::uint8_t inp3_channel{3U};
inline constexpr pin inp4{port::a, 3U};
inline constexpr std::uint8_t inp4_channel{4U};
}  // namespace adc1_pin

}  // namespace hal::stm32g4::device::stm32g484ce
