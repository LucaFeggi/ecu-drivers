#pragma once

#include <cstdint>

namespace hal::stm32g4 {

enum class implementation_status : std::uint8_t {
  available,
  planned,
  unsupported
};

struct capability {
  implementation_status status{};
  const char *reason{};

  [[nodiscard]] constexpr bool available() const noexcept {
    return status == implementation_status::available;
  }
};

struct poll_budget {
  std::uint32_t iterations{};

  [[nodiscard]] constexpr bool valid() const noexcept {
    return iterations > 0U;
  }
};

} // namespace hal::stm32g4
