#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/watchdog.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include <cstddef>
#include <cstdint>
#include "support.hpp"

namespace hal::stm32h5::watchdog {

inline constexpr capability support{
    implementation_status::available,
    "direct independent-watchdog configuration and constant-time feed"};

class error {
public:
  explicit constexpr error(hal::watchdog::error_kind kind) noexcept
      : kind_{kind} {}

  [[nodiscard]] constexpr hal::watchdog::error_kind kind() const noexcept {
    return kind_;
  }

private:
  hal::watchdog::error_kind kind_{};
};

enum class prescaler : std::uint8_t {
  divide_by_4 = 0U,
  divide_by_8 = 1U,
  divide_by_16 = 2U,
  divide_by_32 = 3U,
  divide_by_64 = 4U,
  divide_by_128 = 5U,
  divide_by_256 = 6U
};

template <class Registers, std::uint32_t LsiFrequencyHz,
          prescaler Divider = prescaler::divide_by_32,
          std::uint16_t Reload = 0x0FFFU>
class Feeder {
  static_assert(LsiFrequencyHz > 0U);
  static_assert(Reload <= 0x0FFFU);

public:
  using error_type = error;

  explicit Feeder(Registers &registers,
                  poll_budget timeout = {100'000U}) noexcept
      : registers_{registers}, timeout_{timeout} {}

  Feeder(const Feeder &) = delete;
  Feeder &operator=(const Feeder &) = delete;

  [[nodiscard]] static constexpr hal::watchdog::characteristics
  properties() noexcept {
    const std::uint32_t divider = divider_value();
    const std::uint64_t nanoseconds =
        (static_cast<std::uint64_t>(Reload) + 1U) * divider * 1'000'000'000ULL /
        LsiFrequencyHz;
    return {hal::nanoseconds{0U}, hal::nanoseconds{nanoseconds}, true};
  }

  [[nodiscard]] auto start() -> result<void, error_type> {
    if (!timeout_.valid()) {
      return result<void, error_type>::failure(
          error{hal::watchdog::error_kind::io});
    }
    registers_.KR = key_unlock;
    registers_.PR = static_cast<std::uint32_t>(Divider);
    registers_.RLR = Reload;
    for (std::uint32_t remaining = timeout_.iterations; remaining > 0U;
         --remaining) {
      if ((registers_.SR & (status_prescaler_update | status_reload_update)) ==
          0U) {
        registers_.KR = key_start;
        started_ = true;
        return result<void, error_type>::success();
      }
    }
    return result<void, error_type>::failure(
        error{hal::watchdog::error_kind::hardware_fault});
  }

  [[nodiscard]] auto feed() -> result<void, error_type> {
    if (!started_) {
      return result<void, error_type>::failure(
          error{hal::watchdog::error_kind::io});
    }
    registers_.KR = key_reload;
    return result<void, error_type>::success();
  }

  [[nodiscard]] bool started() const noexcept { return started_; }

private:
  static constexpr std::uint32_t key_reload{0xAAAAU};
  static constexpr std::uint32_t key_start{0xCCCCU};
  static constexpr std::uint32_t key_unlock{0x5555U};
  static constexpr std::uint32_t status_prescaler_update{1U << 0U};
  static constexpr std::uint32_t status_reload_update{1U << 1U};

  [[nodiscard]] static constexpr std::uint32_t divider_value() noexcept {
    constexpr std::uint32_t values[] = {4U, 8U, 16U, 32U, 64U, 128U, 256U};
    const auto index = static_cast<std::size_t>(Divider);
    return index < (sizeof(values) / sizeof(values[0U])) ? values[index] : 0U;
  }

  Registers &registers_;
  poll_budget timeout_{};
  bool started_{};
};

} // namespace hal::stm32h5::watchdog
