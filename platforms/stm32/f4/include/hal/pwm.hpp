#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/pwm.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include <cstdint>
#include "support.hpp"

namespace hal::stm32f4::pwm {

inline constexpr capability support{
    implementation_status::available,
    "direct timer compare output with optional complementary dead time"};

class error {
public:
  explicit constexpr error(hal::pwm::error_kind kind) noexcept : kind_{kind} {}

  [[nodiscard]] constexpr hal::pwm::error_kind kind() const noexcept {
    return kind_;
  }

private:
  hal::pwm::error_kind kind_{};
};

struct no_dead_time {};

struct default_compare_accessor {
  template <class Registers>
  [[nodiscard]] static volatile std::uint32_t &get(Registers &registers,
                                                   unsigned channel) noexcept {
    switch (channel) {
    case 1U:
      return registers.CCR1;
    case 2U:
      return registers.CCR2;
    case 3U:
      return registers.CCR3;
    default:
      return registers.CCR4;
    }
  }
};

template <class Registers, std::uint32_t TimerClockHz, unsigned Channel,
          class CompareAccessor = default_compare_accessor,
          bool Complementary = false>
class Output {
  static_assert(TimerClockHz > 0U);
  static_assert(Channel >= 1U && Channel <= 4U);
  static_assert(!Complementary || Channel <= 3U);

public:
  using error_type = error;

  explicit Output(Registers &registers) noexcept : registers_{registers} {}

  Output(const Output &) = delete;
  Output &operator=(const Output &) = delete;

  [[nodiscard]] auto configure(hal::pwm::config configuration)
      -> result<hal::nanoseconds, error_type> {
    if (configuration.requested_period.value == 0U ||
        configuration.max_error_ppm > 1'000'000U) {
      return failure_period(hal::pwm::error_kind::unrepresentable);
    }

    std::uint32_t best_psc = 0U;
    std::uint32_t best_arr = 0U;
    std::uint64_t best_error = UINT64_MAX;
    bool found = false;
    for (std::uint32_t psc = 0U; psc <= 0xFFFFU; ++psc) {
      const std::uint64_t denominator =
          1'000'000'000ULL * (static_cast<std::uint64_t>(psc) + 1U);
      const std::uint64_t numerator = static_cast<std::uint64_t>(TimerClockHz) *
                                      configuration.requested_period.value;
      std::uint64_t ticks = (numerator + denominator / 2U) / denominator;
      if (ticks == 0U) {
        ticks = 1U;
      }
      if (ticks > 0x1'0000'0000ULL) {
        continue;
      }
      const std::uint64_t actual_period = (denominator * ticks) / TimerClockHz;
      const std::uint64_t error =
          actual_period > configuration.requested_period.value
              ? actual_period - configuration.requested_period.value
              : configuration.requested_period.value - actual_period;
      if (!found || error < best_error) {
        found = true;
        best_psc = psc;
        best_arr = static_cast<std::uint32_t>(ticks - 1U);
        best_error = error;
      }
    }
    if (!found ||
        !within_error(best_error, configuration.requested_period.value,
                      configuration.max_error_ppm)) {
      return failure_period(hal::pwm::error_kind::unrepresentable);
    }

    registers_.CR1 = registers_.CR1 & ~counter_enable;
    registers_.PSC = best_psc;
    registers_.ARR = best_arr;
    registers_.EGR = update_generation;
    CompareAccessor::get(registers_, Channel) = 0U;
    const std::uint32_t shift = (Channel - 1U) * 4U;
    const std::uint32_t mask = 0xFU << shift;
    std::uint32_t ccer = registers_.CCER & ~mask;
    if (configuration.output_polarity == hal::pwm::polarity::active_low) {
      ccer |= 1U << (shift + polarity_bit);
    }
    registers_.CCER = ccer;
    registers_.CR1 = registers_.CR1 | auto_reload_preload;
    configured_ = true;
    enabled_ = false;
    actual_period_ = period_for(best_psc, best_arr);
    return result<hal::nanoseconds, error_type>::success(actual_period_);
  }

  [[nodiscard]] auto set_pulse_width(hal::nanoseconds pulse)
      -> result<void, error_type> {
    if (!configured_ || pulse.value > actual_period_.value) {
      return failure(hal::pwm::error_kind::not_configured);
    }
    const std::uint64_t numerator =
        pulse.value * static_cast<std::uint64_t>(TimerClockHz);
    const std::uint64_t denominator =
        1'000'000'000ULL * (static_cast<std::uint64_t>(registers_.PSC) + 1U);
    std::uint64_t ticks = (numerator + denominator / 2U) / denominator;
    const std::uint64_t period_ticks =
        static_cast<std::uint64_t>(registers_.ARR) + 1U;
    if (ticks > period_ticks) {
      ticks = period_ticks;
    }
    CompareAccessor::get(registers_, Channel) =
        static_cast<std::uint32_t>(ticks);
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto enable_output() -> result<void, error_type> {
    if (!configured_) {
      return failure(hal::pwm::error_kind::not_configured);
    }
    const std::uint32_t shift = (Channel - 1U) * 4U;
    registers_.CCER = registers_.CCER | (1U << (shift + enable_bit));
    if constexpr (requires(Registers &value) { value.BDTR; }) {
      registers_.BDTR = registers_.BDTR | main_output_enable;
    }
    registers_.CR1 = registers_.CR1 | counter_enable;
    enabled_ = true;
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto disable_output() -> result<void, error_type> {
    const std::uint32_t shift = (Channel - 1U) * 4U;
    registers_.CCER = registers_.CCER & ~(1U << (shift + enable_bit));
    enabled_ = false;
    return result<void, error_type>::success();
  }

  // Advanced timers use the nonlinear DTG encoding. The generic family
  // driver accepts the highest representable value not exceeding the request.
  [[nodiscard]] auto set_dead_time(hal::nanoseconds duration)
      -> result<void, error_type> {
    if constexpr (!requires(Registers &value) { value.BDTR; }) {
      (void)duration;
      return failure(hal::pwm::error_kind::other);
    } else {
      const std::uint64_t ticks =
          (duration.value * TimerClockHz) / 1'000'000'000ULL;
      const auto code = dead_time_code(ticks);
      if (!code.valid) {
        return failure(hal::pwm::error_kind::unrepresentable);
      }
      registers_.BDTR = (registers_.BDTR & ~(0xFFU << 8U)) |
                        (static_cast<std::uint32_t>(code.value) << 8U);
      return result<void, error_type>::success();
    }
  }

  [[nodiscard]] bool enabled() const noexcept { return enabled_; }
  [[nodiscard]] hal::nanoseconds actual_period() const noexcept {
    return actual_period_;
  }

private:
  static constexpr std::uint32_t enable_bit{Complementary ? 2U : 0U};
  static constexpr std::uint32_t polarity_bit{Complementary ? 3U : 1U};
  static constexpr std::uint32_t counter_enable{1U << 0U};
  static constexpr std::uint32_t auto_reload_preload{1U << 7U};
  static constexpr std::uint32_t update_generation{1U << 0U};
  static constexpr std::uint32_t main_output_enable{1U << 15U};

  struct dead_time_result {
    std::uint8_t value{};
    bool valid{};
  };

  [[nodiscard]] static constexpr dead_time_result
  dead_time_code(std::uint64_t ticks) noexcept {
    if (ticks <= 127U) {
      return {static_cast<std::uint8_t>(ticks), true};
    }
    if (ticks <= 254U) {
      return {static_cast<std::uint8_t>(0x80U + ((ticks - 128U) / 2U)), true};
    }
    if (ticks <= 504U) {
      return {static_cast<std::uint8_t>(0xC0U + ((ticks - 256U) / 8U)), true};
    }
    if (ticks <= 1008U) {
      return {static_cast<std::uint8_t>(0xE0U + ((ticks - 512U) / 16U)), true};
    }
    return {};
  }

  [[nodiscard]] static constexpr hal::nanoseconds
  period_for(std::uint32_t psc, std::uint32_t arr) noexcept {
    const std::uint64_t numerator = 1'000'000'000ULL *
                                    (static_cast<std::uint64_t>(psc) + 1U) *
                                    (static_cast<std::uint64_t>(arr) + 1U);
    return {numerator / TimerClockHz};
  }

  [[nodiscard]] static constexpr bool within_error(std::uint64_t error,
                                                   std::uint64_t requested,
                                                   std::uint32_t ppm) noexcept {
    return error * 1'000'000ULL <= requested * ppm;
  }

  [[nodiscard]] auto failure(hal::pwm::error_kind kind)
      -> result<void, error_type> {
    return result<void, error_type>::failure(error{kind});
  }

  [[nodiscard]] auto failure_period(hal::pwm::error_kind kind)
      -> result<hal::nanoseconds, error_type> {
    return result<hal::nanoseconds, error_type>::failure(error{kind});
  }

  Registers &registers_;
  hal::nanoseconds actual_period_{};
  bool configured_{};
  bool enabled_{};
};

} // namespace hal::stm32f4::pwm
