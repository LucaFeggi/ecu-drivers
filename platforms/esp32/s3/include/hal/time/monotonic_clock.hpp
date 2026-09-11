#pragma once

#include <cstdint>
#include <hal/foundation/result.hpp>
#include <hal/foundation/units.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/time.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif
#include <soc/systimer_struct.h>

namespace hal::esp32s3_wroom_1_n16r8::time {

class error {
 public:
  [[nodiscard]] static constexpr error io() noexcept { return {}; }
  [[nodiscard]] constexpr std::uint8_t value() const noexcept { return 0U; }
  [[nodiscard]] constexpr bool operator==(const error&) const noexcept =
      default;
};

template <std::uint32_t ClockHz = 16'000'000U>
class MonotonicClock {
  static_assert(ClockHz > 0U);

 public:
  using error_type = error;

  explicit MonotonicClock(systimer_dev_t& systimer,
                          std::uint32_t poll_limit = 1'000U) noexcept
      : systimer_{&systimer}, poll_limit_{poll_limit} {}

  [[nodiscard]] result<void, error_type> initialize() noexcept {
    systimer_->conf.clk_en = 1U;
    systimer_->conf.timer_unit0_work_en = 1U;
    std::uint64_t ticks{};
    return capture_ticks(ticks) ? result<void, error_type>::success()
                                : result<void, error_type>::failure(error::io());
  }

  [[nodiscard]] instant now() const noexcept {
    std::uint64_t ticks{};
    if (capture_ticks(ticks)) {
      // Returning the last observation on a stale/invalid snapshot keeps the
      // infallible portable clock contract monotonic. Users that need to
      // distinguish stale hardware use try_now().
      if (ticks >= last_ticks_) {
        last_ticks_ = ticks;
      }
    }
    return ticks_to_instant(last_ticks_);
  }

  [[nodiscard]] result<instant, error_type> try_now() const noexcept {
    std::uint64_t ticks{};
    if (!capture_ticks(ticks)) {
      return result<instant, error_type>::failure(error::io());
    }
    if (ticks >= last_ticks_) {
      last_ticks_ = ticks;
    }
    return result<instant, error_type>::success(ticks_to_instant(last_ticks_));
  }

  [[nodiscard]] bool observation_valid() const noexcept {
    return observation_valid_;
  }

  [[nodiscard]] static constexpr instant
  ticks_to_instant(std::uint64_t ticks) noexcept {
    const std::uint64_t whole_seconds = ticks / ClockHz;
    const std::uint64_t remainder = ticks % ClockHz;
    return instant{whole_seconds * 1'000'000'000ULL +
                   (remainder * 1'000'000'000ULL) / ClockHz};
  }

  [[nodiscard]] static constexpr std::uint64_t nanoseconds_to_ticks(
      instant value) noexcept {
    const std::uint64_t seconds =
        value.nanoseconds_since_boot / 1'000'000'000ULL;
    const std::uint64_t remainder =
        value.nanoseconds_since_boot % 1'000'000'000ULL;
    return seconds * ClockHz + (remainder * ClockHz) / 1'000'000'000ULL;
  }

 protected:
  [[nodiscard]] bool capture_ticks(std::uint64_t& ticks) const noexcept {
    systimer_->unit_op[0].timer_unit_update = 1U;
    bool valid = false;
    for (std::uint32_t remaining = poll_limit_; remaining != 0U; --remaining) {
      if (systimer_->unit_op[0].timer_unit_value_valid != 0U) {
        valid = true;
        break;
      }
    }
    if (!valid) {
      observation_valid_ = false;
      return false;
    }
    std::uint32_t high = systimer_->unit_val[0].hi.timer_unit_value_hi;
    std::uint32_t low = systimer_->unit_val[0].lo.timer_unit_value_lo;
    std::uint32_t high_again =
        systimer_->unit_val[0].hi.timer_unit_value_hi;
    if (high != high_again) {
      high = high_again;
      low = systimer_->unit_val[0].lo.timer_unit_value_lo;
    }
    ticks = (std::uint64_t{high} << 32U) | low;
    observation_valid_ = true;
    return true;
  }

  systimer_dev_t* systimer_;
  std::uint32_t poll_limit_;
  mutable std::uint64_t last_ticks_{};
  mutable bool observation_valid_{};
};

template <std::uint32_t ClockHz = 16'000'000U>
class Delay {
 public:
  explicit Delay(MonotonicClock<ClockHz>& clock) noexcept : clock_{clock} {}

  void delay_for(nanoseconds duration) noexcept {
    const instant start = clock_.now();
    while (clock_.now().nanoseconds_since_boot -
               start.nanoseconds_since_boot <
           duration.value) {
      asm volatile("" ::: "memory");
    }
  }

 private:
  MonotonicClock<ClockHz>& clock_;
};

template <std::uint32_t ClockHz = 16'000'000U>
class OneShotAlarm : public MonotonicClock<ClockHz> {
 public:
  using error_type = error;

  explicit OneShotAlarm(systimer_dev_t& systimer) noexcept
      : MonotonicClock<ClockHz>{systimer} {}

  [[nodiscard]] result<void, error_type> arm(instant deadline) noexcept {
    const std::uint64_t ticks = this->nanoseconds_to_ticks(deadline);
    this->systimer_->target_val[0].hi.timer_target_hi =
        static_cast<std::uint32_t>((ticks >> 32U) & 0xFFFFFU);
    this->systimer_->target_val[0].lo.timer_target_lo =
        static_cast<std::uint32_t>(ticks);
    this->systimer_->target_conf[0].target_period = 0U;
    this->systimer_->target_conf[0].target_period_mode = 0U;
    this->systimer_->target_conf[0].target_timer_unit_sel = 0U;
    this->systimer_->conf.target0_work_en = 1U;
    this->systimer_->int_ena.target0_int_ena = 1U;
    this->systimer_->comp_load[0].timer_comp_load = 1U;
    return result<void, error_type>::success();
  }

  [[nodiscard]] result<void, error_type> cancel() noexcept {
    this->systimer_->int_ena.target0_int_ena = 0U;
    this->systimer_->conf.target0_work_en = 0U;
    this->systimer_->int_clr.target0_int_clr = 1U;
    return result<void, error_type>::success();
  }

  [[nodiscard]] bool expired() const noexcept {
    return this->systimer_->int_st.target0_int_st != 0U;
  }
};

}  // namespace hal::esp32s3_wroom_1_n16r8::time
