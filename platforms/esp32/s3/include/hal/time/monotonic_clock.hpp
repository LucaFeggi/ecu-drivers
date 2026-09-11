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
 public:
  using error_type = error;

  explicit MonotonicClock(systimer_dev_t& systimer) noexcept
      : systimer_{&systimer} {}

  [[nodiscard]] result<void, error_type> initialize() noexcept {
    systimer_->conf.clk_en = 1U;
    systimer_->conf.timer_unit0_work_en = 1U;
    return result<void, error_type>::success();
  }

  [[nodiscard]] instant now() const noexcept {
    systimer_->unit_op[0].timer_unit_update = 1U;
    for (std::uint32_t remaining = 4U; remaining != 0U; --remaining) {
      if (systimer_->unit_op[0].timer_unit_value_valid != 0U) {
        break;
      }
    }
    std::uint32_t high = systimer_->unit_val[0].hi.timer_unit_value_hi;
    std::uint32_t low = systimer_->unit_val[0].lo.timer_unit_value_lo;
    std::uint32_t high_again =
        systimer_->unit_val[0].hi.timer_unit_value_hi;
    if (high != high_again) {
      high = high_again;
      low = systimer_->unit_val[0].lo.timer_unit_value_lo;
    }
    const std::uint64_t ticks = (std::uint64_t{high} << 32U) | low;
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
  systimer_dev_t* systimer_;
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
