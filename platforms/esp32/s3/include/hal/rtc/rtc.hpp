#pragma once

#include <cstdint>
#include <hal/foundation/result.hpp>
#include <hal/foundation/units.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/rtc.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif
#include <soc/rtc_cntl_struct.h>

namespace hal::esp32s3_wroom_1_n16r8::rtc {

class error {
 public:
  [[nodiscard]] static constexpr error not_set() noexcept {
    return error{hal::rtc::error_kind::not_set};
  }
  [[nodiscard]] static constexpr error out_of_range() noexcept {
    return error{hal::rtc::error_kind::out_of_range};
  }
  [[nodiscard]] static constexpr error io() noexcept {
    return error{hal::rtc::error_kind::io};
  }
  [[nodiscard]] constexpr hal::rtc::error_kind kind() const noexcept {
    return kind_;
  }

 private:
  constexpr explicit error(hal::rtc::error_kind kind) noexcept : kind_{kind} {}
  hal::rtc::error_kind kind_;
};

// The RTC main timer is a free-running 48-bit slow-clock counter. The BSP must
// select SlowClockHz from its calibrated RTC slow clock; 150 kHz is the
// conservative default used by the ESP32-S3 clock tree.
template <std::uint32_t SlowClockHz = 150'000U>
class Clock {
  static_assert(SlowClockHz != 0U);

 public:
  using error_type = error;

  explicit Clock(rtc_cntl_dev_t& peripheral) noexcept : peripheral_{&peripheral} {}

  Clock(const Clock&) = delete;
  Clock& operator=(const Clock&) = delete;

  [[nodiscard]] static constexpr hal::rtc::persistence
  persistence_characteristics() noexcept {
    return {true, true, false};
  }

  [[nodiscard]] result<utc_time, error_type> read() noexcept {
    if (peripheral_->store[0] != magic) {
      return result<utc_time, error_type>::failure(error_type::not_set());
    }
    const std::uint64_t now = read_ticks();
    const std::uint64_t anchor =
        static_cast<std::uint64_t>(peripheral_->store[4]) |
        (static_cast<std::uint64_t>(peripheral_->store[5] & 0xFFFFU) << 32U);
    const std::uint64_t elapsed_ticks = (now - anchor) & tick_mask;
    const std::int64_t base_seconds = static_cast<std::int64_t>(
        static_cast<std::uint64_t>(peripheral_->store[1]) |
        (static_cast<std::uint64_t>(peripheral_->store[2]) << 32U));
    const std::uint32_t base_nanos = peripheral_->store[3];
    if (base_nanos >= 1'000'000'000U) {
      return result<utc_time, error_type>::failure(error_type::io());
    }

    const std::uint64_t whole_seconds = elapsed_ticks / SlowClockHz;
    const std::uint32_t fractional_nanos = static_cast<std::uint32_t>(
        ((elapsed_ticks % SlowClockHz) * 1'000'000'000ULL) / SlowClockHz);
    if (whole_seconds > static_cast<std::uint64_t>(INT64_MAX) ||
        base_seconds > INT64_MAX - static_cast<std::int64_t>(whole_seconds)) {
      return result<utc_time, error_type>::failure(error_type::out_of_range());
    }
    std::int64_t seconds = base_seconds + static_cast<std::int64_t>(whole_seconds);
    std::uint64_t nanos = static_cast<std::uint64_t>(base_nanos) + fractional_nanos;
    if (nanos >= 1'000'000'000ULL) {
      nanos -= 1'000'000'000ULL;
      if (seconds == INT64_MAX) {
        return result<utc_time, error_type>::failure(error_type::out_of_range());
      }
      ++seconds;
    }
    return result<utc_time, error_type>::success(
        utc_time{seconds, static_cast<std::uint32_t>(nanos)});
  }

  [[nodiscard]] result<void, error_type> set(utc_time value) noexcept {
    if (value.nanoseconds >= 1'000'000'000U) {
      return result<void, error_type>::failure(error_type::out_of_range());
    }
    const std::uint64_t ticks = read_ticks();
    peripheral_->store[0] = magic;
    peripheral_->store[1] = static_cast<std::uint32_t>(value.seconds);
    peripheral_->store[2] = static_cast<std::uint32_t>(
        static_cast<std::uint64_t>(value.seconds) >> 32U);
    peripheral_->store[3] = value.nanoseconds;
    peripheral_->store[4] = static_cast<std::uint32_t>(ticks);
    peripheral_->store[5] = static_cast<std::uint32_t>(ticks >> 32U) & 0xFFFFU;
    return result<void, error_type>::success();
  }

  [[nodiscard]] result<void, error_type> set_alarm(utc_time value) noexcept {
    const auto current = read();
    if (!current) {
      return result<void, error_type>::failure(current.error());
    }
    if (value.nanoseconds >= 1'000'000'000U ||
        value.seconds < current.value().seconds ||
        (value.seconds == current.value().seconds &&
         value.nanoseconds <= current.value().nanoseconds)) {
      return result<void, error_type>::failure(error_type::out_of_range());
    }
    const std::uint64_t anchor_ticks =
        static_cast<std::uint64_t>(peripheral_->store[4]) |
        (static_cast<std::uint64_t>(peripheral_->store[5] & 0xFFFFU) << 32U);
    const bool borrow_nanoseconds =
        value.nanoseconds < current.value().nanoseconds;
    const std::uint64_t delta_seconds = static_cast<std::uint64_t>(
        value.seconds - current.value().seconds -
        (borrow_nanoseconds ? static_cast<std::int64_t>(1) :
                              static_cast<std::int64_t>(0)));
    const std::uint64_t delta_nanos =
        value.nanoseconds >= current.value().nanoseconds
            ? static_cast<std::uint64_t>(value.nanoseconds -
                                         current.value().nanoseconds)
            : 1'000'000'000ULL + value.nanoseconds -
                  current.value().nanoseconds;
    const std::uint64_t delta_ticks =
        delta_seconds * SlowClockHz +
        (delta_nanos * SlowClockHz + 999'999'999ULL) / 1'000'000'000ULL;
    const std::uint64_t deadline = (anchor_ticks + delta_ticks) & tick_mask;
    peripheral_->slp_timer0 = static_cast<std::uint32_t>(deadline);
    peripheral_->slp_timer1.slp_val_hi =
        static_cast<std::uint32_t>((deadline >> 32U) & 0xFFFFU);
    peripheral_->slp_timer1.main_timer_alarm_en = 1U;
    peripheral_->int_clr.rtc_main_timer = 1U;
    peripheral_->int_ena.rtc_main_timer = 1U;
    alarm_configured_ = true;
    return result<void, error_type>::success();
  }

  [[nodiscard]] bool alarm_pending() const noexcept {
    return alarm_configured_ && peripheral_->int_st.rtc_main_timer != 0U;
  }

  void clear_alarm() noexcept {
    peripheral_->int_clr.rtc_main_timer = 1U;
    peripheral_->slp_timer1.main_timer_alarm_en = 0U;
    peripheral_->int_ena.rtc_main_timer = 0U;
    alarm_configured_ = false;
  }

  [[nodiscard]] std::uint64_t ticks() noexcept { return read_ticks(); }

 private:
  static constexpr std::uint32_t magic = 0xEC53'3A3U;
  static constexpr std::uint64_t tick_mask = (std::uint64_t{1U} << 48U) - 1U;

  [[nodiscard]] std::uint64_t read_ticks() noexcept {
    peripheral_->time_update.update = 1U;
    const std::uint32_t high = peripheral_->time_high0.rtc_timer_value0_high;
    const std::uint32_t low = peripheral_->time_low0;
    const std::uint32_t high_again =
        peripheral_->time_high0.rtc_timer_value0_high;
    const std::uint32_t stable_low = high == high_again
                                         ? low
                                         : peripheral_->time_low0;
    return (static_cast<std::uint64_t>(high_again) << 32U) | stable_low;
  }

  rtc_cntl_dev_t* peripheral_;
  bool alarm_configured_{};
};

}  // namespace hal::esp32s3_wroom_1_n16r8::rtc
