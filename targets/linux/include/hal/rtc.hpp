#pragma once

#include <cerrno>
#include <cstdint>
#include <fcntl.h>
#include <hal/foundation/result.hpp>
#include <hal/rtc.hpp>
#include <linux/rtc.h>
#include <limits>
#include <sys/ioctl.h>
#include <sys/timerfd.h>
#include <time.h>
#include <utility>

#include <hal/linux/detail/error.hpp>
#include <hal/linux/detail/fd.hpp>

namespace hal::linux::rtc {

using error_type = detail::error<hal::rtc::error_kind>;

namespace detail_rtc {

[[nodiscard]] inline error_type failure(int native_error,
                                        detail::operation operation) noexcept {
  const auto kind = native_error == EINVAL
                        ? hal::rtc::error_kind::out_of_range
                        : native_error == ENODEV || native_error == ENOENT
                              ? hal::rtc::error_kind::clock_failure
                              : hal::rtc::error_kind::io;
  return error_type{kind, native_error, operation};
}

[[nodiscard]] constexpr std::int64_t days_from_civil(
    std::int64_t year, unsigned month, unsigned day) noexcept {
  year -= month <= 2U ? 1 : 0;
  const std::int64_t era = (year >= 0 ? year : year - 399) / 400;
  const unsigned year_of_era = static_cast<unsigned>(year - era * 400);
  const unsigned adjusted_month = month > 2U ? month - 3U : month + 9U;
  const unsigned day_of_year =
      (153U * adjusted_month + 2U) / 5U + day - 1U;
  const unsigned day_of_era = year_of_era * 365U + year_of_era / 4U -
                              year_of_era / 100U + day_of_year;
  return era * 146097 + static_cast<std::int64_t>(day_of_era) - 719468;
}

struct civil_date {
  std::int64_t year{};
  unsigned month{};
  unsigned day{};
};

[[nodiscard]] constexpr civil_date civil_from_days(std::int64_t days) noexcept {
  days += 719468;
  const std::int64_t era = (days >= 0 ? days : days - 146096) / 146097;
  const unsigned day_of_era = static_cast<unsigned>(days - era * 146097);
  const unsigned year_of_era =
      (day_of_era - day_of_era / 1460U + day_of_era / 36524U -
       day_of_era / 146096U) /
      365U;
  const std::int64_t year = era * 400 + static_cast<std::int64_t>(year_of_era);
  const unsigned day_of_year =
      day_of_era - (365U * year_of_era + year_of_era / 4U - year_of_era / 100U);
  const unsigned month = (5U * day_of_year + 2U) / 153U;
  const unsigned day =
      day_of_year - (153U * month + 2U) / 5U + 1U;
  const unsigned actual_month = month + (month < 10U ? 3U : 9U);
  return {year + (actual_month <= 2U ? 1 : 0), actual_month, day};
}

[[nodiscard]] constexpr bool valid_utc(hal::utc_time value) noexcept {
  return value.nanoseconds < 1'000'000'000U;
}

[[nodiscard]] constexpr std::int64_t floor_days(std::int64_t seconds) noexcept {
  constexpr std::int64_t seconds_per_day{86'400};
  const std::int64_t quotient = seconds / seconds_per_day;
  const std::int64_t remainder = seconds % seconds_per_day;
  return quotient - (remainder < 0 ? 1 : 0);
}

[[nodiscard]] inline bool to_rtc_time(hal::utc_time value,
                                      ::rtc_time& result) noexcept {
  if (!valid_utc(value)) {
    return false;
  }
  constexpr std::int64_t seconds_per_day{86'400};
  const std::int64_t days = floor_days(value.seconds);
  const std::int64_t seconds_today = value.seconds - days * seconds_per_day;
  const civil_date date = civil_from_days(days);
  if (date.year < static_cast<std::int64_t>(std::numeric_limits<int>::min()) +
                       1900LL ||
      date.year > static_cast<std::int64_t>(std::numeric_limits<int>::max()) +
                       1900LL) {
    return false;
  }
  result = {};
  result.tm_year = static_cast<int>(date.year - 1900);
  result.tm_mon = static_cast<int>(date.month - 1U);
  result.tm_mday = static_cast<int>(date.day);
  result.tm_hour = static_cast<int>(seconds_today / 3600);
  result.tm_min = static_cast<int>((seconds_today % 3600) / 60);
  result.tm_sec = static_cast<int>(seconds_today % 60);
  return true;
}

[[nodiscard]] inline bool from_rtc_time(const ::rtc_time& value,
                                        hal::utc_time& result) noexcept {
  if (value.tm_sec < 0 || value.tm_sec > 59 || value.tm_min < 0 ||
      value.tm_min > 59 || value.tm_hour < 0 || value.tm_hour > 23 ||
      value.tm_mon < 0 || value.tm_mon > 11 || value.tm_mday < 1 ||
      value.tm_mday > 31) {
    return false;
  }
  const std::int64_t year = static_cast<std::int64_t>(value.tm_year) + 1900;
  const unsigned month = static_cast<unsigned>(value.tm_mon + 1);
  const unsigned day = static_cast<unsigned>(value.tm_mday);
  const std::int64_t days = days_from_civil(year, month, day);
  constexpr std::int64_t seconds_per_day{86'400};
  const std::int64_t seconds =
      days * seconds_per_day + static_cast<std::int64_t>(value.tm_hour) * 3600 +
      static_cast<std::int64_t>(value.tm_min) * 60 + value.tm_sec;
  result = {seconds, 0U};
  return civil_from_days(days).year == year &&
         civil_from_days(days).month == month &&
         civil_from_days(days).day == day;
}

[[nodiscard]] inline bool to_timespec(hal::utc_time value,
                                      ::timespec& result) noexcept {
  if (!valid_utc(value)) {
    return false;
  }
  result.tv_sec = static_cast<time_t>(value.seconds);
  result.tv_nsec = static_cast<long>(value.nanoseconds);
  return static_cast<std::int64_t>(result.tv_sec) == value.seconds;
}

[[nodiscard]] inline bool from_timespec(const ::timespec& value,
                                        hal::utc_time& result) noexcept {
  if (value.tv_nsec < 0 || value.tv_nsec >= 1'000'000'000L ||
      value.tv_sec < 0) {
    return false;
  }
  result = {static_cast<std::int64_t>(value.tv_sec),
            static_cast<std::uint32_t>(value.tv_nsec)};
  return true;
}

}  // namespace detail_rtc

class SystemClock {
 public:
  using error_type = linux::rtc::error_type;

  [[nodiscard]] static constexpr hal::rtc::persistence
  persistence_characteristics() noexcept {
    return {};
  }

  [[nodiscard]] auto read() -> hal::result<hal::utc_time, error_type> {
    ::timespec value{};
    if (::clock_gettime(CLOCK_REALTIME, &value) < 0) {
      return failure_time(errno, detail::operation::read);
    }
    hal::utc_time result{};
    if (!detail_rtc::from_timespec(value, result)) {
      return failure_time(EINVAL, detail::operation::read);
    }
    return hal::result<hal::utc_time, error_type>::success(result);
  }

  [[nodiscard]] auto set(hal::utc_time value)
      -> hal::result<void, error_type> {
    ::timespec timespec_value{};
    if (!detail_rtc::to_timespec(value, timespec_value)) {
      return failure_void(EINVAL, detail::operation::configure);
    }
    if (::clock_settime(CLOCK_REALTIME, &timespec_value) < 0) {
      return failure_void(errno, detail::operation::write);
    }
    return hal::result<void, error_type>::success();
  }

 private:
  [[nodiscard]] static auto failure_time(int native_error,
                                         detail::operation operation)
      -> hal::result<hal::utc_time, error_type> {
    return hal::result<hal::utc_time, error_type>::failure(
        detail_rtc::failure(native_error, operation));
  }
  [[nodiscard]] static auto failure_void(int native_error,
                                         detail::operation operation)
      -> hal::result<void, error_type> {
    return hal::result<void, error_type>::failure(
        detail_rtc::failure(native_error, operation));
  }
};

class SystemClockAlarm {
 public:
  using error_type = linux::rtc::error_type;

  [[nodiscard]] static auto open()
      -> hal::result<SystemClockAlarm, error_type> {
    const int fd = ::timerfd_create(CLOCK_REALTIME,
                                    TFD_CLOEXEC | TFD_NONBLOCK);
    if (fd < 0) {
      return hal::result<SystemClockAlarm, error_type>::failure(
          detail_rtc::failure(errno, detail::operation::open));
    }
    return hal::result<SystemClockAlarm, error_type>::success(
        SystemClockAlarm{detail::owned_fd{fd}});
  }

  SystemClockAlarm(const SystemClockAlarm&) = delete;
  SystemClockAlarm& operator=(const SystemClockAlarm&) = delete;
  SystemClockAlarm(SystemClockAlarm&&) noexcept = default;
  SystemClockAlarm& operator=(SystemClockAlarm&&) noexcept = default;

  [[nodiscard]] static constexpr hal::rtc::persistence
  persistence_characteristics() noexcept {
    return {};
  }

  [[nodiscard]] auto read() -> hal::result<hal::utc_time, error_type> {
    return clock_.read();
  }
  [[nodiscard]] auto set(hal::utc_time value)
      -> hal::result<void, error_type> {
    return clock_.set(value);
  }
  [[nodiscard]] auto set_alarm(hal::utc_time value)
      -> hal::result<void, error_type> {
    ::timespec time_value{};
    if (!detail_rtc::to_timespec(value, time_value)) {
      return failure_void(EINVAL, detail::operation::configure);
    }
    ::itimerspec timer{};
    timer.it_value = time_value;
    if (::timerfd_settime(fd_.get(), TFD_TIMER_ABSTIME, &timer, nullptr) < 0) {
      return failure_void(errno, detail::operation::ioctl);
    }
    fault_ = false;
    return hal::result<void, error_type>::success();
  }
  [[nodiscard]] bool alarm_pending() noexcept {
    std::uint64_t expirations{};
    const ssize_t count = ::read(fd_.get(), &expirations, sizeof(expirations));
    if (count == static_cast<ssize_t>(sizeof(expirations))) {
      return expirations != 0U;
    }
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
      return false;
    }
    fault_ = true;
    return false;
  }
  void clear_alarm() noexcept {
    ::itimerspec timer{};
    if (::timerfd_settime(fd_.get(), 0, &timer, nullptr) < 0) {
      fault_ = true;
      return;
    }
    std::uint64_t expirations{};
    while (::read(fd_.get(), &expirations, sizeof(expirations)) > 0) {
    }
    fault_ = false;
  }
  [[nodiscard]] bool has_fault() const noexcept { return fault_; }
  [[nodiscard]] int native_fd() const noexcept { return fd_.get(); }

 private:
  explicit SystemClockAlarm(detail::owned_fd fd) noexcept : fd_{std::move(fd)} {}

  [[nodiscard]] static auto failure_void(int native_error,
                                         detail::operation operation)
      -> hal::result<void, error_type> {
    return hal::result<void, error_type>::failure(
        detail_rtc::failure(native_error, operation));
  }

  detail::owned_fd fd_{};
  SystemClock clock_{};
  bool fault_{};
};

template <bool SurvivesWarmReset = false, bool SurvivesDeepSleep = false,
          bool SurvivesPowerLoss = false>
class HardwareClock {
 public:
  using error_type = linux::rtc::error_type;

  [[nodiscard]] static constexpr hal::rtc::persistence
  persistence_characteristics() noexcept {
    return {SurvivesWarmReset, SurvivesDeepSleep, SurvivesPowerLoss};
  }

  [[nodiscard]] static auto open(const char* device)
      -> hal::result<HardwareClock, error_type> {
    if (device == nullptr) {
      return failure(EINVAL, detail::operation::open);
    }
    const int fd = ::open(device, O_RDWR | O_CLOEXEC);
    if (fd < 0) {
      return failure(errno, detail::operation::open);
    }
    return hal::result<HardwareClock, error_type>::success(
        HardwareClock{detail::owned_fd{fd}});
  }

  HardwareClock(const HardwareClock&) = delete;
  HardwareClock& operator=(const HardwareClock&) = delete;
  HardwareClock(HardwareClock&&) noexcept = default;
  HardwareClock& operator=(HardwareClock&&) noexcept = default;

  [[nodiscard]] auto read() -> hal::result<hal::utc_time, error_type> {
    ::rtc_time value{};
    if (::ioctl(fd_.get(), RTC_RD_TIME, &value) < 0) {
      return failure_time(errno, detail::operation::ioctl);
    }
    hal::utc_time result{};
    if (!detail_rtc::from_rtc_time(value, result)) {
      return failure_time(EINVAL, detail::operation::read);
    }
    return hal::result<hal::utc_time, error_type>::success(result);
  }

  [[nodiscard]] auto set(hal::utc_time value)
      -> hal::result<void, error_type> {
    ::rtc_time rtc_value{};
    if (!detail_rtc::to_rtc_time(value, rtc_value)) {
      return failure_void(EINVAL, detail::operation::configure);
    }
    if (::ioctl(fd_.get(), RTC_SET_TIME, &rtc_value) < 0) {
      return failure_void(errno, detail::operation::ioctl);
    }
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] auto set_alarm(hal::utc_time value)
      -> hal::result<void, error_type> {
    ::rtc_wkalrm alarm{};
    if (!detail_rtc::to_rtc_time(value, alarm.time)) {
      return failure_void(EINVAL, detail::operation::configure);
    }
    alarm.enabled = 1U;
    if (::ioctl(fd_.get(), RTC_WKALM_SET, &alarm) < 0) {
      return failure_void(errno, detail::operation::ioctl);
    }
    alarm_fault_ = false;
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] bool alarm_pending() noexcept {
    ::rtc_wkalrm alarm{};
    if (::ioctl(fd_.get(), RTC_WKALM_RD, &alarm) < 0) {
      alarm_fault_ = true;
      return false;
    }
    return alarm.pending != 0U;
  }

  void clear_alarm() noexcept {
    ::rtc_wkalrm alarm{};
    if (::ioctl(fd_.get(), RTC_WKALM_RD, &alarm) < 0) {
      alarm_fault_ = true;
      return;
    }
    alarm.enabled = 0U;
    if (::ioctl(fd_.get(), RTC_WKALM_SET, &alarm) < 0) {
      alarm_fault_ = true;
      return;
    }
    alarm_fault_ = false;
  }

  [[nodiscard]] bool has_alarm_fault() const noexcept { return alarm_fault_; }
  [[nodiscard]] int native_fd() const noexcept { return fd_.get(); }

 private:
  explicit HardwareClock(detail::owned_fd fd) noexcept : fd_{std::move(fd)} {}

  [[nodiscard]] static auto failure(int native_error, detail::operation operation)
      -> hal::result<HardwareClock, error_type> {
    return hal::result<HardwareClock, error_type>::failure(
        detail_rtc::failure(native_error, operation));
  }
  [[nodiscard]] static auto failure_time(int native_error,
                                         detail::operation operation)
      -> hal::result<hal::utc_time, error_type> {
    return hal::result<hal::utc_time, error_type>::failure(
        detail_rtc::failure(native_error, operation));
  }
  [[nodiscard]] static auto failure_void(int native_error,
                                         detail::operation operation)
      -> hal::result<void, error_type> {
    return hal::result<void, error_type>::failure(
        detail_rtc::failure(native_error, operation));
  }

  detail::owned_fd fd_{};
  bool alarm_fault_{};
};

static_assert(hal::rtc::Clock<SystemClock>);
static_assert(hal::rtc::Alarm<SystemClockAlarm>);
static_assert(hal::rtc::Clock<HardwareClock<>>);
static_assert(hal::rtc::Alarm<HardwareClock<>>);

}  // namespace hal::linux::rtc
