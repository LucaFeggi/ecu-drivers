#pragma once

#include <cstdint>
#include <hal/rtc.hpp>
#include <hal/stm32h7/support.hpp>

namespace hal::stm32h7::rtc {

inline constexpr capability support{
    implementation_status::available,
    "direct BCD RTC calendar and alarm access"};

class error {
public:
  explicit constexpr error(hal::rtc::error_kind kind) noexcept : kind_{kind} {}

  [[nodiscard]] constexpr hal::rtc::error_kind kind() const noexcept {
    return kind_;
  }

private:
  hal::rtc::error_kind kind_{};
};

struct backup_domain_persistence {
  [[nodiscard]] static constexpr hal::rtc::persistence value() noexcept {
    // Backup-domain retention depends on the board's VBAT/LSE design. The
    // reusable target must not promise alarm survival without that BSP fact.
    return {true, true, false};
  }
};

template <class Registers,
          class PersistencePolicy = backup_domain_persistence>
class Clock {
public:
  using error_type = error;

  explicit Clock(Registers &registers,
                 poll_budget timeout = {100'000U}) noexcept
      : registers_{registers}, timeout_{timeout} {}

  Clock(const Clock &) = delete;
  Clock &operator=(const Clock &) = delete;

  [[nodiscard]] static constexpr hal::rtc::persistence
  persistence_characteristics() noexcept {
    return PersistencePolicy::value();
  }

  [[nodiscard]] auto read() -> result<hal::utc_time, error_type> {
    if (!timeout_.valid()) {
      return failure_time(hal::rtc::error_kind::clock_failure);
    }
    if ((registers_.ISR & flag_rsf) == 0U) {
      registers_.ISR = registers_.ISR & ~flag_rsf;
      bool synchronized = false;
      for (std::uint32_t remaining = timeout_.iterations; remaining > 0U;
           --remaining) {
        if ((registers_.ISR & flag_rsf) != 0U) {
          synchronized = true;
          break;
        }
      }
      if (!synchronized) {
        return failure_time(hal::rtc::error_kind::clock_failure);
      }
    }
    const std::uint32_t time = registers_.TR;
    const std::uint32_t date = registers_.DR;
    const int year = 2000 + static_cast<int>(bcd_to_binary((date >> 16U) & 0xFFU));
    const unsigned month = bcd_to_binary((date >> 8U) & 0x1FU);
    const unsigned day = bcd_to_binary(date & 0x3FU);
    unsigned hour = bcd_to_binary(((time >> 16U) & 0x3FU));
    if ((registers_.CR & control_format_12_hour) != 0U) {
      const bool afternoon = (time & time_pm) != 0U;
      if (hour == 12U) {
        hour = afternoon ? 12U : 0U;
      } else if (afternoon) {
        hour += 12U;
      }
    }
    const unsigned minute = bcd_to_binary((time >> 8U) & 0x7FU);
    const unsigned second = bcd_to_binary(time & 0x7FU);
    if (!valid_date(year, month, day) || hour > 23U || minute > 59U ||
        second > 59U) {
      return failure_time(hal::rtc::error_kind::not_set);
    }
    const std::int64_t days = days_from_civil(year, month, day);
    return result<hal::utc_time, error_type>::success(
        {days * 86'400LL + static_cast<std::int64_t>(hour) * 3'600LL +
             static_cast<std::int64_t>(minute) * 60LL + second,
         0U});
  }

  [[nodiscard]] auto set(hal::utc_time value) -> result<void, error_type> {
    int year{};
    unsigned month{};
    unsigned day{};
    unsigned hour{};
    unsigned minute{};
    unsigned second{};
    if (!split(value, year, month, day, hour, minute, second) ||
        !timeout_.valid()) {
      return result<void, error_type>::failure(
          error{hal::rtc::error_kind::out_of_range});
    }
    return write_calendar(year, month, day, hour, minute, second);
  }

  [[nodiscard]] auto set_alarm(hal::utc_time value) -> result<void, error_type> {
    int year{};
    unsigned month{};
    unsigned day{};
    unsigned hour{};
    unsigned minute{};
    unsigned second{};
    if (!split(value, year, month, day, hour, minute, second) ||
        !timeout_.valid()) {
      return result<void, error_type>::failure(
          error{hal::rtc::error_kind::out_of_range});
    }
    (void)year;
    (void)month;
    write_protected(false);
    registers_.CR = registers_.CR & ~control_format_12_hour;
    registers_.CR = registers_.CR & ~control_alarm_enable;
    registers_.ISR = registers_.ISR & ~flag_alarm;
    registers_.ALRMAR = binary_to_bcd(second) |
                        (binary_to_bcd(minute) << 8U) |
                        (binary_to_bcd(hour) << 16U) |
                        (binary_to_bcd(day) << 24U);
    registers_.CR = registers_.CR | control_alarm_interrupt | control_alarm_enable;
    write_protected(true);
    return result<void, error_type>::success();
  }

  [[nodiscard]] bool alarm_pending() noexcept {
    return (registers_.ISR & flag_alarm) != 0U;
  }

  void clear_alarm() noexcept {
    write_protected(false);
    registers_.ISR = registers_.ISR & ~flag_alarm;
    write_protected(true);
  }

private:
  static constexpr std::uint32_t flag_rsf{1U << 5U};
  static constexpr std::uint32_t flag_initf{1U << 6U};
  static constexpr std::uint32_t flag_init{1U << 7U};
  static constexpr std::uint32_t flag_alarm{1U << 8U};
  static constexpr std::uint32_t control_alarm_enable{1U << 8U};
  static constexpr std::uint32_t control_alarm_interrupt{1U << 12U};
  static constexpr std::uint32_t control_format_12_hour{1U << 6U};
  static constexpr std::uint32_t time_pm{1U << 22U};

  [[nodiscard]] static constexpr unsigned bcd_to_binary(std::uint32_t value)
      noexcept {
    return static_cast<unsigned>((value & 0x0FU) + ((value >> 4U) & 0x0FU) * 10U);
  }

  [[nodiscard]] static constexpr std::uint32_t binary_to_bcd(unsigned value)
      noexcept {
    return static_cast<std::uint32_t>(((value / 10U) << 4U) | (value % 10U));
  }

  [[nodiscard]] static constexpr bool leap(int year) noexcept {
    return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
  }

  [[nodiscard]] static constexpr unsigned days_in_month(int year,
                                                        unsigned month) noexcept {
    constexpr unsigned days[] = {31U, 28U, 31U, 30U, 31U, 30U,
                                  31U, 31U, 30U, 31U, 30U, 31U};
    return month == 2U && leap(year) ? 29U : days[month - 1U];
  }

  [[nodiscard]] static constexpr bool valid_date(int year, unsigned month,
                                                 unsigned day) noexcept {
    return year >= 2000 && year <= 2099 && month >= 1U && month <= 12U &&
           day >= 1U && day <= days_in_month(year, month);
  }

  [[nodiscard]] static constexpr std::int64_t days_from_civil(
      int year, unsigned month, unsigned day) noexcept {
    year -= month <= 2U ? 1 : 0;
    const int era = (year >= 0 ? year : year - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(year - era * 400);
    const unsigned mp = month > 2U ? month - 3U : month + 9U;
    const unsigned doy = (153U * mp + 2U) / 5U + day - 1U;
    const unsigned doe = yoe * 365U + yoe / 4U - yoe / 100U + doy;
    return static_cast<std::int64_t>(era) * 146097LL +
           static_cast<std::int64_t>(doe) - 719468LL;
  }

  [[nodiscard]] static bool split(hal::utc_time value, int &year, unsigned &month,
                                  unsigned &day, unsigned &hour, unsigned &minute,
                                  unsigned &second) noexcept {
    if (value.nanoseconds >= 1'000'000'000U) {
      return false;
    }
    std::int64_t days = value.seconds / 86'400LL;
    std::int64_t remainder = value.seconds % 86'400LL;
    if (remainder < 0) {
      --days;
      remainder += 86'400LL;
    }
    // Invert days_from_civil using a bounded search over the RTC-supported
    // calendar. This runs only on set/alarm configuration, never on read.
    for (int candidate_year = 2000; candidate_year <= 2099; ++candidate_year) {
      for (unsigned candidate_month = 1U; candidate_month <= 12U;
           ++candidate_month) {
        const std::int64_t first =
            days_from_civil(candidate_year, candidate_month, 1U);
        const unsigned length = days_in_month(candidate_year, candidate_month);
        if (days >= first && days < first + static_cast<std::int64_t>(length)) {
          year = candidate_year;
          month = candidate_month;
          day = static_cast<unsigned>(days - first) + 1U;
          hour = static_cast<unsigned>(remainder / 3'600LL);
          minute = static_cast<unsigned>((remainder % 3'600LL) / 60LL);
          second = static_cast<unsigned>(remainder % 60LL);
          return true;
        }
      }
    }
    return false;
  }

  [[nodiscard]] auto write_calendar(int year, unsigned month, unsigned day,
                                    unsigned hour, unsigned minute,
                                    unsigned second)
      -> result<void, error_type> {
    write_protected(false);
    registers_.CR = registers_.CR & ~control_format_12_hour;
    registers_.ISR = registers_.ISR & ~flag_init;
    registers_.ISR = registers_.ISR | flag_init;
    bool initialized = false;
    for (std::uint32_t remaining = timeout_.iterations; remaining > 0U; --remaining) {
      if ((registers_.ISR & flag_initf) != 0U) {
        initialized = true;
        break;
      }
    }
    if (!initialized) {
      write_protected(true);
      return result<void, error_type>::failure(
          error{hal::rtc::error_kind::clock_failure});
    }
    registers_.TR = binary_to_bcd(second) | (binary_to_bcd(minute) << 8U) |
                    (binary_to_bcd(hour) << 16U);
    registers_.DR = binary_to_bcd(day) | (binary_to_bcd(month) << 8U) |
                    (binary_to_bcd(static_cast<unsigned>(year - 2000)) << 16U);
    registers_.ISR = registers_.ISR & ~flag_init;
    write_protected(true);
    return result<void, error_type>::success();
  }

  void write_protected(bool enabled) noexcept {
    registers_.WPR = enabled ? 0xFFU : 0xCAU;
    if (!enabled) {
      registers_.WPR = 0x53U;
    }
  }

  [[nodiscard]] static auto failure_time(hal::rtc::error_kind kind)
      -> result<hal::utc_time, error_type> {
    return result<hal::utc_time, error_type>::failure(error{kind});
  }

  Registers &registers_;
  poll_budget timeout_{};
};

} // namespace hal::stm32h7::rtc
