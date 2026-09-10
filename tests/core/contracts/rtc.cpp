#include <hal/rtc.hpp>

namespace {

struct Error {
  hal::rtc::error_kind kind() const noexcept;
};

struct Clock {
  using error_type = Error;
  static hal::rtc::persistence persistence_characteristics() noexcept;
  hal::result<hal::utc_time, error_type> read();
  hal::result<void, error_type> set(hal::utc_time);
  hal::result<void, error_type> set_alarm(hal::utc_time);
  bool alarm_pending() noexcept;
  void clear_alarm() noexcept;
};

static_assert(hal::rtc::Clock<Clock>);
static_assert(hal::rtc::Alarm<Clock>);

}  // namespace

int main() {}
