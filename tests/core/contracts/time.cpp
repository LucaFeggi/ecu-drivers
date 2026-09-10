#include <hal/time.hpp>

namespace {

struct TimeError {};

struct Alarm {
  using error_type = TimeError;
  hal::instant now() noexcept;
  hal::result<void, error_type> arm(hal::instant);
  hal::result<void, error_type> cancel();
  bool expired() noexcept;
};

struct Delay {
  void delay_for(hal::nanoseconds) noexcept;
};

static_assert(hal::time::MonotonicClock<Alarm>);
static_assert(hal::time::OneShotAlarm<Alarm>);
static_assert(hal::time::Delay<Delay>);

}  // namespace

int main() {}
