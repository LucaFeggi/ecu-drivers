#include <hal/pwm.hpp>

namespace {

struct Error {
  hal::pwm::error_kind kind() const noexcept;
};

struct Output {
  using error_type = Error;
  hal::result<hal::nanoseconds, error_type> configure(hal::pwm::config);
  hal::result<void, error_type> set_pulse_width(hal::nanoseconds);
  hal::result<void, error_type> enable_output();
  hal::result<void, error_type> disable_output();
  hal::result<void, error_type> set_dead_time(hal::nanoseconds);
};

static_assert(hal::pwm::Output<Output>);
static_assert(hal::pwm::ComplementaryOutput<Output>);

}  // namespace

int main() {}
