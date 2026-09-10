#include <hal/pwm/duty_cycle.hpp>

namespace {

struct Error {
  hal::pwm::error_kind kind() const noexcept;
};

struct Output {
  using error_type = Error;

  hal::result<hal::nanoseconds, error_type> configure(hal::pwm::config);
  hal::result<void, error_type> set_pulse_width(hal::nanoseconds width) {
    last_pulse = width;
    return hal::result<void, error_type>::success();
  }
  hal::result<void, error_type> enable_output();
  hal::result<void, error_type> disable_output();

  hal::nanoseconds last_pulse{};
};

static_assert(hal::pwm::Output<Output>);

}  // namespace

int main() {
  Output output;
  if (!hal::pwm::set_duty_cycle(output, hal::nanoseconds{1'001U},
                                hal::pwm::duty_cycle{500U}) ||
      output.last_pulse.value != 501U) {
    return 1;
  }
  if (!hal::pwm::set_duty_cycle(output, hal::nanoseconds{17U},
                                hal::pwm::duty_cycle{0U}) ||
      output.last_pulse.value != 0U) {
    return 2;
  }
  if (!hal::pwm::set_duty_cycle(output, hal::nanoseconds{17U},
                                hal::pwm::duty_cycle{1000U}) ||
      output.last_pulse.value != 17U) {
    return 3;
  }
  if (hal::pwm::duty_cycle{1001U}.valid()) {
    return 4;
  }
  return 0;
}
