#pragma once

#include <cstdint>
#include <hal/foundation/assert.hpp>
#include <hal/pwm.hpp>

namespace hal::pwm {

// Integer duty-cycle ratio. 0 is always inactive and 1000 is always active.
struct duty_cycle {
  std::uint16_t permille{};

  [[nodiscard]] constexpr bool valid() const noexcept {
    return permille <= 1000U;
  }
};

// Converts a valid permille ratio into a pulse width using the actual period
// returned by Output::configure(). The conversion rounds to the nearest
// nanosecond and does not use floating-point arithmetic.
template <Output Output>
[[nodiscard]] auto set_duty_cycle(Output& output, nanoseconds actual_period,
                                  duty_cycle duty)
    -> result<void, typename Output::error_type> {
  HAL_CORE_ASSERT(duty.valid());

  constexpr std::uint64_t scale = 1000U;
  const std::uint64_t whole_periods = actual_period.value / scale;
  const std::uint64_t remainder = actual_period.value % scale;
  const std::uint64_t pulse =
      whole_periods * duty.permille +
      ((remainder * duty.permille + (scale / 2U)) / scale);
  return output.set_pulse_width(nanoseconds{pulse});
}

}  // namespace hal::pwm
