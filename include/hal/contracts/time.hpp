#pragma once

#include <concepts>
#include <hal/foundation/result.hpp>
#include <hal/foundation/units.hpp>

namespace hal::time {

template <class T>
concept MonotonicClock = requires(T& clock) {
  { clock.now() } noexcept -> std::same_as<instant>;
};

template <class T>
concept Delay = requires(T& delay, nanoseconds duration) {
  { delay.delay_for(duration) } noexcept -> std::same_as<void>;
};

template <class T>
concept OneShotAlarm =
    MonotonicClock<T> && requires(T& alarm, instant deadline) {
      typename T::error_type;
      {
        alarm.arm(deadline)
      } -> std::same_as<result<void, typename T::error_type>>;
      { alarm.cancel() } -> std::same_as<result<void, typename T::error_type>>;
      { alarm.expired() } noexcept -> std::same_as<bool>;
    };

}  // namespace hal::time
