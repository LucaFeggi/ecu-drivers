#pragma once

#include <concepts>
#include <cstdint>
#include <hal/foundation/result.hpp>
#include <hal/foundation/units.hpp>

namespace hal::rtc {

enum class error_kind : std::uint8_t {
  not_set,
  out_of_range,
  clock_failure,
  io,
  other
};

template <class E>
concept Error = requires(const E& error) {
  { error.kind() } noexcept -> std::same_as<error_kind>;
};

struct persistence {
  bool survives_warm_reset{};
  bool survives_deep_sleep{};
  bool can_survive_main_power_loss_with_backup_supply{};
};

template <class T>
concept Clock =
    Error<typename T::error_type> && requires(T& clock, utc_time value) {
      {
        T::persistence_characteristics()
      } noexcept -> std::same_as<persistence>;
      {
        clock.read()
      } -> std::same_as<result<utc_time, typename T::error_type>>;
      {
        clock.set(value)
      } -> std::same_as<result<void, typename T::error_type>>;
    };

template <class T>
concept Alarm = Clock<T> && requires(T& clock, utc_time value) {
  {
    clock.set_alarm(value)
  } -> std::same_as<result<void, typename T::error_type>>;
  { clock.alarm_pending() } noexcept -> std::same_as<bool>;
  { clock.clear_alarm() } noexcept -> std::same_as<void>;
};

}  // namespace hal::rtc
