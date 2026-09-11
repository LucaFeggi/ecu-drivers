#pragma once

#include <concepts>
#include <cstdint>
#include <hal/foundation/result.hpp>
#include <hal/foundation/units.hpp>

namespace hal::pwm {

enum class polarity : std::uint8_t { active_high, active_low };

struct config {
  nanoseconds requested_period{};
  std::uint32_t max_error_ppm{};
  polarity output_polarity{polarity::active_high};
};

enum class error_kind : std::uint8_t {
  unrepresentable,
  shared_period_conflict,
  not_configured,
  io,
  other
};

template <class E>
concept Error = requires(const E& error) {
  { error.kind() } noexcept -> std::same_as<error_kind>;
};

template <class T>
concept Output =
    Error<typename T::error_type> &&
    requires(T& output, config configuration, nanoseconds pulse) {
      {
        output.configure(configuration)
      } -> std::same_as<result<nanoseconds, typename T::error_type>>;
      {
        output.set_pulse_width(pulse)
      } -> std::same_as<result<void, typename T::error_type>>;
      {
        output.enable_output()
      } -> std::same_as<result<void, typename T::error_type>>;
      {
        output.disable_output()
      } -> std::same_as<result<void, typename T::error_type>>;
    };

template <class T>
concept ComplementaryOutput =
    Output<T> && requires(T& output, nanoseconds dead_time) {
      {
        output.set_dead_time(dead_time)
      } -> std::same_as<result<void, typename T::error_type>>;
    };

}  // namespace hal::pwm
