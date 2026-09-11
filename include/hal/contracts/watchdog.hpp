#pragma once

#include <concepts>
#include <cstdint>
#include <hal/foundation/result.hpp>
#include <hal/foundation/units.hpp>

namespace hal::watchdog {

enum class error_kind : std::uint8_t { too_early, hardware_fault, io, other };

template <class E>
concept Error = requires(const E& error) {
  { error.kind() } noexcept -> std::same_as<error_kind>;
};

struct characteristics {
  nanoseconds minimum_feed_interval{};
  nanoseconds maximum_feed_interval{};
  bool oscillator_independent{};
};

template <class T>
concept Feeder = Error<typename T::error_type> && requires(T& feeder) {
  { T::properties() } noexcept -> std::same_as<characteristics>;
  { feeder.feed() } -> std::same_as<result<void, typename T::error_type>>;
};

}  // namespace hal::watchdog
