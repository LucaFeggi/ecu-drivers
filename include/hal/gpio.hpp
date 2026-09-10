#pragma once

#include <concepts>
#include <cstdint>
#include <hal/foundation/result.hpp>

namespace hal::gpio {
enum class level : std::uint8_t { low, high };
enum class edge : std::uint8_t { rising, falling, both };
enum class error_kind : std::uint8_t { io, unavailable, other };
template <class E>
concept Error = requires(const E& error) {
  { error.kind() } noexcept -> std::same_as<error_kind>;
};
template <class T>
concept OutputPin =
    Error<typename T::error_type> && requires(T& pin, level value) {
      {
        pin.write(value)
      } -> std::same_as<result<void, typename T::error_type>>;
    };
template <class T>
concept InputPin = Error<typename T::error_type> && requires(T& pin) {
  { pin.read() } -> std::same_as<result<level, typename T::error_type>>;
};
template <class T>
concept StatefulOutputPin = OutputPin<T> && requires(T& pin) {
  { pin.output_latch() } -> std::same_as<result<level, typename T::error_type>>;
};
template <class T>
concept EdgeInput = InputPin<T> && requires(T& pin, edge selected_edge) {
  {
    pin.configure_edge(selected_edge)
  } -> std::same_as<result<void, typename T::error_type>>;
  { pin.take_event() } noexcept -> std::same_as<bool>;
};
}  // namespace hal::gpio
