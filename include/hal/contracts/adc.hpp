#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <hal/foundation/result.hpp>
#include <hal/foundation/span.hpp>
#include <hal/foundation/units.hpp>

namespace hal::adc {

struct characteristics {
  std::uint8_t resolution_bits{};
  microvolts nominal_min{};
  microvolts nominal_max{};
};

template <class Value>
struct sample {
  Value value{};
  instant captured_at{};
  std::uint64_t sequence{};
};

using raw_sample = sample<std::uint32_t>;
using voltage_sample = sample<microvolts>;

struct stream_read {
  std::size_t count{};
  std::uint64_t dropped{};
};

enum class error_kind : std::uint8_t {
  not_ready,
  overrun,
  timeout,
  calibration,
  configuration,
  unavailable,
  io,
  other
};

template <class E>
concept Error = requires(const E& error) {
  { error.kind() } noexcept -> std::same_as<error_kind>;
};

template <class T>
concept Channel = Error<typename T::error_type> && requires(T& channel) {
  { T::properties() } noexcept -> std::same_as<characteristics>;
  {
    channel.read_raw()
  } -> std::same_as<result<raw_sample, typename T::error_type>>;
  {
    channel.read_voltage()
  } -> std::same_as<result<voltage_sample, typename T::error_type>>;
};

template <class T>
concept ContinuousChannel =
    Channel<T> && requires(T& channel, span<raw_sample> output) {
      {
        channel.try_read(output)
      } -> std::same_as<result<stream_read, typename T::error_type>>;
    };

}  // namespace hal::adc
