#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <hal/foundation/result.hpp>
#include <hal/foundation/span.hpp>
#include <hal/foundation/units.hpp>

namespace hal::serial {

enum class data_bits : std::uint8_t { bits7 = 7, bits8 = 8, bits9 = 9 };
enum class parity : std::uint8_t { none, even, odd };
enum class stop_bits : std::uint8_t { one, two };
enum class flow_control : std::uint8_t { none, rts, cts, rts_cts };

struct config {
  hertz baud_rate{};
  data_bits bits{data_bits::bits8};
  parity parity_mode{parity::none};
  stop_bits stops{stop_bits::one};
  flow_control flow{flow_control::none};
};

enum class error_kind : std::uint8_t {
  overrun,
  framing,
  parity,
  break_detected,
  configuration,
  timeout,
  io,
  other
};

template <class E>
concept Error = requires(const E& error) {
  { error.kind() } noexcept -> std::same_as<error_kind>;
};

template <class T>
concept Tx = Error<typename T::error_type> &&
             requires(T& serial, span<const std::byte> data) {
               {
                 serial.try_write(data)
               } -> std::same_as<result<std::size_t, typename T::error_type>>;
               {
                 serial.flush()
               } -> std::same_as<result<void, typename T::error_type>>;
             };

template <class T>
concept Rx =
    Error<typename T::error_type> && requires(T& serial, span<std::byte> data) {
      {
        serial.try_read(data)
      } -> std::same_as<result<std::size_t, typename T::error_type>>;
    };

template <class T>
concept Port = Tx<T> && Rx<T>;

template <class T>
concept ConfigurablePort =
    Port<T> && requires(T& serial, config configuration) {
      {
        serial.configure(configuration)
      } -> std::same_as<result<hertz, typename T::error_type>>;
    };

}  // namespace hal::serial
