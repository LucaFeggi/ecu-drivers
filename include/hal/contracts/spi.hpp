#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <hal/foundation/result.hpp>
#include <hal/foundation/span.hpp>
#include <hal/foundation/units.hpp>

namespace hal::spi {

enum class error_kind : std::uint8_t {
  timeout,
  busy,
  configuration,
  io,
  other
};

template <class E>
concept Error = requires(const E& error) {
  { error.kind() } noexcept -> std::same_as<error_kind>;
};

enum class operation_kind : std::uint8_t {
  write,
  read,
  transfer,
  delay
};

class operation8 {
 public:
  operation8() = delete;
  [[nodiscard]] static constexpr operation8 write(
      span<const std::byte> data) noexcept {
    return operation8{operation_kind::write, data, {}, {}};
  }
  [[nodiscard]] static constexpr operation8 read(
      span<std::byte> data) noexcept {
    return operation8{operation_kind::read, {}, data, {}};
  }
  [[nodiscard]] static constexpr operation8 transfer(
      span<const std::byte> tx, span<std::byte> rx) noexcept {
    return operation8{operation_kind::transfer, tx, rx, {}};
  }
  // Retained because Linux spidev and some device protocols require a pause
  // while chip select remains asserted. Omit it from a device-specific API
  // when that protocol has no such timing requirement.
  [[nodiscard]] static constexpr operation8 delay_for(
      nanoseconds duration) noexcept {
    return operation8{operation_kind::delay, {}, {}, duration};
  }
  [[nodiscard]] constexpr operation_kind kind() const noexcept { return kind_; }
  [[nodiscard]] constexpr span<const std::byte> tx() const noexcept {
    return tx_;
  }
  [[nodiscard]] constexpr span<std::byte> rx() const noexcept { return rx_; }
  [[nodiscard]] constexpr nanoseconds delay() const noexcept { return delay_; }
  [[nodiscard]] constexpr bool valid() const noexcept {
    switch (kind_) {
      case operation_kind::write:
        return tx_.valid() && rx_.empty();
      case operation_kind::read:
        return tx_.empty() && rx_.valid();
      case operation_kind::transfer:
        return tx_.valid() && rx_.valid();
      case operation_kind::delay:
        return tx_.empty() && rx_.empty();
    }
    return false;
  }

 private:
  constexpr operation8(operation_kind kind, span<const std::byte> tx,
                       span<std::byte> rx, nanoseconds delay) noexcept
      : kind_{kind}, tx_{tx}, rx_{rx}, delay_{delay} {}
  operation_kind kind_;
  span<const std::byte> tx_;
  span<std::byte> rx_;
  nanoseconds delay_{};
};

template <class T>
concept Device8 = Error<typename T::error_type> &&
                  requires(T& device, span<const operation8> operations) {
                    {
                      device.transaction(operations)
                    } -> std::same_as<result<void, typename T::error_type>>;
                  };

}  // namespace hal::spi
