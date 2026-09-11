#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <hal/foundation/result.hpp>
#include <hal/foundation/span.hpp>

namespace hal::i2c {

struct address7 {
  std::uint8_t value{};
  [[nodiscard]] constexpr bool valid() const noexcept { return value <= 0x7FU; }
};

struct address10 {
  std::uint16_t value{};
  [[nodiscard]] constexpr bool valid() const noexcept {
    return value <= 0x3FFU;
  }
};

enum class error_kind : std::uint8_t {
  no_acknowledge,
  arbitration_lost,
  bus_error,
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

enum class direction : std::uint8_t { write, read };

class operation {
 public:
  operation() = delete;

  [[nodiscard]] static constexpr operation write(
      span<const std::byte> buffer) noexcept {
    return operation{direction::write, buffer, {}};
  }

  [[nodiscard]] static constexpr operation read(
      span<std::byte> buffer) noexcept {
    return operation{direction::read, {}, buffer};
  }

  [[nodiscard]] constexpr direction dir() const noexcept { return dir_; }
  [[nodiscard]] constexpr span<const std::byte> write_buffer() const noexcept {
    return tx_;
  }
  [[nodiscard]] constexpr span<std::byte> read_buffer() const noexcept {
    return rx_;
  }

  [[nodiscard]] constexpr std::size_t size() const noexcept {
    return dir_ == direction::write ? tx_.size() : rx_.size();
  }

  [[nodiscard]] constexpr bool valid() const noexcept {
    return dir_ == direction::write ? tx_.valid() && rx_.empty()
                                    : rx_.valid() && tx_.empty();
  }

 private:
  constexpr operation(direction selected_direction, span<const std::byte> tx,
                      span<std::byte> rx) noexcept
      : dir_{selected_direction}, tx_{tx}, rx_{rx} {}

  direction dir_;
  span<const std::byte> tx_;
  span<std::byte> rx_;
};

template <class T>
concept Address = std::same_as<T, address7> || std::same_as<T, address10>;

template <class Bus, class AddressType>
concept Controller =
    Address<AddressType> && Error<typename Bus::error_type> &&
    requires(Bus& bus, AddressType address, span<const operation> operations) {
      {
        bus.transaction(address, operations)
      } -> std::same_as<result<void, typename Bus::error_type>>;
    };

template <class T>
concept Controller7 = Controller<T, address7>;

template <class T>
concept Controller10 = Controller<T, address10>;

template <class Bus, class AddressType>
  requires Controller<Bus, AddressType>
[[nodiscard]] inline auto write(Bus& bus, AddressType address,
                                span<const std::byte> data)
    -> result<void, typename Bus::error_type> {
  operation op = operation::write(data);
  return bus.transaction(address, span<const operation>{&op, 1U});
}

template <class Bus, class AddressType>
  requires Controller<Bus, AddressType>
[[nodiscard]] inline auto read(Bus& bus, AddressType address,
                               span<std::byte> data)
    -> result<void, typename Bus::error_type> {
  operation op = operation::read(data);
  return bus.transaction(address, span<const operation>{&op, 1U});
}

template <class Bus, class AddressType>
  requires Controller<Bus, AddressType>
[[nodiscard]] inline auto write_read(Bus& bus, AddressType address,
                                     span<const std::byte> tx,
                                     span<std::byte> rx)
    -> result<void, typename Bus::error_type> {
  operation operations[2] = {operation::write(tx), operation::read(rx)};
  return bus.transaction(address, span<const operation>{operations, 2U});
}

}  // namespace hal::i2c
