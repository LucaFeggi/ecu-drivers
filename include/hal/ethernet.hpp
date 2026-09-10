#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstddef>
#include <hal/foundation/result.hpp>
#include <hal/foundation/span.hpp>

namespace hal::ethernet {

struct mac_address {
  std::array<std::byte, 6U> bytes{};
};

enum class speed : std::uint16_t {
  unknown = 0,
  mbps10 = 10,
  mbps100 = 100,
  mbps1000 = 1000
};
enum class duplex : std::uint8_t { half, full };

struct link_state {
  bool up{};
  speed link_speed{speed::unknown};
  duplex link_duplex{duplex::full};
};

struct received_frame {
  bool available{};
  std::size_t size{};
};

enum class error_kind : std::uint8_t {
  link_down,
  would_block,
  frame_too_large,
  io,
  other
};

template <class E>
concept Error = requires(const E& error) {
  { error.kind() } noexcept -> std::same_as<error_kind>;
};

template <class T>
concept MdioBus =
    Error<typename T::error_type> &&
    requires(T& bus, std::uint8_t phy, std::uint8_t reg, std::uint16_t value) {
      {
        bus.read_clause22(phy, reg)
      } -> std::same_as<result<std::uint16_t, typename T::error_type>>;
      {
        bus.write_clause22(phy, reg, value)
      } -> std::same_as<result<void, typename T::error_type>>;
    };

template <class T>
concept Phy = Error<typename T::error_type> && requires(T& phy) {
  { phy.link() } -> std::same_as<result<link_state, typename T::error_type>>;
  {
    phy.restart_autonegotiation()
  } -> std::same_as<result<void, typename T::error_type>>;
};

template <class T>
concept Mac =
    Error<typename T::error_type> &&
    requires(T& mac, mac_address address, link_state state, span<const std::byte> tx,
             span<std::byte> rx) {
      {
        mac.set_address(address)
      } -> std::same_as<result<void, typename T::error_type>>;
      {
        mac.configure_link(state)
      } -> std::same_as<result<void, typename T::error_type>>;
      {
        mac.try_transmit(tx)
      } -> std::same_as<result<bool, typename T::error_type>>;
      {
        mac.try_receive(rx)
      } -> std::same_as<result<received_frame, typename T::error_type>>;
    };

template <class T>
concept FramePort =
    Error<typename T::error_type> &&
    requires(T& port, span<const std::byte> tx, span<std::byte> rx) {
      { port.address() } noexcept -> std::same_as<mac_address>;
      {
        port.link()
      } -> std::same_as<result<link_state, typename T::error_type>>;
      {
        port.try_transmit(tx)
      } -> std::same_as<result<bool, typename T::error_type>>;
      {
        port.try_receive(rx)
      } -> std::same_as<result<received_frame, typename T::error_type>>;
    };

struct timestamped_frame {
  received_frame frame{};
  std::uint64_t timestamp_ns{};
};

template <class T>
concept PtpFramePort = FramePort<T> && requires(T& port, span<std::byte> rx) {
  {
    port.try_receive_timestamped(rx)
  } -> std::same_as<result<timestamped_frame, typename T::error_type>>;
};

}  // namespace hal::ethernet
