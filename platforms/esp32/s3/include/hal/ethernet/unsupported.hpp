#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/ethernet.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

namespace hal::esp32s3_wroom_1_n16r8::ethernet {

class error {
 public:
  [[nodiscard]] constexpr hal::ethernet::error_kind kind() const noexcept {
    return hal::ethernet::error_kind::link_down;
  }
};

// The ESP32-S3 has no native Ethernet MAC.  Keeping this explicit type in the
// target package makes accidental use fail at the call site while preserving
// a uniform firmware composition surface for boards with an external MAC.
class Unsupported {
 public:
  using error_type = error;

  [[nodiscard]] constexpr hal::ethernet::mac_address address() const noexcept {
    return {};
  }

  [[nodiscard]] result<hal::ethernet::link_state, error_type> link() noexcept {
    return result<hal::ethernet::link_state, error_type>::failure(error{});
  }

  [[nodiscard]] result<bool, error_type> try_transmit(
      span<const std::byte>) noexcept {
    return result<bool, error_type>::failure(error{});
  }

  [[nodiscard]] result<hal::ethernet::received_frame, error_type> try_receive(
      span<std::byte>) noexcept {
    return result<hal::ethernet::received_frame, error_type>::failure(error{});
  }
};

using FramePort = Unsupported;

}  // namespace hal::esp32s3_wroom_1_n16r8::ethernet
