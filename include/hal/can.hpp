#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <hal/foundation/assert.hpp>
#include <hal/foundation/result.hpp>
#include <hal/foundation/span.hpp>
#include <new>
#include <type_traits>
#include <utility>

namespace hal::can {

struct id {
  std::uint32_t value{};
  bool extended{};

  [[nodiscard]] constexpr bool valid() const noexcept {
    return extended ? value <= 0x1FFFFFFFU : value <= 0x7FFU;
  }
};

enum class frame_validation_error : std::uint8_t { invalid_id, invalid_length };

// A validated Classic CAN data frame. Remote Transmission Request frames are
// intentionally not part of this portable contract.
class classic_frame {
 public:
  classic_frame(const classic_frame&) = default;
  classic_frame(classic_frame&&) noexcept = default;
  classic_frame& operator=(const classic_frame&) = default;
  classic_frame& operator=(classic_frame&&) noexcept = default;

  [[nodiscard]] static result<classic_frame, frame_validation_error> make(
      id frame_id, span<const std::byte> payload) noexcept {
    HAL_CORE_ASSERT(payload.valid());
    if (!frame_id.valid()) {
      return result<classic_frame, frame_validation_error>::failure(
          frame_validation_error::invalid_id);
    }
    if (!payload.valid() || payload.size() > 8U) {
      return result<classic_frame, frame_validation_error>::failure(
          frame_validation_error::invalid_length);
    }

    classic_frame value{init_tag{}};
    value.id_ = frame_id;
    value.size_ = static_cast<std::uint8_t>(payload.size());
    for (std::size_t index = 0U; index < payload.size(); ++index) {
      value.data_[index] = payload[index];
    }
    return result<classic_frame, frame_validation_error>::success(
        std::move(value));
  }

  [[nodiscard]] constexpr id frame_id() const noexcept { return id_; }
  [[nodiscard]] constexpr std::uint8_t size() const noexcept { return size_; }
  [[nodiscard]] constexpr span<const std::byte> data() const noexcept {
    return {data_.data(), size_};
  }

 private:
  struct init_tag {};
  explicit constexpr classic_frame(init_tag) noexcept {}

  id id_{};
  std::uint8_t size_{};
  std::array<std::byte, 8U> data_{};
};

class fd_frame {
 public:
  fd_frame(const fd_frame&) = default;
  fd_frame(fd_frame&&) noexcept = default;
  fd_frame& operator=(const fd_frame&) = default;
  fd_frame& operator=(fd_frame&&) noexcept = default;

  [[nodiscard]] static constexpr bool valid_payload_length(
      std::size_t length) noexcept {
    return length <= 8U || length == 12U || length == 16U || length == 20U ||
           length == 24U || length == 32U || length == 48U || length == 64U;
  }

  [[nodiscard]] static result<fd_frame, frame_validation_error> make(
      id frame_id, span<const std::byte> payload, bool bit_rate_switch = false,
      bool error_state_indicator = false) noexcept {
    HAL_CORE_ASSERT(payload.valid());
    if (!frame_id.valid()) {
      return result<fd_frame, frame_validation_error>::failure(
          frame_validation_error::invalid_id);
    }
    if (!payload.valid() || !valid_payload_length(payload.size())) {
      return result<fd_frame, frame_validation_error>::failure(
          frame_validation_error::invalid_length);
    }

    fd_frame value{init_tag{}};
    value.id_ = frame_id;
    value.bit_rate_switch_ = bit_rate_switch;
    value.error_state_indicator_ = error_state_indicator;
    value.size_ = static_cast<std::uint8_t>(payload.size());
    for (std::size_t index = 0U; index < payload.size(); ++index) {
      value.data_[index] = payload[index];
    }
    return result<fd_frame, frame_validation_error>::success(std::move(value));
  }

  [[nodiscard]] constexpr id frame_id() const noexcept { return id_; }
  [[nodiscard]] constexpr bool bit_rate_switch() const noexcept {
    return bit_rate_switch_;
  }
  [[nodiscard]] constexpr bool error_state_indicator() const noexcept {
    return error_state_indicator_;
  }
  [[nodiscard]] constexpr std::uint8_t size() const noexcept { return size_; }
  [[nodiscard]] constexpr span<const std::byte> data() const noexcept {
    return {data_.data(), size_};
  }

  [[nodiscard]] constexpr std::uint8_t dlc() const noexcept {
    return size_ <= 8U    ? size_
           : size_ == 12U ? std::uint8_t{9U}
           : size_ == 16U ? std::uint8_t{10U}
           : size_ == 20U ? std::uint8_t{11U}
           : size_ == 24U ? std::uint8_t{12U}
           : size_ == 32U ? std::uint8_t{13U}
           : size_ == 48U ? std::uint8_t{14U}
                          : std::uint8_t{15U};
  }

 private:
  struct init_tag {};
  explicit constexpr fd_frame(init_tag) noexcept {}

  id id_{};
  bool bit_rate_switch_{};
  bool error_state_indicator_{};
  std::uint8_t size_{};
  std::array<std::byte, 64U> data_{};
};

// The value returned by a nonblocking receive operation. It represents a
// normal empty receive queue without treating it as an error. has_frame() is
// false when no frame was waiting; value() is valid only when it is true.
template <class Frame>
class polled_frame {
 public:
  [[nodiscard]] static polled_frame unavailable() noexcept {
    return polled_frame(none_tag{});
  }

  [[nodiscard]] static polled_frame available(Frame value) noexcept(
      std::is_nothrow_move_constructible_v<Frame>) {
    return polled_frame(some_tag{}, std::move(value));
  }

  polled_frame(const polled_frame& other) noexcept(
      std::is_nothrow_copy_constructible_v<Frame>)
    requires std::copy_constructible<Frame>
      : available_{other.available_} {
    if (available_) {
      ::new (static_cast<void*>(&storage_.value)) Frame(other.storage_.value);
    }
  }

  polled_frame(const polled_frame&)
    requires(!std::copy_constructible<Frame>)
  = delete;

  polled_frame(polled_frame&& other) noexcept(
      std::is_nothrow_move_constructible_v<Frame>)
    requires std::move_constructible<Frame>
      : available_{other.available_} {
    if (available_) {
      ::new (static_cast<void*>(&storage_.value))
          Frame(std::move(other.storage_.value));
    }
  }

  polled_frame(polled_frame&&)
    requires(!std::move_constructible<Frame>)
  = delete;
  polled_frame& operator=(const polled_frame&) = delete;
  polled_frame& operator=(polled_frame&&) = delete;

  ~polled_frame() {
    if (available_) {
      storage_.value.~Frame();
    }
  }

  [[nodiscard]] constexpr bool has_frame() const noexcept { return available_; }

  [[nodiscard]] Frame& value() & noexcept {
    HAL_CORE_ASSERT(available_);
    return storage_.value;
  }

  [[nodiscard]] const Frame& value() const& noexcept {
    HAL_CORE_ASSERT(available_);
    return storage_.value;
  }

 private:
  struct none_tag {};
  struct some_tag {};

  union storage_t {
    Frame value;

    constexpr storage_t() noexcept {}
    ~storage_t() {}
  } storage_;

  bool available_{};

  explicit polled_frame(none_tag) noexcept : available_{false} {}

  polled_frame(some_tag, Frame value) noexcept(
      std::is_nothrow_move_constructible_v<Frame>)
      : available_{true} {
    ::new (static_cast<void*>(&storage_.value)) Frame(std::move(value));
  }
};

enum class error_kind : std::uint8_t {
  bus_off,
  arbitration_lost,
  protocol_error,
  tx_overflow,
  rx_overflow,
  configuration,
  io,
  other
};

template <class E>
concept Error = requires(const E& error) {
  { error.kind() } noexcept -> std::same_as<error_kind>;
};

// A nonblocking Classic CAN controller. A successful false from try_send()
// means that no transmit slot is currently available; it is not an error.
template <class T>
concept ClassicController =
    Error<typename T::error_type> &&
    requires(T& controller, const classic_frame& tx) {
      {
        controller.try_send(tx)
      } -> std::same_as<result<bool, typename T::error_type>>;
      {
        controller.try_receive()
      } -> std::same_as<
          result<polled_frame<classic_frame>, typename T::error_type>>;
    };

template <class T>
concept FdController =
    ClassicController<T> && requires(T& controller, const fd_frame& tx) {
      {
        controller.try_send_fd(tx)
      } -> std::same_as<result<bool, typename T::error_type>>;
      {
        controller.try_receive_fd()
      } -> std::same_as<result<polled_frame<fd_frame>, typename T::error_type>>;
    };

struct filter {
  id frame_id{};
  std::uint32_t mask{};

  [[nodiscard]] constexpr bool valid() const noexcept {
    const std::uint32_t width_mask = frame_id.extended ? 0x1FFFFFFFU : 0x7FFU;
    return frame_id.valid() && (mask & ~width_mask) == 0U;
  }

  [[nodiscard]] constexpr bool matches(id candidate) const noexcept {
    return valid() && candidate.valid() &&
           candidate.extended == frame_id.extended &&
           ((candidate.value & mask) == (frame_id.value & mask));
  }
};

template <class T>
concept FilterableController =
    ClassicController<T> &&
    requires(T& controller, span<const filter> filters) {
      {
        controller.set_filters(filters)
      } -> std::same_as<result<void, typename T::error_type>>;
    };

}  // namespace hal::can
