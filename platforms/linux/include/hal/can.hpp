#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/can.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <hal/foundation/result.hpp>
#include <hal/foundation/span.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <unistd.h>
#include <utility>

#include <hal/detail/error.hpp>
#include <hal/detail/fd.hpp>

namespace hal::linux::can {

using error_type = detail::error<hal::can::error_kind>;

struct config {
  const char* interface_name{};
  bool loopback{true};
  bool receive_own_messages{};
  bool receive_error_frames{};
};

namespace detail_can {

[[nodiscard]] inline hal::can::error_kind map_errno(int native_error) noexcept {
  switch (native_error) {
    case ENOBUFS:
      return hal::can::error_kind::tx_overflow;
    case ENETDOWN:
    case ENOLINK:
      return hal::can::error_kind::bus_off;
    case EAGAIN:
      return hal::can::error_kind::other;
    case EPROTO:
      return hal::can::error_kind::protocol_error;
    case EINVAL:
      return hal::can::error_kind::configuration;
    default:
      return hal::can::error_kind::io;
  }
}

[[nodiscard]] inline error_type failure(int native_error,
                                        detail::operation operation) noexcept {
  return error_type{map_errno(native_error), native_error, operation};
}

[[nodiscard]] inline canid_t encode_id(hal::can::id value) noexcept {
  return static_cast<canid_t>(value.value) |
         (value.extended ? CAN_EFF_FLAG : 0U);
}

[[nodiscard]] inline hal::can::id decode_id(canid_t value) noexcept {
  return {static_cast<std::uint32_t>(value & CAN_EFF_MASK),
          (value & CAN_EFF_FLAG) != 0U};
}

[[nodiscard]] inline std::uint8_t dlc_to_length(std::uint8_t dlc) noexcept {
  constexpr std::array<std::uint8_t, 16U> lengths{
      0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U,
      8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U};
  return lengths[dlc & 0x0FU];
}

}  // namespace detail_can

template <bool EnableFd = true, std::size_t MaxFilters = 16U>
class Controller {
  static_assert(MaxFilters > 0U);

 public:
  using error_type = linux::can::error_type;

  [[nodiscard]] static auto open(config configuration)
      -> hal::result<Controller, error_type> {
    if (configuration.interface_name == nullptr) {
      return failure(EINVAL, detail::operation::open);
    }
    const int fd = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC,
                            CAN_RAW);
    if (fd < 0) {
      return failure(errno, detail::operation::open);
    }

    const int loopback = configuration.loopback ? 1 : 0;
    if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback,
                     sizeof(loopback)) < 0) {
      const int native_error = errno;
      (void)::close(fd);
      return failure(native_error, detail::operation::configure);
    }
    const int receive_own = configuration.receive_own_messages ? 1 : 0;
    if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &receive_own,
                     sizeof(receive_own)) < 0) {
      const int native_error = errno;
      (void)::close(fd);
      return failure(native_error, detail::operation::configure);
    }
    if constexpr (EnableFd) {
      const int enabled = 1;
      if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enabled,
                       sizeof(enabled)) < 0) {
        const int native_error = errno;
        (void)::close(fd);
        return failure(native_error, detail::operation::configure);
      }
    }
    if (configuration.receive_error_frames) {
      const can_err_mask_t mask = CAN_ERR_MASK;
      if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &mask,
                       sizeof(mask)) < 0) {
        const int native_error = errno;
        (void)::close(fd);
        return failure(native_error, detail::operation::configure);
      }
    }

    const unsigned int index = ::if_nametoindex(configuration.interface_name);
    if (index == 0U) {
      const int native_error = errno == 0 ? ENODEV : errno;
      (void)::close(fd);
      return failure(native_error, detail::operation::configure);
    }
    ::sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = static_cast<int>(index);
    if (::bind(fd, reinterpret_cast<const ::sockaddr*>(&address),
               sizeof(address)) < 0) {
      const int native_error = errno;
      (void)::close(fd);
      return failure(native_error, detail::operation::configure);
    }
    return hal::result<Controller, error_type>::success(
        Controller{detail::owned_fd{fd}});
  }

  Controller(const Controller&) = delete;
  Controller& operator=(const Controller&) = delete;
  Controller(Controller&&) noexcept = default;
  Controller& operator=(Controller&&) noexcept = default;

  [[nodiscard]] auto try_send(const hal::can::classic_frame& frame)
      -> hal::result<bool, error_type> {
    ::can_frame value{};
    value.can_id = detail_can::encode_id(frame.frame_id());
    value.can_dlc = frame.size();
    if (!frame.data().empty()) {
      std::memcpy(value.data, frame.data().data(), frame.data().size());
    }
    const ssize_t count = ::send(fd_.get(), &value, CAN_MTU,
                                 MSG_DONTWAIT | MSG_NOSIGNAL);
    if (count == CAN_MTU) {
      return hal::result<bool, error_type>::success(true);
    }
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return hal::result<bool, error_type>::success(false);
    }
    return hal::result<bool, error_type>::failure(
        detail_can::failure(count < 0 ? errno : EIO, detail::operation::write));
  }

  template <bool Enabled = EnableFd>
    requires Enabled
  [[nodiscard]] auto try_send_fd(const hal::can::fd_frame& frame)
      -> hal::result<bool, error_type> {
    ::canfd_frame value{};
    value.can_id = detail_can::encode_id(frame.frame_id());
    value.len = frame.size();
    value.flags = static_cast<__u8>((frame.bit_rate_switch() ? CANFD_BRS : 0U) |
                                    (frame.error_state_indicator() ? CANFD_ESI : 0U));
    if (!frame.data().empty()) {
      std::memcpy(value.data, frame.data().data(), frame.data().size());
    }
    const ssize_t count = ::send(fd_.get(), &value, CANFD_MTU,
                                 MSG_DONTWAIT | MSG_NOSIGNAL);
    if (count == CANFD_MTU) {
      return hal::result<bool, error_type>::success(true);
    }
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return hal::result<bool, error_type>::success(false);
    }
    return hal::result<bool, error_type>::failure(
        detail_can::failure(count < 0 ? errno : EIO, detail::operation::write));
  }

  [[nodiscard]] auto try_receive()
      -> hal::result<hal::can::polled_frame<hal::can::classic_frame>, error_type> {
    std::array<std::byte, CANFD_MTU> buffer{};
    const ssize_t count = ::recv(fd_.get(), buffer.data(), buffer.size(),
                                 MSG_DONTWAIT);
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return hal::result<hal::can::polled_frame<hal::can::classic_frame>,
                         error_type>::success(
          hal::can::polled_frame<hal::can::classic_frame>::unavailable());
    }
    if (count < 0) {
      return failure_classic(errno, detail::operation::read);
    }
    if (count != CAN_MTU && count != CANFD_MTU) {
      return failure_classic(EPROTO, detail::operation::read);
    }

    const auto* classic = reinterpret_cast<const ::can_frame*>(buffer.data());
    const canid_t can_id = classic->can_id;
    if ((can_id & CAN_ERR_FLAG) != 0U) {
      return failure_classic(EPROTO, detail::operation::read);
    }
    const std::uint8_t length =
        count == CAN_MTU
            ? classic->can_dlc
            : detail_can::dlc_to_length(
                  reinterpret_cast<const ::canfd_frame*>(buffer.data())->len);
    if (length > 8U) {
      return failure_classic(EPROTO, detail::operation::read);
    }
    const std::byte* payload = count == CAN_MTU
                                   ? reinterpret_cast<const std::byte*>(classic->data)
                                   : reinterpret_cast<const std::byte*>(
                                         reinterpret_cast<const ::canfd_frame*>(buffer.data())
                                             ->data);
    const auto made = hal::can::classic_frame::make(
        detail_can::decode_id(can_id), {payload, length});
    if (!made) {
      return failure_classic(EPROTO, detail::operation::read);
    }
    return hal::result<hal::can::polled_frame<hal::can::classic_frame>,
                       error_type>::success(
        hal::can::polled_frame<hal::can::classic_frame>::available(
            std::move(made).value()));
  }

  template <bool Enabled = EnableFd>
    requires Enabled
  [[nodiscard]] auto try_receive_fd()
      -> hal::result<hal::can::polled_frame<hal::can::fd_frame>, error_type> {
    std::array<std::byte, CANFD_MTU> buffer{};
    const ssize_t count = ::recv(fd_.get(), buffer.data(), buffer.size(),
                                 MSG_DONTWAIT);
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return hal::result<hal::can::polled_frame<hal::can::fd_frame>,
                         error_type>::success(
          hal::can::polled_frame<hal::can::fd_frame>::unavailable());
    }
    if (count < 0) {
      return failure_fd(errno, detail::operation::read);
    }
    if (count != CAN_MTU && count != CANFD_MTU) {
      return failure_fd(EPROTO, detail::operation::read);
    }
    const auto* classic = reinterpret_cast<const ::can_frame*>(buffer.data());
    const auto* flexible = reinterpret_cast<const ::canfd_frame*>(buffer.data());
    if ((classic->can_id & CAN_ERR_FLAG) != 0U) {
      return failure_fd(EPROTO, detail::operation::read);
    }
    const std::uint8_t length =
        count == CAN_MTU ? classic->can_dlc : detail_can::dlc_to_length(flexible->len);
    const std::byte* payload = count == CAN_MTU
                                   ? reinterpret_cast<const std::byte*>(classic->data)
                                   : reinterpret_cast<const std::byte*>(flexible->data);
    const auto made = hal::can::fd_frame::make(
        detail_can::decode_id(classic->can_id), {payload, length},
        count == CANFD_MTU && (flexible->flags & CANFD_BRS) != 0U,
        count == CANFD_MTU && (flexible->flags & CANFD_ESI) != 0U);
    if (!made) {
      return failure_fd(EPROTO, detail::operation::read);
    }
    return hal::result<hal::can::polled_frame<hal::can::fd_frame>,
                       error_type>::success(
        hal::can::polled_frame<hal::can::fd_frame>::available(
            std::move(made).value()));
  }

  [[nodiscard]] auto set_filters(hal::span<const hal::can::filter> filters)
      -> hal::result<void, error_type> {
    if (!filters.valid() || filters.size() > MaxFilters) {
      return failure_void(EINVAL, detail::operation::configure);
    }
    std::array<::can_filter, MaxFilters> native{};
    for (std::size_t index = 0U; index < filters.size(); ++index) {
      if (!filters[index].valid()) {
        return failure_void(EINVAL, detail::operation::configure);
      }
      native[index].can_id = static_cast<canid_t>(filters[index].frame_id.value) |
                             (filters[index].frame_id.extended ? CAN_EFF_FLAG : 0U);
      native[index].can_mask = static_cast<canid_t>(filters[index].mask) |
                               CAN_EFF_FLAG;
    }
    const void* data = filters.empty() ? nullptr : native.data();
    if (::setsockopt(fd_.get(), SOL_CAN_RAW, CAN_RAW_FILTER, data,
                     static_cast<socklen_t>(filters.size() * sizeof(::can_filter))) <
        0) {
      return failure_void(errno, detail::operation::configure);
    }
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] int native_fd() const noexcept { return fd_.get(); }

 private:
  explicit Controller(detail::owned_fd fd) noexcept : fd_{std::move(fd)} {}

  [[nodiscard]] static auto failure(int native_error, detail::operation operation)
      -> hal::result<Controller, error_type> {
    return hal::result<Controller, error_type>::failure(
        detail_can::failure(native_error, operation));
  }

  [[nodiscard]] static auto failure_void(int native_error,
                                         detail::operation operation)
      -> hal::result<void, error_type> {
    return hal::result<void, error_type>::failure(
        detail_can::failure(native_error, operation));
  }

  [[nodiscard]] static auto failure_classic(int native_error,
                                            detail::operation operation)
      -> hal::result<hal::can::polled_frame<hal::can::classic_frame>, error_type> {
    return hal::result<hal::can::polled_frame<hal::can::classic_frame>,
                       error_type>::failure(detail_can::failure(native_error, operation));
  }

  [[nodiscard]] static auto failure_fd(int native_error, detail::operation operation)
      -> hal::result<hal::can::polled_frame<hal::can::fd_frame>, error_type> {
    return hal::result<hal::can::polled_frame<hal::can::fd_frame>, error_type>::failure(
        detail_can::failure(native_error, operation));
  }

  detail::owned_fd fd_{};
};

static_assert(hal::can::ClassicController<Controller<false>>);
static_assert(hal::can::FilterableController<Controller<false>>);
static_assert(hal::can::FdController<Controller<true>>);

}  // namespace hal::linux::can
