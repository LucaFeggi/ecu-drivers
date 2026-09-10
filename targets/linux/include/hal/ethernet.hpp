#pragma once

#include <array>
#include <arpa/inet.h>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <hal/ethernet.hpp>
#include <hal/foundation/result.hpp>
#include <hal/foundation/span.hpp>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <linux/if_tun.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <utility>

#include <hal/linux/detail/error.hpp>
#include <hal/linux/detail/fd.hpp>

namespace hal::linux::ethernet {

using error_type = detail::error<hal::ethernet::error_kind>;

inline constexpr std::size_t default_frame_capacity{1600U};

namespace detail_ethernet {

[[nodiscard]] inline error_type failure(int native_error,
                                        detail::operation operation) noexcept {
  hal::ethernet::error_kind kind = hal::ethernet::error_kind::io;
  if (native_error == EAGAIN) {
    kind = hal::ethernet::error_kind::would_block;
  } else if (native_error == ENETDOWN || native_error == ENOLINK) {
    kind = hal::ethernet::error_kind::link_down;
  } else if (native_error == EMSGSIZE) {
    kind = hal::ethernet::error_kind::frame_too_large;
  }
  return error_type{kind, native_error, operation};
}

[[nodiscard]] inline bool valid_name(const char* name) noexcept {
  return name != nullptr && ::strnlen(name, IFNAMSIZ) < IFNAMSIZ;
}

[[nodiscard]] inline bool copy_name(char (&destination)[IFNAMSIZ],
                                    const char* source) noexcept {
  if (!valid_name(source)) {
    return false;
  }
  std::strncpy(destination, source, IFNAMSIZ - 1U);
  destination[IFNAMSIZ - 1U] = '\0';
  return true;
}

}  // namespace detail_ethernet

template <std::size_t FrameCapacity = default_frame_capacity>
class TapFramePort {
  static_assert(FrameCapacity >= ETH_ZLEN);

 public:
  using error_type = linux::ethernet::error_type;

  struct config {
    const char* interface_name{};
    hal::ethernet::mac_address address{};
    bool link_up{true};
  };

  [[nodiscard]] static auto open(config configuration)
      -> hal::result<TapFramePort, error_type> {
    const int fd = ::open("/dev/net/tun", O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
      return failure(errno, detail::operation::open);
    }
    ::ifreq request{};
    if (configuration.interface_name != nullptr &&
        !detail_ethernet::copy_name(request.ifr_name, configuration.interface_name)) {
      (void)::close(fd);
      return failure(EINVAL, detail::operation::configure);
    }
    request.ifr_flags = static_cast<short>(IFF_TAP | IFF_NO_PI);
    if (::ioctl(fd, TUNSETIFF, &request) < 0) {
      const int native_error = errno;
      (void)::close(fd);
      return failure(native_error, detail::operation::ioctl);
    }
    return hal::result<TapFramePort, error_type>::success(TapFramePort{
        detail::owned_fd{fd}, configuration.address,
        {configuration.link_up, hal::ethernet::speed::unknown,
         hal::ethernet::duplex::full}});
  }

  TapFramePort(const TapFramePort&) = delete;
  TapFramePort& operator=(const TapFramePort&) = delete;
  TapFramePort(TapFramePort&&) noexcept = default;
  TapFramePort& operator=(TapFramePort&&) noexcept = default;

  [[nodiscard]] hal::ethernet::mac_address address() const noexcept {
    return address_;
  }

  [[nodiscard]] auto link()
      -> hal::result<hal::ethernet::link_state, error_type> {
    return hal::result<hal::ethernet::link_state, error_type>::success(link_);
  }

  [[nodiscard]] auto try_transmit(hal::span<const std::byte> frame)
      -> hal::result<bool, error_type> {
    if (!frame.valid() || frame.size() > FrameCapacity) {
      return failure_bool(EMSGSIZE, detail::operation::write);
    }
    if (!link_.up) {
      return failure_bool(ENETDOWN, detail::operation::write);
    }
    const ssize_t count = ::write(fd_.get(), frame.data(), frame.size());
    if (count == static_cast<ssize_t>(frame.size())) {
      return hal::result<bool, error_type>::success(true);
    }
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return hal::result<bool, error_type>::success(false);
    }
    return failure_bool(count < 0 ? errno : EIO, detail::operation::write);
  }

  [[nodiscard]] auto try_receive(hal::span<std::byte> output)
      -> hal::result<hal::ethernet::received_frame, error_type> {
    if (!output.valid()) {
      return failure_frame(EINVAL, detail::operation::read);
    }
    std::array<std::byte, FrameCapacity> staging{};
    std::byte* destination = output.size() >= FrameCapacity ? output.data()
                                                             : staging.data();
    const ssize_t count = ::read(fd_.get(), destination, FrameCapacity);
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return hal::result<hal::ethernet::received_frame, error_type>::success({});
    }
    if (count < 0) {
      return failure_frame(errno, detail::operation::read);
    }
    const std::size_t size = static_cast<std::size_t>(count);
    if (size > output.size()) {
      return failure_frame(EMSGSIZE, detail::operation::read);
    }
    if (destination == staging.data() && size != 0U) {
      std::memcpy(output.data(), staging.data(), size);
    }
    return hal::result<hal::ethernet::received_frame, error_type>::success(
        {true, size});
  }

  [[nodiscard]] int native_fd() const noexcept { return fd_.get(); }

 private:
  TapFramePort(detail::owned_fd fd, hal::ethernet::mac_address address,
               hal::ethernet::link_state link) noexcept
      : fd_{std::move(fd)}, address_{address}, link_{link} {}

  [[nodiscard]] static auto failure(int native_error, detail::operation operation)
      -> hal::result<TapFramePort, error_type> {
    return hal::result<TapFramePort, error_type>::failure(
        detail_ethernet::failure(native_error, operation));
  }
  [[nodiscard]] static auto failure_bool(int native_error,
                                         detail::operation operation)
      -> hal::result<bool, error_type> {
    return hal::result<bool, error_type>::failure(
        detail_ethernet::failure(native_error, operation));
  }
  [[nodiscard]] static auto failure_frame(int native_error,
                                          detail::operation operation)
      -> hal::result<hal::ethernet::received_frame, error_type> {
    return hal::result<hal::ethernet::received_frame, error_type>::failure(
        detail_ethernet::failure(native_error, operation));
  }

  detail::owned_fd fd_{};
  hal::ethernet::mac_address address_{};
  hal::ethernet::link_state link_{};
};

template <std::size_t FrameCapacity = default_frame_capacity>
class PacketFramePort {
  static_assert(FrameCapacity >= ETH_ZLEN);

 public:
  using error_type = linux::ethernet::error_type;

  [[nodiscard]] static auto open(const char* interface_name)
      -> hal::result<PacketFramePort, error_type> {
    if (!detail_ethernet::valid_name(interface_name)) {
      return failure(EINVAL, detail::operation::open);
    }
    const int fd = ::socket(AF_PACKET, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC,
                            htons(ETH_P_ALL));
    if (fd < 0) {
      return failure(errno, detail::operation::open);
    }
    const unsigned int index = ::if_nametoindex(interface_name);
    if (index == 0U) {
      const int native_error = errno == 0 ? ENODEV : errno;
      (void)::close(fd);
      return failure(native_error, detail::operation::configure);
    }

    ::sockaddr_ll address{};
    address.sll_family = AF_PACKET;
    address.sll_protocol = htons(ETH_P_ALL);
    address.sll_ifindex = static_cast<int>(index);
    if (::bind(fd, reinterpret_cast<const ::sockaddr*>(&address),
               sizeof(address)) < 0) {
      const int native_error = errno;
      (void)::close(fd);
      return failure(native_error, detail::operation::configure);
    }

    ::ifreq hardware{};
    if (!detail_ethernet::copy_name(hardware.ifr_name, interface_name) ||
        ::ioctl(fd, SIOCGIFHWADDR, &hardware) < 0) {
      const int native_error = errno == 0 ? EIO : errno;
      (void)::close(fd);
      return failure(native_error, detail::operation::ioctl);
    }
    hal::ethernet::mac_address mac{};
    std::memcpy(mac.bytes.data(), hardware.ifr_hwaddr.sa_data,
                mac.bytes.size());
    return hal::result<PacketFramePort, error_type>::success(
        PacketFramePort{detail::owned_fd{fd}, mac, interface_name});
  }

  PacketFramePort(const PacketFramePort&) = delete;
  PacketFramePort& operator=(const PacketFramePort&) = delete;
  PacketFramePort(PacketFramePort&&) noexcept = default;
  PacketFramePort& operator=(PacketFramePort&&) noexcept = default;

  [[nodiscard]] hal::ethernet::mac_address address() const noexcept {
    return address_;
  }

  [[nodiscard]] auto link()
      -> hal::result<hal::ethernet::link_state, error_type> {
    ::ifreq request{};
    std::memcpy(request.ifr_name, interface_name_, IFNAMSIZ);
    if (::ioctl(fd_.get(), SIOCGIFFLAGS, &request) < 0) {
      return hal::result<hal::ethernet::link_state, error_type>::failure(
          detail_ethernet::failure(errno, detail::operation::ioctl));
    }
    const short flags = request.ifr_flags;
    return hal::result<hal::ethernet::link_state, error_type>::success(
        {((flags & IFF_UP) != 0) && ((flags & IFF_RUNNING) != 0),
         hal::ethernet::speed::unknown, hal::ethernet::duplex::full});
  }

  [[nodiscard]] auto try_transmit(hal::span<const std::byte> frame)
      -> hal::result<bool, error_type> {
    if (!frame.valid() || frame.size() > FrameCapacity) {
      return failure_bool(EMSGSIZE, detail::operation::write);
    }
    const ssize_t count = ::send(fd_.get(), frame.data(), frame.size(),
                                 MSG_DONTWAIT | MSG_NOSIGNAL);
    if (count == static_cast<ssize_t>(frame.size())) {
      return hal::result<bool, error_type>::success(true);
    }
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return hal::result<bool, error_type>::success(false);
    }
    return failure_bool(count < 0 ? errno : EIO, detail::operation::write);
  }

  [[nodiscard]] auto try_receive(hal::span<std::byte> output)
      -> hal::result<hal::ethernet::received_frame, error_type> {
    if (!output.valid()) {
      return failure_frame(EINVAL, detail::operation::read);
    }
    std::array<std::byte, FrameCapacity> staging{};
    std::byte* destination = output.size() >= FrameCapacity ? output.data()
                                                             : staging.data();
    const ssize_t count = ::recv(fd_.get(), destination, FrameCapacity,
                                 MSG_DONTWAIT | MSG_TRUNC);
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return hal::result<hal::ethernet::received_frame, error_type>::success({});
    }
    if (count < 0) {
      return failure_frame(errno, detail::operation::read);
    }
    const std::size_t size = static_cast<std::size_t>(count);
    if (size > FrameCapacity || size > output.size()) {
      return failure_frame(EMSGSIZE, detail::operation::read);
    }
    if (destination == staging.data() && size != 0U) {
      std::memcpy(output.data(), staging.data(), size);
    }
    return hal::result<hal::ethernet::received_frame, error_type>::success(
        {true, size});
  }

  [[nodiscard]] int native_fd() const noexcept { return fd_.get(); }

 private:
  PacketFramePort(detail::owned_fd fd, hal::ethernet::mac_address address,
                  const char* interface_name) noexcept
      : fd_{std::move(fd)}, address_{address} {
    (void)detail_ethernet::copy_name(interface_name_, interface_name);
  }

  [[nodiscard]] static auto failure(int native_error, detail::operation operation)
      -> hal::result<PacketFramePort, error_type> {
    return hal::result<PacketFramePort, error_type>::failure(
        detail_ethernet::failure(native_error, operation));
  }
  [[nodiscard]] static auto failure_bool(int native_error,
                                         detail::operation operation)
      -> hal::result<bool, error_type> {
    return hal::result<bool, error_type>::failure(
        detail_ethernet::failure(native_error, operation));
  }
  [[nodiscard]] static auto failure_frame(int native_error,
                                          detail::operation operation)
      -> hal::result<hal::ethernet::received_frame, error_type> {
    return hal::result<hal::ethernet::received_frame, error_type>::failure(
        detail_ethernet::failure(native_error, operation));
  }

  detail::owned_fd fd_{};
  hal::ethernet::mac_address address_{};
  char interface_name_[IFNAMSIZ]{};
};

static_assert(hal::ethernet::FramePort<TapFramePort<>>);
static_assert(hal::ethernet::FramePort<PacketFramePort<>>);

}  // namespace hal::linux::ethernet
