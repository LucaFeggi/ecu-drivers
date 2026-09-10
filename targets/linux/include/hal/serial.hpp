#pragma once

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <fcntl.h>
#include <hal/foundation/result.hpp>
#include <hal/foundation/span.hpp>
#include <hal/serial.hpp>
#include <termios.h>
#include <unistd.h>
#include <utility>

#include <hal/linux/detail/error.hpp>
#include <hal/linux/detail/fd.hpp>

namespace hal::linux::serial {

using error_type = detail::error<hal::serial::error_kind>;

struct config {
  const char* device{};
};

namespace detail_serial {

[[nodiscard]] inline error_type failure(int native_error,
                                        detail::operation operation) noexcept {
  hal::serial::error_kind kind = hal::serial::error_kind::io;
  if (native_error == EINVAL || native_error == ENOTSUP) {
    kind = hal::serial::error_kind::configuration;
  } else if (native_error == EIO) {
    kind = hal::serial::error_kind::io;
  }
  return error_type{kind, native_error, operation};
}

[[nodiscard]] inline bool baud_rate(hal::hertz requested, speed_t& value) noexcept {
  switch (requested.value) {
    case 50U:
      value = B50;
      return true;
    case 75U:
      value = B75;
      return true;
    case 110U:
      value = B110;
      return true;
    case 134U:
      value = B134;
      return true;
    case 150U:
      value = B150;
      return true;
    case 200U:
      value = B200;
      return true;
    case 300U:
      value = B300;
      return true;
    case 600U:
      value = B600;
      return true;
    case 1200U:
      value = B1200;
      return true;
    case 1800U:
      value = B1800;
      return true;
    case 2400U:
      value = B2400;
      return true;
    case 4800U:
      value = B4800;
      return true;
    case 9600U:
      value = B9600;
      return true;
    case 19200U:
      value = B19200;
      return true;
    case 38400U:
      value = B38400;
      return true;
#ifdef B57600
    case 57600U:
      value = B57600;
      return true;
#endif
#ifdef B115200
    case 115200U:
      value = B115200;
      return true;
#endif
#ifdef B230400
    case 230400U:
      value = B230400;
      return true;
#endif
#ifdef B460800
    case 460800U:
      value = B460800;
      return true;
#endif
#ifdef B500000
    case 500000U:
      value = B500000;
      return true;
#endif
#ifdef B576000
    case 576000U:
      value = B576000;
      return true;
#endif
#ifdef B921600
    case 921600U:
      value = B921600;
      return true;
#endif
#ifdef B1000000
    case 1000000U:
      value = B1000000;
      return true;
#endif
#ifdef B1152000
    case 1152000U:
      value = B1152000;
      return true;
#endif
#ifdef B1500000
    case 1500000U:
      value = B1500000;
      return true;
#endif
#ifdef B2000000
    case 2000000U:
      value = B2000000;
      return true;
#endif
#ifdef B2500000
    case 2500000U:
      value = B2500000;
      return true;
#endif
#ifdef B3000000
    case 3000000U:
      value = B3000000;
      return true;
#endif
#ifdef B3500000
    case 3500000U:
      value = B3500000;
      return true;
#endif
#ifdef B4000000
    case 4000000U:
      value = B4000000;
      return true;
#endif
    default:
      return false;
  }
}

}  // namespace detail_serial

class Port {
 public:
  using error_type = linux::serial::error_type;

  [[nodiscard]] static auto open(config configuration)
      -> hal::result<Port, error_type> {
    if (configuration.device == nullptr) {
      return failure(EINVAL, detail::operation::open);
    }
    const int fd = ::open(configuration.device,
                          O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
      return failure(errno, detail::operation::open);
    }
    return hal::result<Port, error_type>::success(Port{detail::owned_fd{fd}});
  }

  Port(const Port&) = delete;
  Port& operator=(const Port&) = delete;
  Port(Port&&) noexcept = default;
  Port& operator=(Port&&) noexcept = default;

  [[nodiscard]] auto configure(hal::serial::config configuration)
      -> hal::result<hal::hertz, error_type> {
    speed_t speed{};
    if (!detail_serial::baud_rate(configuration.baud_rate, speed) ||
        configuration.bits == hal::serial::data_bits::bits9 ||
        (configuration.flow != hal::serial::flow_control::none &&
         configuration.flow != hal::serial::flow_control::rts_cts)) {
      return failure_hertz(EINVAL, detail::operation::configure);
    }

    ::termios settings{};
    if (::tcgetattr(fd_.get(), &settings) < 0) {
      return failure_hertz(errno, detail::operation::configure);
    }

    settings.c_iflag = 0U;
    settings.c_oflag = 0U;
    settings.c_lflag = 0U;
    settings.c_cflag = CLOCAL | CREAD;
    settings.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    settings.c_cflag |= configuration.bits == hal::serial::data_bits::bits7
                            ? CS7
                            : CS8;
    if (configuration.parity_mode != hal::serial::parity::none) {
      settings.c_cflag |= PARENB;
      if (configuration.parity_mode == hal::serial::parity::odd) {
        settings.c_cflag |= PARODD;
      }
    }
    if (configuration.stops == hal::serial::stop_bits::two) {
      settings.c_cflag |= CSTOPB;
    }
#ifdef CRTSCTS
    if (configuration.flow == hal::serial::flow_control::rts_cts) {
      settings.c_cflag |= CRTSCTS;
    }
#else
    if (configuration.flow == hal::serial::flow_control::rts_cts) {
      return failure_hertz(ENOTSUP, detail::operation::configure);
    }
#endif
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    if (::cfsetispeed(&settings, speed) < 0 ||
        ::cfsetospeed(&settings, speed) < 0 ||
        ::tcsetattr(fd_.get(), TCSANOW, &settings) < 0) {
      return failure_hertz(errno, detail::operation::configure);
    }
    configured_ = true;
    return hal::result<hal::hertz, error_type>::success(configuration.baud_rate);
  }

  [[nodiscard]] auto try_write(hal::span<const std::byte> data)
      -> hal::result<std::size_t, error_type> {
    if (!configured_ || !data.valid()) {
      return failure_size(EINVAL, detail::operation::write);
    }
    if (data.empty()) {
      return hal::result<std::size_t, error_type>::success(0U);
    }
    const ssize_t count = ::write(fd_.get(), data.data(), data.size());
    if (count >= 0) {
      return hal::result<std::size_t, error_type>::success(
          static_cast<std::size_t>(count));
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      return hal::result<std::size_t, error_type>::success(0U);
    }
    return failure_size(errno, detail::operation::write);
  }

  [[nodiscard]] auto try_read(hal::span<std::byte> data)
      -> hal::result<std::size_t, error_type> {
    if (!configured_ || !data.valid()) {
      return failure_size(EINVAL, detail::operation::read);
    }
    if (data.empty()) {
      return hal::result<std::size_t, error_type>::success(0U);
    }
    const ssize_t count = ::read(fd_.get(), data.data(), data.size());
    if (count >= 0) {
      return hal::result<std::size_t, error_type>::success(
          static_cast<std::size_t>(count));
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      return hal::result<std::size_t, error_type>::success(0U);
    }
    return failure_size(errno, detail::operation::read);
  }

  [[nodiscard]] auto flush() -> hal::result<void, error_type> {
    if (!configured_) {
      return failure_void(EINVAL, detail::operation::configure);
    }
    if (::tcdrain(fd_.get()) < 0) {
      return failure_void(errno, detail::operation::synchronize);
    }
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] int native_fd() const noexcept { return fd_.get(); }

 private:
  explicit Port(detail::owned_fd fd) noexcept : fd_{std::move(fd)} {}

  [[nodiscard]] static auto failure(int native_error, detail::operation operation)
      -> hal::result<Port, error_type> {
    return hal::result<Port, error_type>::failure(
        detail_serial::failure(native_error, operation));
  }

  [[nodiscard]] static auto failure_hertz(int native_error,
                                          detail::operation operation)
      -> hal::result<hal::hertz, error_type> {
    return hal::result<hal::hertz, error_type>::failure(
        detail_serial::failure(native_error, operation));
  }

  [[nodiscard]] static auto failure_size(int native_error,
                                         detail::operation operation)
      -> hal::result<std::size_t, error_type> {
    return hal::result<std::size_t, error_type>::failure(
        detail_serial::failure(native_error, operation));
  }

  [[nodiscard]] static auto failure_void(int native_error,
                                         detail::operation operation)
      -> hal::result<void, error_type> {
    return hal::result<void, error_type>::failure(
        detail_serial::failure(native_error, operation));
  }

  detail::owned_fd fd_{};
  bool configured_{};
};

static_assert(hal::serial::Port<Port>);
static_assert(hal::serial::ConfigurablePort<Port>);

}  // namespace hal::linux::serial
