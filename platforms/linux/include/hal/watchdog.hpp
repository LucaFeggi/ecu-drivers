#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/watchdog.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include <cerrno>
#include <cstdint>
#include <fcntl.h>
#include <hal/foundation/result.hpp>
#include <linux/watchdog.h>
#include <limits>
#include <sys/ioctl.h>
#include <unistd.h>
#include <utility>

#include <hal/detail/error.hpp>
#include <hal/detail/fd.hpp>

namespace hal::linux::watchdog {

using error_type = detail::error<hal::watchdog::error_kind>;

struct config {
  const char* device{};
  std::uint32_t timeout_seconds{};
  bool magic_close{};
};

namespace detail_watchdog {

[[nodiscard]] inline error_type failure(int native_error,
                                        detail::operation operation) noexcept {
  const auto kind = native_error == EINVAL
                        ? hal::watchdog::error_kind::hardware_fault
                        : hal::watchdog::error_kind::io;
  return error_type{kind, native_error, operation};
}

}  // namespace detail_watchdog

template <std::uint64_t MaximumFeedIntervalNanoseconds,
          bool OscillatorIndependent = false>
class Feeder {
  static_assert(MaximumFeedIntervalNanoseconds > 0U);
  static_assert(MaximumFeedIntervalNanoseconds /
                        1'000'000'000U <=
                    std::numeric_limits<std::uint32_t>::max());

 public:
  using error_type = linux::watchdog::error_type;

  [[nodiscard]] static constexpr hal::watchdog::characteristics
  properties() noexcept {
    return {hal::nanoseconds{0U},
            hal::nanoseconds{MaximumFeedIntervalNanoseconds},
            OscillatorIndependent};
  }

  [[nodiscard]] static auto open(config configuration)
      -> hal::result<Feeder, error_type> {
    if (configuration.device == nullptr || configuration.timeout_seconds == 0U ||
        configuration.timeout_seconds >
            static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
      return failure(EINVAL, detail::operation::open);
    }
    const int fd = ::open(configuration.device, O_WRONLY | O_CLOEXEC);
    if (fd < 0) {
      return failure(errno, detail::operation::open);
    }
    int timeout = static_cast<int>(configuration.timeout_seconds);
    if (::ioctl(fd, WDIOC_SETTIMEOUT, &timeout) < 0) {
      const int native_error = errno;
      (void)::close(fd);
      return failure(native_error, detail::operation::configure);
    }
    if (timeout <= 0 || static_cast<std::uint64_t>(timeout) *
                            std::uint64_t{1'000'000'000U} >
                        MaximumFeedIntervalNanoseconds) {
      (void)::close(fd);
      return failure(EINVAL, detail::operation::configure);
    }
    return hal::result<Feeder, error_type>::success(
        Feeder{detail::owned_fd{fd}, configuration.magic_close});
  }

  Feeder(const Feeder&) = delete;
  Feeder& operator=(const Feeder&) = delete;
  Feeder(Feeder&& other) noexcept
      : fd_{std::move(other.fd_)}, magic_close_{other.magic_close_} {
    other.magic_close_ = false;
  }
  Feeder& operator=(Feeder&&) = delete;

  ~Feeder() {
    if (magic_close_ && fd_.valid()) {
      const char value = 'V';
      (void)::write(fd_.get(), &value, 1U);
    }
  }

  [[nodiscard]] auto feed() -> hal::result<void, error_type> {
    if (::ioctl(fd_.get(), WDIOC_KEEPALIVE, nullptr) < 0) {
      return hal::result<void, error_type>::failure(
          detail_watchdog::failure(errno, detail::operation::ioctl));
    }
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] int native_fd() const noexcept { return fd_.get(); }

 private:
  Feeder(detail::owned_fd fd, bool magic_close) noexcept
      : fd_{std::move(fd)}, magic_close_{magic_close} {}

  [[nodiscard]] static auto failure(int native_error, detail::operation operation)
      -> hal::result<Feeder, error_type> {
    return hal::result<Feeder, error_type>::failure(
        detail_watchdog::failure(native_error, operation));
  }

  detail::owned_fd fd_{};
  bool magic_close_{};
};

static_assert(hal::watchdog::Feeder<Feeder<1'000'000'000U>>);

}  // namespace hal::linux::watchdog
