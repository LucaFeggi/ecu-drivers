#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/time.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include <cerrno>
#include <cstdint>
#include <hal/foundation/assert.hpp>
#include <hal/foundation/result.hpp>
#include <hal/foundation/units.hpp>
#include <limits>
#include <sys/timerfd.h>
#include <time.h>
#include <utility>

#include <hal/detail/error.hpp>
#include <hal/detail/fd.hpp>

namespace hal::linux::time {

enum class error_kind : std::uint8_t { configuration, io, other };
using error_type = detail::error<error_kind>;

namespace detail_time {

[[nodiscard]] inline hal::instant monotonic_now() noexcept {
  ::timespec value{};
  const int status = ::clock_gettime(CLOCK_MONOTONIC, &value);
  HAL_CORE_ASSERT(status == 0);
  if (status != 0 || value.tv_sec < 0) {
    return {};
  }

  const auto seconds = static_cast<std::uint64_t>(value.tv_sec);
  constexpr std::uint64_t nanoseconds_per_second{1'000'000'000U};
  if (seconds > std::numeric_limits<std::uint64_t>::max() /
                   nanoseconds_per_second) {
    return {std::numeric_limits<std::uint64_t>::max()};
  }
  return {seconds * nanoseconds_per_second +
          static_cast<std::uint64_t>(value.tv_nsec)};
}

[[nodiscard]] inline ::timespec to_timespec(hal::instant value) noexcept {
  constexpr std::uint64_t nanoseconds_per_second{1'000'000'000U};
  return {static_cast<time_t>(value.nanoseconds_since_boot /
                              nanoseconds_per_second),
          static_cast<long>(value.nanoseconds_since_boot %
                            nanoseconds_per_second)};
}

}  // namespace detail_time

class MonotonicClock {
 public:
  [[nodiscard]] hal::instant now() const noexcept {
    return detail_time::monotonic_now();
  }
};

class Delay {
 public:
  void delay_for(hal::nanoseconds duration) const noexcept {
    const hal::instant start = detail_time::monotonic_now();
    const std::uint64_t maximum =
        std::numeric_limits<std::uint64_t>::max();
    const std::uint64_t deadline =
        duration.value > maximum - start.nanoseconds_since_boot
            ? maximum
            : start.nanoseconds_since_boot + duration.value;
    const ::timespec requested =
        detail_time::to_timespec(hal::instant{deadline});

    for (;;) {
      const int status =
          ::clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &requested, nullptr);
      if (status == 0) {
        return;
      }
      if (status != EINTR) {
        HAL_CORE_ASSERT(false);
        return;
      }
    }
  }
};

class OneShotAlarm {
 public:
  using error_type = linux::time::error_type;

  [[nodiscard]] static auto open()
      -> hal::result<OneShotAlarm, error_type> {
    const int fd = ::timerfd_create(CLOCK_MONOTONIC,
                                    TFD_CLOEXEC | TFD_NONBLOCK);
    if (fd < 0) {
      return hal::result<OneShotAlarm, error_type>::failure(
          error_type{error_kind::io, errno, detail::operation::open});
    }
    return hal::result<OneShotAlarm, error_type>::success(
        OneShotAlarm{detail::owned_fd{fd}});
  }

  OneShotAlarm(const OneShotAlarm&) = delete;
  OneShotAlarm& operator=(const OneShotAlarm&) = delete;
  OneShotAlarm(OneShotAlarm&&) noexcept = default;
  OneShotAlarm& operator=(OneShotAlarm&&) noexcept = default;

  [[nodiscard]] hal::instant now() const noexcept {
    return detail_time::monotonic_now();
  }

  [[nodiscard]] auto arm(hal::instant deadline)
      -> hal::result<void, error_type> {
    if (!fd_.valid()) {
      return failure(error_kind::configuration, EBADF,
                     detail::operation::configure);
    }
    ::itimerspec value{};
    value.it_value = detail_time::to_timespec(deadline);
    if (::timerfd_settime(fd_.get(), TFD_TIMER_ABSTIME, &value, nullptr) < 0) {
      return failure(error_kind::io, errno, detail::operation::ioctl);
    }
    fault_ = false;
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] auto cancel() -> hal::result<void, error_type> {
    if (!fd_.valid()) {
      return failure(error_kind::configuration, EBADF,
                     detail::operation::configure);
    }
    ::itimerspec value{};
    if (::timerfd_settime(fd_.get(), 0, &value, nullptr) < 0) {
      return failure(error_kind::io, errno, detail::operation::ioctl);
    }

    std::uint64_t expirations{};
    while (::read(fd_.get(), &expirations, sizeof(expirations)) > 0) {
    }
    fault_ = false;
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] bool expired() noexcept {
    if (!fd_.valid()) {
      fault_ = true;
      return false;
    }
    std::uint64_t expirations{};
    const ssize_t count = ::read(fd_.get(), &expirations, sizeof(expirations));
    if (count == static_cast<ssize_t>(sizeof(expirations))) {
      return expirations != 0U;
    }
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return false;
    }
    if (count < 0 && errno == EINTR) {
      return false;
    }
    fault_ = true;
    return false;
  }

  [[nodiscard]] bool has_fault() const noexcept { return fault_; }
  [[nodiscard]] int native_fd() const noexcept { return fd_.get(); }

 private:
  explicit OneShotAlarm(detail::owned_fd fd) noexcept : fd_{std::move(fd)} {}

  [[nodiscard]] static auto failure(error_kind kind, int native_error,
                                    detail::operation operation)
      -> hal::result<void, error_type> {
    return hal::result<void, error_type>::failure(
        error_type{kind, native_error, operation});
  }

  detail::owned_fd fd_{};
  bool fault_{};
};

static_assert(hal::time::MonotonicClock<MonotonicClock>);
static_assert(hal::time::Delay<Delay>);
static_assert(hal::time::OneShotAlarm<OneShotAlarm>);

}  // namespace hal::linux::time
