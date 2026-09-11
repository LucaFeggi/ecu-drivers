#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/pwm.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include <cerrno>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <hal/foundation/result.hpp>
#include <limits>
#include <sys/stat.h>
#include <unistd.h>
#include <utility>

#include <hal/detail/error.hpp>
#include <hal/detail/fd.hpp>
#include <hal/detail/io.hpp>

namespace hal::linux::pwm {

using error_type = detail::error<hal::pwm::error_kind>;

struct config {
  const char* chip{};
  std::uint32_t channel{};
  bool export_channel{true};
};

namespace detail_pwm {

[[nodiscard]] inline error_type failure(int native_error,
                                        detail::operation operation) noexcept {
  const auto kind = native_error == EINVAL || native_error == ENOTSUP
                        ? hal::pwm::error_kind::unrepresentable
                        : hal::pwm::error_kind::io;
  return error_type{kind, native_error, operation};
}

[[nodiscard]] inline bool write_number(int fd, std::uint64_t value) noexcept {
  char buffer[32]{};
  const auto converted = std::to_chars(buffer, buffer + sizeof(buffer), value);
  if (converted.ec != std::errc{}) {
    return false;
  }
  if (::lseek(fd, 0, SEEK_SET) < 0) {
    return false;
  }
  return detail::write_all(fd, reinterpret_cast<const std::byte*>(buffer),
                           static_cast<std::size_t>(converted.ptr - buffer));
}

[[nodiscard]] inline bool write_text(int fd, const char* text) noexcept {
  if (text == nullptr || ::lseek(fd, 0, SEEK_SET) < 0) {
    return false;
  }
  return detail::write_all(fd, reinterpret_cast<const std::byte*>(text),
                           std::strlen(text));
}

inline void unexport(int chip_fd, std::uint32_t channel) noexcept {
  const int fd = ::openat(chip_fd, "unexport", O_WRONLY | O_CLOEXEC);
  if (fd >= 0) {
    (void)write_number(fd, channel);
    (void)::close(fd);
  }
}

[[nodiscard]] inline bool read_number(int fd, std::uint64_t& value) noexcept {
  char buffer[32]{};
  if (::lseek(fd, 0, SEEK_SET) < 0) {
    return false;
  }
  const ssize_t count = ::read(fd, buffer, sizeof(buffer) - 1U);
  if (count <= 0) {
    return false;
  }
  const auto parsed = std::from_chars(buffer, buffer + count, value);
  return parsed.ec == std::errc{};
}

[[nodiscard]] inline bool within_error(std::uint64_t requested,
                                       std::uint64_t actual,
                                       std::uint32_t maximum_ppm) noexcept {
  if (requested == 0U) {
    return false;
  }
  const std::uint64_t difference = requested > actual ? requested - actual
                                                       : actual - requested;
  if (maximum_ppm >= 1'000'000U) {
    return true;
  }
  const std::uint64_t maximum = std::numeric_limits<std::uint64_t>::max();
  const std::uint64_t allowed =
      maximum_ppm != 0U && requested > maximum / maximum_ppm
          ? maximum
          : (requested * maximum_ppm) / 1'000'000U;
  return difference <= allowed;
}

}  // namespace detail_pwm

class Output {
 public:
  using error_type = linux::pwm::error_type;

  [[nodiscard]] static auto open(config configuration)
      -> hal::result<Output, error_type> {
    if (configuration.chip == nullptr) {
      return failure(EINVAL, detail::operation::open);
    }
    const int chip_fd = ::open(configuration.chip,
                               O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    if (chip_fd < 0) {
      return failure(errno, detail::operation::open);
    }

    const int npwm_fd = ::openat(chip_fd, "npwm", O_RDONLY | O_CLOEXEC);
    std::uint64_t channel_count{};
    if (npwm_fd < 0 || !detail_pwm::read_number(npwm_fd, channel_count) ||
        channel_count <= configuration.channel) {
      const int native_error = errno == 0 ? EINVAL : errno;
      if (npwm_fd >= 0) {
        (void)::close(npwm_fd);
      }
      (void)::close(chip_fd);
      return failure(native_error, detail::operation::configure);
    }
    (void)::close(npwm_fd);

    bool owns_export = false;
    if (configuration.export_channel) {
      const int export_fd = ::openat(chip_fd, "export", O_WRONLY | O_CLOEXEC);
      if (export_fd < 0) {
        const int native_error = errno;
        (void)::close(chip_fd);
        return failure(native_error, detail::operation::configure);
      }
      owns_export = detail_pwm::write_number(export_fd, configuration.channel);
      const int export_error = errno;
      (void)::close(export_fd);
      if (!owns_export && export_error != EBUSY) {
        (void)::close(chip_fd);
        return failure(export_error == 0 ? EIO : export_error,
                       detail::operation::configure);
      }
    }

    char channel_name[32]{};
    channel_name[0] = 'p';
    channel_name[1] = 'w';
    channel_name[2] = 'm';
    const auto converted = std::to_chars(
        channel_name + 3, channel_name + sizeof(channel_name) - 1U,
        configuration.channel);
    if (converted.ec != std::errc{}) {
      if (owns_export) {
        detail_pwm::unexport(chip_fd, configuration.channel);
      }
      (void)::close(chip_fd);
      return failure(EINVAL, detail::operation::configure);
    }
    const int pwm_dir = ::openat(chip_fd, channel_name,
                                 O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    if (pwm_dir < 0) {
      const int native_error = errno;
      if (owns_export) {
        detail_pwm::unexport(chip_fd, configuration.channel);
      }
      (void)::close(chip_fd);
      return failure(native_error, detail::operation::configure);
    }

    const int period = ::openat(pwm_dir, "period", O_RDWR | O_CLOEXEC);
    const int duty = ::openat(pwm_dir, "duty_cycle", O_RDWR | O_CLOEXEC);
    const int polarity = ::openat(pwm_dir, "polarity", O_RDWR | O_CLOEXEC);
    const int enable = ::openat(pwm_dir, "enable", O_RDWR | O_CLOEXEC);
    if (period < 0 || duty < 0 || polarity < 0 || enable < 0) {
      const int native_error = errno;
      if (period >= 0) (void)::close(period);
      if (duty >= 0) (void)::close(duty);
      if (polarity >= 0) (void)::close(polarity);
      if (enable >= 0) (void)::close(enable);
      (void)::close(pwm_dir);
      if (owns_export) {
        detail_pwm::unexport(chip_fd, configuration.channel);
      }
      (void)::close(chip_fd);
      return failure(native_error, detail::operation::open);
    }
    if (!detail_pwm::write_number(enable, 0U)) {
      const int native_error = errno == 0 ? EIO : errno;
      (void)::close(period);
      (void)::close(duty);
      (void)::close(polarity);
      (void)::close(enable);
      (void)::close(pwm_dir);
      if (owns_export) {
        detail_pwm::unexport(chip_fd, configuration.channel);
      }
      (void)::close(chip_fd);
      return failure(native_error, detail::operation::configure);
    }

    return hal::result<Output, error_type>::success(Output{
        detail::owned_fd{chip_fd}, detail::owned_fd{pwm_dir},
        detail::owned_fd{period}, detail::owned_fd{duty},
        detail::owned_fd{polarity}, detail::owned_fd{enable},
        configuration.channel, owns_export});
  }

  Output(const Output&) = delete;
  Output& operator=(const Output&) = delete;
  Output(Output&& other) noexcept
      : chip_{std::move(other.chip_)}, pwm_dir_{std::move(other.pwm_dir_)},
        period_{std::move(other.period_)}, duty_{std::move(other.duty_)},
        polarity_{std::move(other.polarity_)}, enable_{std::move(other.enable_)},
        channel_{other.channel_}, owns_export_{other.owns_export_},
        configured_{other.configured_}, actual_period_{other.actual_period_} {
    other.owns_export_ = false;
    other.configured_ = false;
  }
  Output& operator=(Output&&) = delete;

  ~Output() {
    (void)detail_pwm::write_number(enable_.get(), 0U);
    if (owns_export_ && chip_.valid()) {
      const int unexport = ::openat(chip_.get(), "unexport", O_WRONLY | O_CLOEXEC);
      if (unexport >= 0) {
        (void)detail_pwm::write_number(unexport, channel_);
        (void)::close(unexport);
      }
    }
  }

  [[nodiscard]] auto configure(hal::pwm::config configuration)
      -> hal::result<hal::nanoseconds, error_type> {
    configured_ = false;
    if (configuration.requested_period.value == 0U) {
      return failure_nanoseconds(EINVAL, detail::operation::configure);
    }
    if (!detail_pwm::write_number(enable_.get(), 0U)) {
      return failure_nanoseconds(errno == 0 ? EIO : errno,
                                 detail::operation::configure);
    }
    const char* polarity = configuration.output_polarity ==
                                   hal::pwm::polarity::active_high
                               ? "normal"
                               : "inversed";
    if (!detail_pwm::write_text(polarity_.get(), polarity) ||
        !detail_pwm::write_number(period_.get(),
                                   configuration.requested_period.value)) {
      return failure_nanoseconds(errno == 0 ? EIO : errno,
                                 detail::operation::configure);
    }
    std::uint64_t actual{};
    if (!detail_pwm::read_number(period_.get(), actual) || actual == 0U ||
        !detail_pwm::within_error(configuration.requested_period.value, actual,
                                   configuration.max_error_ppm)) {
      return failure_nanoseconds(EINVAL, detail::operation::configure);
    }
    if (!detail_pwm::write_number(duty_.get(), 0U)) {
      return failure_nanoseconds(errno == 0 ? EIO : errno,
                                 detail::operation::configure);
    }
    actual_period_ = hal::nanoseconds{actual};
    configured_ = true;
    return hal::result<hal::nanoseconds, error_type>::success(actual_period_);
  }

  [[nodiscard]] auto set_pulse_width(hal::nanoseconds pulse)
      -> hal::result<void, error_type> {
    if (!configured_) {
      return failure_void(EINVAL, detail::operation::configure);
    }
    if (pulse.value > actual_period_.value ||
        !detail_pwm::write_number(duty_.get(), pulse.value)) {
      return failure_void(EINVAL, detail::operation::write);
    }
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] auto enable_output() -> hal::result<void, error_type> {
    if (!configured_ || !detail_pwm::write_number(enable_.get(), 1U)) {
      return failure_void(EINVAL, detail::operation::write);
    }
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] auto disable_output() -> hal::result<void, error_type> {
    if (!detail_pwm::write_number(enable_.get(), 0U)) {
      return failure_void(errno == 0 ? EIO : errno, detail::operation::write);
    }
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] hal::nanoseconds actual_period() const noexcept {
    return actual_period_;
  }

 private:
  Output(detail::owned_fd chip, detail::owned_fd pwm_dir, detail::owned_fd period,
         detail::owned_fd duty, detail::owned_fd polarity, detail::owned_fd enable,
         std::uint32_t channel, bool owns_export) noexcept
      : chip_{std::move(chip)}, pwm_dir_{std::move(pwm_dir)},
        period_{std::move(period)}, duty_{std::move(duty)},
        polarity_{std::move(polarity)}, enable_{std::move(enable)},
        channel_{channel}, owns_export_{owns_export} {}

  [[nodiscard]] static auto failure(int native_error, detail::operation operation)
      -> hal::result<Output, error_type> {
    return hal::result<Output, error_type>::failure(
        detail_pwm::failure(native_error, operation));
  }
  [[nodiscard]] static auto failure_nanoseconds(
      int native_error, detail::operation operation)
      -> hal::result<hal::nanoseconds, error_type> {
    return hal::result<hal::nanoseconds, error_type>::failure(
        detail_pwm::failure(native_error, operation));
  }
  [[nodiscard]] static auto failure_void(int native_error,
                                         detail::operation operation)
      -> hal::result<void, error_type> {
    return hal::result<void, error_type>::failure(
        detail_pwm::failure(native_error, operation));
  }

  detail::owned_fd chip_{};
  detail::owned_fd pwm_dir_{};
  detail::owned_fd period_{};
  detail::owned_fd duty_{};
  detail::owned_fd polarity_{};
  detail::owned_fd enable_{};
  std::uint32_t channel_{};
  bool owns_export_{};
  bool configured_{};
  hal::nanoseconds actual_period_{};
};

static_assert(hal::pwm::Output<Output>);

}  // namespace hal::linux::pwm
