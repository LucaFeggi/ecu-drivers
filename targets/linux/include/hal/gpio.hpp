#pragma once

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <hal/gpio.hpp>
#include <hal/foundation/result.hpp>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <utility>

#include <hal/linux/detail/error.hpp>
#include <hal/linux/detail/fd.hpp>

namespace hal::linux::gpio {

using error_type = detail::error<hal::gpio::error_kind>;

namespace detail_gpio {

[[nodiscard]] inline error_type failure(int native_error,
                                        detail::operation operation) noexcept {
  const auto kind = native_error == ENODEV || native_error == ENOENT ||
                            native_error == EACCES || native_error == EBUSY
                        ? hal::gpio::error_kind::unavailable
                        : hal::gpio::error_kind::io;
  return error_type{kind, native_error, operation};
}

[[nodiscard]] inline bool set_consumer(char (&destination)[GPIO_MAX_NAME_SIZE],
                                       const char* consumer) noexcept {
  if (consumer == nullptr) {
    return true;
  }
  const std::size_t length = ::strnlen(consumer, GPIO_MAX_NAME_SIZE);
  if (length >= GPIO_MAX_NAME_SIZE) {
    return false;
  }
  std::memcpy(destination, consumer, length);
  destination[length] = '\0';
  return true;
}

[[nodiscard]] inline std::uint64_t flag_for_edge(hal::gpio::edge edge) noexcept {
  switch (edge) {
    case hal::gpio::edge::rising:
      return GPIO_V2_LINE_FLAG_EDGE_RISING;
    case hal::gpio::edge::falling:
      return GPIO_V2_LINE_FLAG_EDGE_FALLING;
    case hal::gpio::edge::both:
      return GPIO_V2_LINE_FLAG_EDGE_RISING | GPIO_V2_LINE_FLAG_EDGE_FALLING;
  }
  return 0U;
}

}  // namespace detail_gpio

struct line_config {
  const char* chip{};
  std::uint32_t offset{};
  const char* consumer{};
  bool active_low{};
};

class OutputLine {
 public:
  using error_type = linux::gpio::error_type;

  [[nodiscard]] static auto open(line_config configuration,
                                 hal::gpio::level initial = hal::gpio::level::low)
      -> hal::result<OutputLine, error_type> {
    if (configuration.chip == nullptr) {
      return failure(EINVAL, detail::operation::open);
    }
    const int chip_fd = ::open(configuration.chip, O_RDONLY | O_CLOEXEC);
    if (chip_fd < 0) {
      return failure(errno, detail::operation::open);
    }

    ::gpio_v2_line_request request{};
    request.offsets[0] = configuration.offset;
    request.num_lines = 1U;
    if (!detail_gpio::set_consumer(request.consumer, configuration.consumer)) {
      (void)::close(chip_fd);
      return failure(EINVAL, detail::operation::configure);
    }
    request.config.flags = static_cast<std::uint64_t>(GPIO_V2_LINE_FLAG_OUTPUT) |
                           (configuration.active_low
                                ? static_cast<std::uint64_t>(GPIO_V2_LINE_FLAG_ACTIVE_LOW)
                                : 0U);
    request.config.num_attrs = 1U;
    request.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
    request.config.attrs[0].attr.values = initial == hal::gpio::level::high ? 1U : 0U;
    request.config.attrs[0].mask = 1U;
    if (::ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &request) < 0) {
      const int native_error = errno;
      (void)::close(chip_fd);
      return failure(native_error, detail::operation::ioctl);
    }
    (void)::close(chip_fd);
    return hal::result<OutputLine, error_type>::success(
        OutputLine{detail::owned_fd{request.fd}});
  }

  OutputLine(const OutputLine&) = delete;
  OutputLine& operator=(const OutputLine&) = delete;
  OutputLine(OutputLine&&) noexcept = default;
  OutputLine& operator=(OutputLine&&) noexcept = default;

  [[nodiscard]] auto write(hal::gpio::level value)
      -> hal::result<void, error_type> {
    ::gpio_v2_line_values values{};
    values.bits = value == hal::gpio::level::high ? 1U : 0U;
    values.mask = 1U;
    if (::ioctl(fd_.get(), GPIO_V2_LINE_SET_VALUES_IOCTL, &values) < 0) {
      return hal::result<void, error_type>::failure(
          detail_gpio::failure(errno, detail::operation::ioctl));
    }
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] int native_fd() const noexcept { return fd_.get(); }

 private:
  explicit OutputLine(detail::owned_fd fd) noexcept : fd_{std::move(fd)} {}

  [[nodiscard]] static auto failure(int native_error, detail::operation operation)
      -> hal::result<OutputLine, error_type> {
    return hal::result<OutputLine, error_type>::failure(
        detail_gpio::failure(native_error, operation));
  }

  detail::owned_fd fd_{};
};

class InputLine {
 public:
  using error_type = linux::gpio::error_type;

  [[nodiscard]] static auto open(line_config configuration)
      -> hal::result<InputLine, error_type> {
    if (configuration.chip == nullptr) {
      return failure(EINVAL, detail::operation::open);
    }
    const int chip_fd = ::open(configuration.chip, O_RDONLY | O_CLOEXEC);
    if (chip_fd < 0) {
      return failure(errno, detail::operation::open);
    }
    ::gpio_v2_line_request request{};
    request.offsets[0] = configuration.offset;
    request.num_lines = 1U;
    if (!detail_gpio::set_consumer(request.consumer, configuration.consumer)) {
      (void)::close(chip_fd);
      return failure(EINVAL, detail::operation::configure);
    }
    request.config.flags = static_cast<std::uint64_t>(GPIO_V2_LINE_FLAG_INPUT) |
                           (configuration.active_low
                                ? static_cast<std::uint64_t>(GPIO_V2_LINE_FLAG_ACTIVE_LOW)
                                : 0U);
    if (::ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &request) < 0) {
      const int native_error = errno;
      (void)::close(chip_fd);
      return failure(native_error, detail::operation::ioctl);
    }
    (void)::close(chip_fd);
    return hal::result<InputLine, error_type>::success(
        InputLine{detail::owned_fd{request.fd}});
  }

  InputLine(const InputLine&) = delete;
  InputLine& operator=(const InputLine&) = delete;
  InputLine(InputLine&&) noexcept = default;
  InputLine& operator=(InputLine&&) noexcept = default;

  [[nodiscard]] auto read() -> hal::result<hal::gpio::level, error_type> {
    ::gpio_v2_line_values values{};
    values.mask = 1U;
    if (::ioctl(fd_.get(), GPIO_V2_LINE_GET_VALUES_IOCTL, &values) < 0) {
      return hal::result<hal::gpio::level, error_type>::failure(
          detail_gpio::failure(errno, detail::operation::ioctl));
    }
    return hal::result<hal::gpio::level, error_type>::success(
        (values.bits & 1U) != 0U ? hal::gpio::level::high
                                 : hal::gpio::level::low);
  }

  [[nodiscard]] int native_fd() const noexcept { return fd_.get(); }

 private:
  explicit InputLine(detail::owned_fd fd) noexcept : fd_{std::move(fd)} {}

  [[nodiscard]] static auto failure(int native_error, detail::operation operation)
      -> hal::result<InputLine, error_type> {
    return hal::result<InputLine, error_type>::failure(
        detail_gpio::failure(native_error, operation));
  }

  detail::owned_fd fd_{};
};

class EdgeLine {
 public:
  using error_type = linux::gpio::error_type;

  [[nodiscard]] static auto open(line_config configuration,
                                 hal::gpio::edge selected_edge)
      -> hal::result<EdgeLine, error_type> {
    if (configuration.chip == nullptr ||
        detail_gpio::flag_for_edge(selected_edge) == 0U) {
      return failure(EINVAL, detail::operation::configure);
    }
    const int chip_fd = ::open(configuration.chip, O_RDONLY | O_CLOEXEC);
    if (chip_fd < 0) {
      return failure(errno, detail::operation::open);
    }
    ::gpio_v2_line_request request{};
    request.offsets[0] = configuration.offset;
    request.num_lines = 1U;
    request.event_buffer_size = 16U;
    if (!detail_gpio::set_consumer(request.consumer, configuration.consumer)) {
      (void)::close(chip_fd);
      return failure(EINVAL, detail::operation::configure);
    }
    request.config.flags = static_cast<std::uint64_t>(GPIO_V2_LINE_FLAG_INPUT) |
                           detail_gpio::flag_for_edge(selected_edge) |
                           (configuration.active_low
                                ? static_cast<std::uint64_t>(GPIO_V2_LINE_FLAG_ACTIVE_LOW)
                                : 0U);
    if (::ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &request) < 0) {
      const int native_error = errno;
      (void)::close(chip_fd);
      return failure(native_error, detail::operation::ioctl);
    }
    (void)::close(chip_fd);
    return hal::result<EdgeLine, error_type>::success(
        EdgeLine{detail::owned_fd{request.fd}, request.config.flags});
  }

  EdgeLine(const EdgeLine&) = delete;
  EdgeLine& operator=(const EdgeLine&) = delete;
  EdgeLine(EdgeLine&&) noexcept = default;
  EdgeLine& operator=(EdgeLine&&) noexcept = default;

  [[nodiscard]] auto read() -> hal::result<hal::gpio::level, error_type> {
    ::gpio_v2_line_values values{};
    values.mask = 1U;
    if (::ioctl(fd_.get(), GPIO_V2_LINE_GET_VALUES_IOCTL, &values) < 0) {
      return hal::result<hal::gpio::level, error_type>::failure(
          detail_gpio::failure(errno, detail::operation::ioctl));
    }
    return hal::result<hal::gpio::level, error_type>::success(
        (values.bits & 1U) != 0U ? hal::gpio::level::high
                                 : hal::gpio::level::low);
  }

  [[nodiscard]] auto configure_edge(hal::gpio::edge selected_edge)
      -> hal::result<void, error_type> {
    const std::uint64_t edge_flags = detail_gpio::flag_for_edge(selected_edge);
    if (edge_flags == 0U) {
      return hal::result<void, error_type>::failure(
          detail_gpio::failure(EINVAL, detail::operation::configure));
    }
    ::gpio_v2_line_config config{};
    config.flags =
        (flags_ & static_cast<std::uint64_t>(GPIO_V2_LINE_FLAG_ACTIVE_LOW)) |
        static_cast<std::uint64_t>(GPIO_V2_LINE_FLAG_INPUT) | edge_flags;
    if (::ioctl(fd_.get(), GPIO_V2_LINE_SET_CONFIG_IOCTL, &config) < 0) {
      return hal::result<void, error_type>::failure(
          detail_gpio::failure(errno, detail::operation::configure));
    }
    flags_ = config.flags;
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] bool take_event() noexcept {
    ::gpio_v2_line_event event{};
    const ssize_t count = ::read(fd_.get(), &event, sizeof(event));
    if (count == static_cast<ssize_t>(sizeof(event))) {
      return true;
    }
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
      return false;
    }
    fault_ = true;
    return false;
  }

  [[nodiscard]] bool has_fault() const noexcept { return fault_; }
  [[nodiscard]] int native_fd() const noexcept { return fd_.get(); }

 private:
  explicit EdgeLine(detail::owned_fd fd, std::uint64_t flags = 0U) noexcept
      : fd_{std::move(fd)}, flags_{flags} {}

  [[nodiscard]] static auto failure(int native_error, detail::operation operation)
      -> hal::result<EdgeLine, error_type> {
    return hal::result<EdgeLine, error_type>::failure(
        detail_gpio::failure(native_error, operation));
  }

  detail::owned_fd fd_{};
  std::uint64_t flags_{};
  bool fault_{};
};

static_assert(hal::gpio::OutputPin<OutputLine>);
static_assert(hal::gpio::InputPin<InputLine>);
static_assert(hal::gpio::InputPin<EdgeLine>);
static_assert(hal::gpio::EdgeInput<EdgeLine>);

}  // namespace hal::linux::gpio
