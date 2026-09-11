#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/adc.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include <array>
#include <charconv>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <fcntl.h>
#include <hal/foundation/result.hpp>
#include <hal/foundation/span.hpp>
#include <hal/detail/error.hpp>
#include <hal/detail/fd.hpp>
#include <hal/detail/io.hpp>
#include <cstring>
#include <limits>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <utility>

namespace hal::linux::adc {

using error_type = detail::error<hal::adc::error_kind>;

struct config {
  const char* device{};
  const char* buffer_length{};
  const char* buffer_enable{};
  std::uint32_t buffer_length_scans{};
  std::size_t scan_bytes{};
  std::size_t channel_offset{};
  std::size_t channel_storage_bytes{};
  std::uint8_t valid_bits{};
  std::uint8_t shift{};
  bool big_endian{};
  std::int64_t scale_numerator_microvolts{1};
  std::uint32_t scale_denominator{1U};
  std::int64_t offset_microvolts{};
};

namespace detail_adc {

[[nodiscard]] inline error_type failure(int native_error,
                                        detail::operation operation) noexcept {
  const auto kind = native_error == EAGAIN || native_error == EWOULDBLOCK
                        ? hal::adc::error_kind::not_ready
                        : native_error == ENODEV || native_error == ENOENT
                              ? hal::adc::error_kind::unavailable
                              : native_error == EINVAL || native_error == ENOTSUP
                                    ? hal::adc::error_kind::configuration
                                    : hal::adc::error_kind::io;
  return error_type{kind, native_error, operation};
}

[[nodiscard]] inline bool write_sysfs(const char* path, const char* value) noexcept {
  if (path == nullptr || value == nullptr) {
    return false;
  }
  const int fd = ::open(path, O_WRONLY | O_CLOEXEC);
  if (fd < 0) {
    return false;
  }
  const bool written = detail::write_all(
      fd, reinterpret_cast<const std::byte*>(value), std::strlen(value));
  (void)::close(fd);
  return written;
}

[[nodiscard]] inline hal::instant monotonic_now() noexcept {
  ::timespec value{};
  if (::clock_gettime(CLOCK_MONOTONIC, &value) < 0 || value.tv_sec < 0) {
    return {};
  }
  constexpr std::uint64_t nanoseconds_per_second{1'000'000'000U};
  const auto seconds = static_cast<std::uint64_t>(value.tv_sec);
  if (seconds > std::numeric_limits<std::uint64_t>::max() /
                   nanoseconds_per_second) {
    return {std::numeric_limits<std::uint64_t>::max()};
  }
  return {seconds * nanoseconds_per_second +
          static_cast<std::uint64_t>(value.tv_nsec)};
}

}  // namespace detail_adc

template <std::uint8_t ResolutionBits, std::int64_t MinimumMicrovolts,
          std::int64_t MaximumMicrovolts, std::size_t Capacity,
          std::size_t ReadBufferBytes = 4096U>
class BufferedChannel {
  static_assert(ResolutionBits > 0U && ResolutionBits <= 32U);
  static_assert(Capacity > 1U);
  static_assert(ReadBufferBytes > 0U);

 public:
  using error_type = linux::adc::error_type;

  [[nodiscard]] static constexpr hal::adc::characteristics properties() noexcept {
    return {ResolutionBits, {MinimumMicrovolts}, {MaximumMicrovolts}};
  }

  [[nodiscard]] static auto open(config configuration)
      -> hal::result<BufferedChannel, error_type> {
    if (configuration.device == nullptr || configuration.scan_bytes == 0U ||
        configuration.scan_bytes > ReadBufferBytes ||
        configuration.channel_storage_bytes == 0U ||
        configuration.channel_storage_bytes > 4U ||
        configuration.channel_offset >
            configuration.scan_bytes - configuration.channel_storage_bytes ||
        configuration.valid_bits == 0U ||
        configuration.valid_bits >
            configuration.channel_storage_bytes * 8U ||
        configuration.shift >= configuration.valid_bits ||
        configuration.scale_numerator_microvolts < 0 ||
        configuration.scale_denominator == 0U) {
      return failure(EINVAL, detail::operation::configure);
    }

    const int fd = ::open(configuration.device, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
      return failure(errno, detail::operation::open);
    }
    if (configuration.buffer_length != nullptr &&
        configuration.buffer_length_scans == 0U) {
      (void)::close(fd);
      return failure(EINVAL, detail::operation::configure);
    }
    if (configuration.buffer_length != nullptr &&
        !write_number(configuration.buffer_length,
                      configuration.buffer_length_scans)) {
      const int native_error = errno == 0 ? EIO : errno;
      (void)::close(fd);
      return failure(native_error, detail::operation::configure);
    }
    bool owns_buffer = false;
    if (configuration.buffer_enable != nullptr) {
      if (!detail_adc::write_sysfs(configuration.buffer_enable, "1")) {
        const int native_error = errno == 0 ? EIO : errno;
        (void)::close(fd);
        return failure(native_error, detail::operation::configure);
      }
      owns_buffer = true;
    }
    return hal::result<BufferedChannel, error_type>::success(
        BufferedChannel{detail::owned_fd{fd}, configuration, owns_buffer});
  }

  BufferedChannel(const BufferedChannel&) = delete;
  BufferedChannel& operator=(const BufferedChannel&) = delete;
  BufferedChannel(BufferedChannel&& other) noexcept
      : fd_{std::move(other.fd_)}, configuration_{other.configuration_},
        owns_buffer_{other.owns_buffer_}, pending_{other.pending_},
        pending_size_{other.pending_size_}, ring_{other.ring_}, produced_{other.produced_},
        consumed_{other.consumed_}, dropped_{other.dropped_}, sequence_{other.sequence_} {
    other.owns_buffer_ = false;
    other.pending_size_ = 0U;
  }
  BufferedChannel& operator=(BufferedChannel&&) = delete;

  ~BufferedChannel() {
    if (owns_buffer_) {
      (void)detail_adc::write_sysfs(configuration_.buffer_enable, "0");
    }
  }

  [[nodiscard]] auto service()
      -> hal::result<std::size_t, error_type> {
    if (!fd_.valid()) {
      return failure_size(EBADF, detail::operation::read);
    }
    const ssize_t count = ::read(fd_.get(), input_.data(), input_.size());
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
      return hal::result<std::size_t, error_type>::success(0U);
    }
    if (count < 0) {
      return failure_size(errno, detail::operation::read);
    }
    std::size_t published = 0U;
    std::size_t offset = 0U;
    const std::size_t bytes = static_cast<std::size_t>(count);
    while (offset < bytes) {
      const std::size_t needed = configuration_.scan_bytes - pending_size_;
      const std::size_t available = bytes - offset;
      const std::size_t copied = needed < available ? needed : available;
      for (std::size_t index = 0U; index < copied; ++index) {
        pending_[pending_size_ + index] = input_[offset + index];
      }
      pending_size_ += copied;
      offset += copied;
      if (pending_size_ == configuration_.scan_bytes) {
        publish(decode_raw());
        pending_size_ = 0U;
        ++published;
      }
    }
    return hal::result<std::size_t, error_type>::success(published);
  }

  [[nodiscard]] auto read_raw()
      -> hal::result<hal::adc::raw_sample, error_type> {
    if (produced_ == 0U) {
      return hal::result<hal::adc::raw_sample, error_type>::failure(
          detail_adc::failure(EAGAIN, detail::operation::read));
    }
    return hal::result<hal::adc::raw_sample, error_type>::success(
        ring_[(produced_ - 1U) % Capacity]);
  }

  [[nodiscard]] auto read_voltage()
      -> hal::result<hal::adc::voltage_sample, error_type> {
    auto raw = read_raw();
    if (!raw) {
      return hal::result<hal::adc::voltage_sample, error_type>::failure(
          std::move(raw).error());
    }
    const hal::adc::raw_sample sample = raw.value();
    const std::uint64_t raw_value = sample.value;
    const std::uint64_t scale =
        static_cast<std::uint64_t>(configuration_.scale_numerator_microvolts);
    const std::uint64_t maximum = std::numeric_limits<std::uint64_t>::max();
    if (scale != 0U && raw_value > maximum / scale) {
      return hal::result<hal::adc::voltage_sample, error_type>::failure(
          detail_adc::failure(EOVERFLOW, detail::operation::read));
    }
    const std::uint64_t scaled =
        (raw_value * scale) /
        static_cast<std::uint64_t>(configuration_.scale_denominator);
    if (scaled > static_cast<std::uint64_t>(
                      std::numeric_limits<std::int64_t>::max())) {
      return hal::result<hal::adc::voltage_sample, error_type>::failure(
          detail_adc::failure(EOVERFLOW, detail::operation::read));
    }
    const std::int64_t scaled_voltage = static_cast<std::int64_t>(scaled);
    if ((configuration_.offset_microvolts > 0 &&
         scaled_voltage > std::numeric_limits<std::int64_t>::max() -
                              configuration_.offset_microvolts) ||
        (configuration_.offset_microvolts < 0 &&
         scaled_voltage < std::numeric_limits<std::int64_t>::min() -
                              configuration_.offset_microvolts)) {
      return hal::result<hal::adc::voltage_sample, error_type>::failure(
          detail_adc::failure(EOVERFLOW, detail::operation::read));
    }
    const std::int64_t voltage = scaled_voltage + configuration_.offset_microvolts;
    return hal::result<hal::adc::voltage_sample, error_type>::success(
        hal::adc::voltage_sample{
            hal::microvolts{voltage}, sample.captured_at, sample.sequence});
  }

  [[nodiscard]] auto try_read(hal::span<hal::adc::raw_sample> output)
      -> hal::result<hal::adc::stream_read, error_type> {
    if (!output.valid()) {
      return failure_stream(EINVAL, detail::operation::read);
    }
    const std::uint64_t available = produced_ - consumed_;
    const std::size_t count =
        available < output.size() ? static_cast<std::size_t>(available)
                                  : output.size();
    for (std::size_t index = 0U; index < count; ++index) {
      output[index] = ring_[(consumed_ + index) % Capacity];
    }
    consumed_ += count;
    const std::uint64_t dropped = dropped_;
    dropped_ = 0U;
    return hal::result<hal::adc::stream_read, error_type>::success({count, dropped});
  }

  [[nodiscard]] int native_fd() const noexcept { return fd_.get(); }

 private:
  explicit BufferedChannel(detail::owned_fd fd, config configuration,
                           bool owns_buffer) noexcept
      : fd_{std::move(fd)}, configuration_{configuration},
        owns_buffer_{owns_buffer} {}

  [[nodiscard]] static bool write_number(const char* path,
                                         std::uint32_t value) noexcept {
    char buffer[16]{};
    const auto converted = std::to_chars(buffer, buffer + sizeof(buffer), value);
    if (converted.ec != std::errc{}) {
      return false;
    }
    return detail_adc::write_sysfs(path, buffer);
  }

  [[nodiscard]] std::uint32_t decode_raw() const noexcept {
    std::uint32_t value = 0U;
    if (configuration_.big_endian) {
      for (std::size_t index = 0U; index < configuration_.channel_storage_bytes;
           ++index) {
        value = (value << 8U) |
                std::to_integer<std::uint8_t>(
                    pending_[configuration_.channel_offset + index]);
      }
    } else {
      for (std::size_t index = 0U; index < configuration_.channel_storage_bytes;
           ++index) {
        value |= static_cast<std::uint32_t>(std::to_integer<std::uint8_t>(
                     pending_[configuration_.channel_offset + index]))
                  << (8U * index);
      }
    }
    value >>= configuration_.shift;
    const std::uint32_t mask = configuration_.valid_bits == 32U
                                   ? std::numeric_limits<std::uint32_t>::max()
                                   : (std::uint32_t{1U} << configuration_.valid_bits) - 1U;
    return value & mask;
  }

  void publish(std::uint32_t value) noexcept {
    if (produced_ - consumed_ >= Capacity) {
      ++consumed_;
      ++dropped_;
    }
    ring_[produced_ % Capacity] =
        {value, detail_adc::monotonic_now(), sequence_++};
    ++produced_;
  }

  [[nodiscard]] static auto failure(int native_error, detail::operation operation)
      -> hal::result<BufferedChannel, error_type> {
    return hal::result<BufferedChannel, error_type>::failure(
        detail_adc::failure(native_error, operation));
  }
  [[nodiscard]] static auto failure_size(int native_error,
                                         detail::operation operation)
      -> hal::result<std::size_t, error_type> {
    return hal::result<std::size_t, error_type>::failure(
        detail_adc::failure(native_error, operation));
  }
  [[nodiscard]] static auto failure_stream(int native_error,
                                           detail::operation operation)
      -> hal::result<hal::adc::stream_read, error_type> {
    return hal::result<hal::adc::stream_read, error_type>::failure(
        detail_adc::failure(native_error, operation));
  }

  detail::owned_fd fd_{};
  config configuration_{};
  bool owns_buffer_{};
  std::array<std::byte, ReadBufferBytes> input_{};
  std::array<std::byte, ReadBufferBytes> pending_{};
  std::size_t pending_size_{};
  std::array<hal::adc::raw_sample, Capacity> ring_{};
  std::uint64_t produced_{};
  std::uint64_t consumed_{};
  std::uint64_t dropped_{};
  std::uint64_t sequence_{};
};

static_assert(hal::adc::Channel<BufferedChannel<12U, 0, 3300000, 16U>>);
static_assert(
    hal::adc::ContinuousChannel<BufferedChannel<12U, 0, 3300000, 16U>>);

}  // namespace hal::linux::adc
