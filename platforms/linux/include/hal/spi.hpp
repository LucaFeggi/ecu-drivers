#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/spi.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <hal/foundation/result.hpp>
#include <hal/foundation/span.hpp>
#include <hal/contracts/spi_bus.hpp>
#include <limits>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <utility>

#include <hal/detail/error.hpp>
#include <hal/detail/fd.hpp>
#include "support.hpp"

namespace hal::linux::spi {

using error_type = detail::error<hal::spi::error_kind>;

namespace detail_spi {

[[nodiscard]] inline hal::spi::error_kind map_errno(int native_error) noexcept {
  switch (native_error) {
    case EINVAL:
    case ENOTSUP:
      return hal::spi::error_kind::configuration;
    case EBUSY:
    case EAGAIN:
      return hal::spi::error_kind::busy;
    case ETIMEDOUT:
      return hal::spi::error_kind::timeout;
    default:
      return hal::spi::error_kind::io;
  }
}

[[nodiscard]] inline error_type failure(int native_error,
                                        detail::operation operation) noexcept {
  return error_type{map_errno(native_error), native_error, operation};
}

[[nodiscard]] inline std::uint32_t mode_value(hal::spi::mode selected_mode) noexcept {
  switch (selected_mode) {
    case hal::spi::mode::mode0:
      return SPI_MODE_0;
    case hal::spi::mode::mode1:
      return SPI_MODE_1;
    case hal::spi::mode::mode2:
      return SPI_MODE_2;
    case hal::spi::mode::mode3:
      return SPI_MODE_3;
  }
  return SPI_MODE_0;
}

[[nodiscard]] inline std::uint16_t delay_microseconds(
    hal::nanoseconds duration, bool& valid) noexcept {
  constexpr std::uint64_t nanoseconds_per_microsecond{1'000U};
  const std::uint64_t rounding = nanoseconds_per_microsecond - 1U;
  if (duration.value > std::numeric_limits<std::uint64_t>::max() - rounding) {
    valid = false;
    return 0U;
  }
  const std::uint64_t rounded =
      (duration.value + rounding) / nanoseconds_per_microsecond;
  if (rounded > std::numeric_limits<std::uint16_t>::max()) {
    valid = false;
    return 0U;
  }
  return static_cast<std::uint16_t>(rounded);
}

[[nodiscard]] inline std::uint64_t pointer_value(const void* pointer) noexcept {
  return static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(pointer));
}

}  // namespace detail_spi

template <std::size_t MaxOperations = hal::linux::max_transaction_operations,
          std::size_t ScratchBytes = 4096U>
class transfer_engine {
  static_assert(MaxOperations > 0U);
  static_assert(MaxOperations <= 8U);
  static_assert(ScratchBytes > 0U);

 public:
  using error_type = linux::spi::error_type;

  [[nodiscard]] static auto open(const char* device)
      -> hal::result<transfer_engine, error_type> {
    if (device == nullptr) {
      return failure(EINVAL, detail::operation::open);
    }
    const int fd = ::open(device, O_RDWR | O_CLOEXEC);
    if (fd < 0) {
      return failure(errno, detail::operation::open);
    }
    return hal::result<transfer_engine, error_type>::success(
        transfer_engine{detail::owned_fd{fd}});
  }

  transfer_engine(const transfer_engine&) = delete;
  transfer_engine& operator=(const transfer_engine&) = delete;
  transfer_engine(transfer_engine&&) noexcept = default;
  transfer_engine& operator=(transfer_engine&&) noexcept = default;

  [[nodiscard]] auto configure(hal::spi::config8 configuration)
      -> hal::result<hal::hertz, error_type> {
    configured_ = false;
    if (configuration.max_frequency.value == 0U ||
        configuration.max_frequency.value >
            std::numeric_limits<std::uint32_t>::max()) {
      return failure_hertz(EINVAL, detail::operation::configure);
    }

    std::uint32_t mode = detail_spi::mode_value(configuration.clock_mode);
    if (configuration.order == hal::spi::bit_order::lsb_first) {
      mode |= SPI_LSB_FIRST;
    }
    if (::ioctl(fd_.get(), SPI_IOC_WR_MODE32, &mode) < 0) {
      return failure_hertz(errno, detail::operation::ioctl);
    }

    std::uint8_t bits = 8U;
    if (::ioctl(fd_.get(), SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
      return failure_hertz(errno, detail::operation::ioctl);
    }

    std::uint32_t requested =
        static_cast<std::uint32_t>(configuration.max_frequency.value);
    if (::ioctl(fd_.get(), SPI_IOC_WR_MAX_SPEED_HZ, &requested) < 0) {
      return failure_hertz(errno, detail::operation::ioctl);
    }

    std::uint32_t actual{};
    if (::ioctl(fd_.get(), SPI_IOC_RD_MAX_SPEED_HZ, &actual) < 0 ||
        actual == 0U || actual > requested) {
      const int native_error = errno == 0 ? EINVAL : errno;
      return failure_hertz(native_error, detail::operation::ioctl);
    }

    configuration_ = configuration;
    actual_frequency_ = hal::hertz{actual};
    configured_ = true;
    return hal::result<hal::hertz, error_type>::success(actual_frequency_);
  }

  [[nodiscard]] auto transaction(hal::span<const hal::spi::operation8> operations)
      -> hal::result<void, error_type> {
    if (!configured_ || !operations.valid() ||
        operations.size() > MaxOperations) {
      return failure_void(EINVAL, detail::operation::configure);
    }
    if (operations.empty()) {
      return hal::result<void, error_type>::success();
    }

    for (std::size_t index = 0U; index < MaxOperations; ++index) {
      transfers_[index] = {};
      staged_[index] = {};
    }
    std::size_t transfer_count = 0U;
    std::size_t scratch_used = 0U;
    std::uint64_t expected_bytes = 0U;

    for (const hal::spi::operation8& operation : operations) {
      if (!operation.valid()) {
        return failure_void(EINVAL, detail::operation::configure);
      }
      if (operation.kind() == hal::spi::operation_kind::delay) {
        if (transfer_count == 0U) {
          return failure_void(EINVAL, detail::operation::configure);
        }
        bool valid_delay = true;
        const std::uint16_t delay =
            detail_spi::delay_microseconds(operation.delay(), valid_delay);
        if (!valid_delay ||
            static_cast<std::uint32_t>(transfers_[transfer_count - 1U].delay_usecs) +
                    delay >
                std::numeric_limits<std::uint16_t>::max()) {
          return failure_void(EINVAL, detail::operation::configure);
        }
        transfers_[transfer_count - 1U].delay_usecs =
            static_cast<__u16>(transfers_[transfer_count - 1U].delay_usecs + delay);
        continue;
      }

      if (transfer_count >= MaxOperations) {
        return failure_void(EINVAL, detail::operation::configure);
      }
      ::spi_ioc_transfer& transfer = transfers_[transfer_count];
      transfer.speed_hz = static_cast<__u32>(actual_frequency_.value);
      transfer.bits_per_word = 8U;

      std::size_t length = 0U;
      if (operation.kind() == hal::spi::operation_kind::write) {
        length = operation.tx().size();
        transfer.tx_buf = detail_spi::pointer_value(operation.tx().data());
      } else if (operation.kind() == hal::spi::operation_kind::read) {
        length = operation.rx().size();
        transfer.rx_buf = detail_spi::pointer_value(operation.rx().data());
        if (configuration_.read_fill != std::byte{0}) {
          if (!stage_tx(transfer, length, scratch_used)) {
            return failure_void(EINVAL, detail::operation::configure);
          }
          std::fill_n(tx_scratch_.data() + scratch_used - length, length,
                      configuration_.read_fill);
        }
      } else {
        const std::size_t tx_size = operation.tx().size();
        const std::size_t rx_size = operation.rx().size();
        length = tx_size > rx_size ? tx_size : rx_size;
        if (tx_size == rx_size) {
          transfer.tx_buf = detail_spi::pointer_value(operation.tx().data());
          transfer.rx_buf = detail_spi::pointer_value(operation.rx().data());
        } else {
          current_transfer_ = transfer_count;
          if (!stage_transfer(transfer, operation, length, scratch_used)) {
            return failure_void(EINVAL, detail::operation::configure);
          }
        }
      }

      if (length > std::numeric_limits<__u32>::max()) {
        return failure_void(EINVAL, detail::operation::configure);
      }
      transfer.len = static_cast<__u32>(length);
      expected_bytes += length;
      ++transfer_count;
    }

    const long status = invoke(transfer_count);
    if (status < 0) {
      return failure_void(errno, detail::operation::ioctl);
    }
    if (static_cast<std::uint64_t>(status) != expected_bytes) {
      return failure_void(EIO, detail::operation::ioctl);
    }

    for (std::size_t index = 0U; index < transfer_count; ++index) {
      if (staged_[index].destination != nullptr) {
        std::memcpy(staged_[index].destination,
                    rx_scratch_.data() + staged_[index].rx_offset,
                    staged_[index].size);
      }
    }
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] auto write(hal::span<const std::byte> data)
      -> hal::result<void, error_type> {
    const hal::spi::operation8 operation = hal::spi::operation8::write(data);
    return transaction({&operation, 1U});
  }

  [[nodiscard]] auto read(hal::span<std::byte> data, std::byte fill)
      -> hal::result<void, error_type> {
    const hal::spi::config8 previous = configuration_;
    configuration_.read_fill = fill;
    const hal::spi::operation8 operation = hal::spi::operation8::read(data);
    auto result = transaction({&operation, 1U});
    configuration_.read_fill = previous.read_fill;
    return result;
  }

  [[nodiscard]] auto transfer(hal::span<const std::byte> tx,
                              hal::span<std::byte> rx)
      -> hal::result<void, error_type> {
    const hal::spi::operation8 operation = hal::spi::operation8::transfer(tx, rx);
    return transaction({&operation, 1U});
  }

  [[nodiscard]] auto flush() -> hal::result<void, error_type> {
    if (!configured_) {
      return failure_void(EINVAL, detail::operation::configure);
    }
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] constexpr bool supports_lsb_first() const noexcept { return true; }
  [[nodiscard]] hal::hertz actual_frequency() const noexcept {
    return actual_frequency_;
  }
  [[nodiscard]] int native_fd() const noexcept { return fd_.get(); }

 private:
  struct staged_output {
    std::byte* destination{};
    std::size_t rx_offset{};
    std::size_t size{};
  };

  explicit transfer_engine(detail::owned_fd fd) noexcept : fd_{std::move(fd)} {}

  [[nodiscard]] bool stage_tx(::spi_ioc_transfer& transfer, std::size_t size,
                              std::size_t& scratch_used) noexcept {
    if (size > ScratchBytes - scratch_used) {
      return false;
    }
    transfer.tx_buf = detail_spi::pointer_value(tx_scratch_.data() + scratch_used);
    scratch_used += size;
    return true;
  }

  [[nodiscard]] bool stage_transfer(
      ::spi_ioc_transfer& transfer, const hal::spi::operation8& operation,
      std::size_t length, std::size_t& scratch_used) noexcept {
    if (length > ScratchBytes - scratch_used) {
      return false;
    }
    const std::size_t tx_offset = scratch_used;
    std::byte* tx_destination = tx_scratch_.data() + tx_offset;
    scratch_used += length;
    if (length > ScratchBytes - scratch_used) {
      return false;
    }
    const std::size_t rx_offset = scratch_used;
    std::byte* rx_destination = rx_scratch_.data() + rx_offset;
    scratch_used += length;

    std::fill_n(tx_destination, length, configuration_.read_fill);
    if (!operation.tx().empty()) {
      std::memcpy(tx_destination, operation.tx().data(), operation.tx().size());
    }
    std::fill_n(rx_destination, length, std::byte{0});
    transfer.tx_buf = detail_spi::pointer_value(tx_destination);
    transfer.rx_buf = detail_spi::pointer_value(rx_destination);
    staged_[current_transfer_].destination = operation.rx().data();
    staged_[current_transfer_].rx_offset = rx_offset;
    staged_[current_transfer_].size = operation.rx().size();
    return true;
  }

  [[nodiscard]] long invoke(std::size_t count) noexcept {
    // The ioctl number encodes the array length, so dispatch through the small
    // fixed set of legal message counts instead of passing a runtime-sized ABI.
    switch (count) {
      case 1U:
        return ::ioctl(fd_.get(), SPI_IOC_MESSAGE(1), transfers_.data());
      case 2U:
        return ::ioctl(fd_.get(), SPI_IOC_MESSAGE(2), transfers_.data());
      case 3U:
        return ::ioctl(fd_.get(), SPI_IOC_MESSAGE(3), transfers_.data());
      case 4U:
        return ::ioctl(fd_.get(), SPI_IOC_MESSAGE(4), transfers_.data());
      case 5U:
        return ::ioctl(fd_.get(), SPI_IOC_MESSAGE(5), transfers_.data());
      case 6U:
        return ::ioctl(fd_.get(), SPI_IOC_MESSAGE(6), transfers_.data());
      case 7U:
        return ::ioctl(fd_.get(), SPI_IOC_MESSAGE(7), transfers_.data());
      case 8U:
        return ::ioctl(fd_.get(), SPI_IOC_MESSAGE(8), transfers_.data());
      default:
        errno = EINVAL;
        return -1L;
    }
  }

  [[nodiscard]] static auto failure(int native_error, detail::operation operation)
      -> hal::result<transfer_engine, error_type> {
    return hal::result<transfer_engine, error_type>::failure(
        detail_spi::failure(native_error, operation));
  }

  [[nodiscard]] static auto failure_hertz(int native_error,
                                          detail::operation operation)
      -> hal::result<hal::hertz, error_type> {
    return hal::result<hal::hertz, error_type>::failure(
        detail_spi::failure(native_error, operation));
  }

  [[nodiscard]] static auto failure_void(int native_error,
                                         detail::operation operation)
      -> hal::result<void, error_type> {
    return hal::result<void, error_type>::failure(
        detail_spi::failure(native_error, operation));
  }

  detail::owned_fd fd_{};
  hal::spi::config8 configuration_{};
  hal::hertz actual_frequency_{};
  std::array<::spi_ioc_transfer, MaxOperations> transfers_{};
  std::array<staged_output, MaxOperations> staged_{};
  std::array<std::byte, ScratchBytes> tx_scratch_{};
  std::array<std::byte, ScratchBytes> rx_scratch_{};
  std::size_t current_transfer_{};
  bool configured_{};
};

template <std::size_t MaxOperations = hal::linux::max_transaction_operations,
          std::size_t ScratchBytes = 4096U>
class Bus {
 public:
  using error_type = linux::spi::error_type;
  static constexpr bool supports_lsb_first{true};

  [[nodiscard]] static auto open(const char* device)
      -> hal::result<Bus, error_type> {
    auto engine = transfer_engine<MaxOperations, ScratchBytes>::open(device);
    if (!engine) {
      return hal::result<Bus, error_type>::failure(std::move(engine).error());
    }
    return hal::result<Bus, error_type>::success(
        Bus{std::move(engine).value()});
  }

  Bus(const Bus&) = delete;
  Bus& operator=(const Bus&) = delete;
  Bus(Bus&&) noexcept = default;
  Bus& operator=(Bus&&) noexcept = default;

  [[nodiscard]] auto configure(hal::spi::config8 configuration)
      -> hal::result<hal::hertz, error_type> {
    return engine_.configure(configuration);
  }
  [[nodiscard]] auto write(hal::span<const std::byte> data)
      -> hal::result<void, error_type> {
    return engine_.write(data);
  }
  [[nodiscard]] auto read(hal::span<std::byte> data, std::byte fill)
      -> hal::result<void, error_type> {
    return engine_.read(data, fill);
  }
  [[nodiscard]] auto transfer(hal::span<const std::byte> tx,
                              hal::span<std::byte> rx)
      -> hal::result<void, error_type> {
    return engine_.transfer(tx, rx);
  }
  [[nodiscard]] auto flush() -> hal::result<void, error_type> {
    return engine_.flush();
  }
  [[nodiscard]] int native_fd() const noexcept { return engine_.native_fd(); }

 private:
  explicit Bus(transfer_engine<MaxOperations, ScratchBytes> engine) noexcept
      : engine_{std::move(engine)} {}

  transfer_engine<MaxOperations, ScratchBytes> engine_{};
};

template <std::size_t MaxOperations = hal::linux::max_transaction_operations,
          std::size_t ScratchBytes = 4096U>
class Device {
 public:
  using error_type = linux::spi::error_type;

  struct config {
    const char* device{};
    hal::spi::config8 bus{};
    hal::nanoseconds cs_setup{};
    hal::nanoseconds cs_hold{};
  };

  [[nodiscard]] static auto open(config configuration)
      -> hal::result<Device, error_type> {
    if (configuration.cs_setup.value != 0U ||
        configuration.cs_hold.value != 0U) {
      return failure(EINVAL, detail::operation::configure);
    }
    auto engine = transfer_engine<MaxOperations, ScratchBytes>::open(
        configuration.device);
    if (!engine) {
      return hal::result<Device, error_type>::failure(std::move(engine).error());
    }
    auto device = Device{std::move(engine).value()};
    auto configured = device.engine_.configure(configuration.bus);
    if (!configured) {
      return hal::result<Device, error_type>::failure(
          std::move(configured).error());
    }
    return hal::result<Device, error_type>::success(std::move(device));
  }

  Device(const Device&) = delete;
  Device& operator=(const Device&) = delete;
  Device(Device&&) noexcept = default;
  Device& operator=(Device&&) noexcept = default;

  [[nodiscard]] auto transaction(hal::span<const hal::spi::operation8> operations)
      -> hal::result<void, error_type> {
    return engine_.transaction(operations);
  }
  [[nodiscard]] hal::hertz actual_frequency() const noexcept {
    return engine_.actual_frequency();
  }
  [[nodiscard]] int native_fd() const noexcept { return engine_.native_fd(); }

 private:
  explicit Device(transfer_engine<MaxOperations, ScratchBytes> engine) noexcept
      : engine_{std::move(engine)} {}

  [[nodiscard]] static auto failure(int native_error, detail::operation operation)
      -> hal::result<Device, error_type> {
    return hal::result<Device, error_type>::failure(
        detail_spi::failure(native_error, operation));
  }

  transfer_engine<MaxOperations, ScratchBytes> engine_{};
};

static_assert(hal::spi::Bus8<Bus<>>);
static_assert(hal::spi::LsbFirstBus8<Bus<>>);
static_assert(hal::spi::Device8<Device<>>);

}  // namespace hal::linux::spi
