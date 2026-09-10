#pragma once

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <fcntl.h>
#include <hal/foundation/result.hpp>
#include <hal/foundation/span.hpp>
#include <hal/i2c.hpp>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <utility>

#include <hal/linux/detail/error.hpp>
#include <hal/linux/detail/fd.hpp>
#include <hal/linux/support.hpp>

namespace hal::linux::i2c {

using error_type = detail::error<hal::i2c::error_kind>;

struct config {
  const char* device{};
  std::uint32_t timeout_ms{100U};
  std::uint32_t retries{};
};

namespace detail_i2c {

[[nodiscard]] inline hal::i2c::error_kind map_errno(int native_error) noexcept {
  switch (native_error) {
    case ENXIO:
    case EREMOTEIO:
      return hal::i2c::error_kind::no_acknowledge;
    case EAGAIN:
    case EBUSY:
      return hal::i2c::error_kind::busy;
    case ETIMEDOUT:
      return hal::i2c::error_kind::timeout;
    case EPROTO:
      return hal::i2c::error_kind::bus_error;
    case EINVAL:
      return hal::i2c::error_kind::configuration;
    default:
      return hal::i2c::error_kind::io;
  }
}

[[nodiscard]] inline error_type failure(int native_error,
                                        detail::operation operation) noexcept {
  return error_type{map_errno(native_error), native_error, operation};
}

[[nodiscard]] inline std::uint32_t timeout_units(std::uint32_t timeout_ms) noexcept {
  return (timeout_ms / 10U) + ((timeout_ms % 10U) == 0U ? 0U : 1U);
}

}  // namespace detail_i2c

template <std::size_t MaxOperations = hal::linux::max_transaction_operations>
class Controller {
  static_assert(MaxOperations > 0U);

 public:
  using error_type = linux::i2c::error_type;

  [[nodiscard]] static auto open(config configuration)
      -> hal::result<Controller, error_type> {
    if (configuration.device == nullptr || configuration.timeout_ms == 0U ||
        detail_i2c::timeout_units(configuration.timeout_ms) == 0U) {
      return failure(EINVAL, detail::operation::open);
    }

    const int fd = ::open(configuration.device, O_RDWR | O_CLOEXEC);
    if (fd < 0) {
      return failure(errno, detail::operation::open);
    }

    unsigned long functions{};
    if (::ioctl(fd, I2C_FUNCS, &functions) < 0) {
      const int native_error = errno;
      (void)::close(fd);
      return failure(native_error, detail::operation::ioctl);
    }
    if ((functions & I2C_FUNC_I2C) == 0U) {
      (void)::close(fd);
      return failure(ENOTSUP, detail::operation::configure);
    }

    const unsigned long timeout = detail_i2c::timeout_units(configuration.timeout_ms);
    if (::ioctl(fd, I2C_TIMEOUT, timeout) < 0 ||
        ::ioctl(fd, I2C_RETRIES,
                static_cast<unsigned long>(configuration.retries)) < 0) {
      const int native_error = errno;
      (void)::close(fd);
      return failure(native_error, detail::operation::configure);
    }

    return hal::result<Controller, error_type>::success(
        Controller{detail::owned_fd{fd}, functions});
  }

  Controller(const Controller&) = delete;
  Controller& operator=(const Controller&) = delete;
  Controller(Controller&&) noexcept = default;
  Controller& operator=(Controller&&) noexcept = default;

  [[nodiscard]] auto transaction(hal::i2c::address7 address,
                                 hal::span<const hal::i2c::operation> operations)
      -> hal::result<void, error_type> {
    return transaction_impl(address.value, false, operations);
  }

  [[nodiscard]] auto transaction(hal::i2c::address10 address,
                                 hal::span<const hal::i2c::operation> operations)
      -> hal::result<void, error_type> {
    if ((functions_ & I2C_FUNC_10BIT_ADDR) == 0U) {
      return failure_result(ENOTSUP, detail::operation::configure);
    }
    return transaction_impl(address.value, true, operations);
  }

  [[nodiscard]] int native_fd() const noexcept { return fd_.get(); }

 private:
  explicit Controller(detail::owned_fd fd, unsigned long functions) noexcept
      : fd_{std::move(fd)}, functions_{functions} {}

  [[nodiscard]] auto transaction_impl(
      std::uint16_t address, bool ten_bit,
      hal::span<const hal::i2c::operation> operations)
      -> hal::result<void, error_type> {
    if (!operations.valid() || operations.empty() ||
        operations.size() > MaxOperations || address > (ten_bit ? 0x3FFU : 0x7FU)) {
      return failure_result(EINVAL, detail::operation::configure);
    }

    std::array<::i2c_msg, MaxOperations> messages{};
    for (std::size_t index = 0U; index < operations.size(); ++index) {
      const hal::i2c::operation& operation = operations[index];
      if (!operation.valid() || operation.size() > 0xFFFFU) {
        return failure_result(EINVAL, detail::operation::configure);
      }
      messages[index].addr = address;
      messages[index].flags = ten_bit ? I2C_M_TEN : 0U;
      if (operation.dir() == hal::i2c::direction::read) {
        messages[index].flags = static_cast<__u16>(messages[index].flags | I2C_M_RD);
        messages[index].buf = reinterpret_cast<__u8*>(operation.read_buffer().data());
      } else {
        messages[index].buf = reinterpret_cast<__u8*>(
            const_cast<std::byte*>(operation.write_buffer().data()));
      }
      messages[index].len = static_cast<__u16>(operation.size());
    }

    ::i2c_rdwr_ioctl_data request{};
    request.msgs = messages.data();
    request.nmsgs = static_cast<__u32>(operations.size());
    const long completed = ::ioctl(fd_.get(), I2C_RDWR, &request);
    if (completed < 0) {
      return failure_result(errno, detail::operation::ioctl);
    }
    if (completed != static_cast<long>(operations.size())) {
      return failure_result(EIO, detail::operation::ioctl);
    }
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] static auto failure(int native_error, detail::operation operation)
      -> hal::result<Controller, error_type> {
    return hal::result<Controller, error_type>::failure(
        detail_i2c::failure(native_error, operation));
  }

  [[nodiscard]] static auto failure_result(int native_error,
                                            detail::operation operation)
      -> hal::result<void, error_type> {
    return hal::result<void, error_type>::failure(
        detail_i2c::failure(native_error, operation));
  }

  detail::owned_fd fd_{};
  unsigned long functions_{};
};

static_assert(hal::i2c::Controller7<Controller<>>);
static_assert(hal::i2c::Controller10<Controller<>>);

}  // namespace hal::linux::i2c
