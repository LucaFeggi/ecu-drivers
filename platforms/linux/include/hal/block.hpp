#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/block.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <fcntl.h>
#include <hal/foundation/result.hpp>
#include <hal/foundation/span.hpp>
#include <limits>
#include <linux/fs.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <utility>

#include <hal/detail/error.hpp>
#include <hal/detail/fd.hpp>
#include <hal/detail/io.hpp>

namespace hal::linux::block {

using error_type = detail::error<hal::block::error_kind>;

struct config {
  const char* device{};
  std::uint32_t file_block_size{512U};
  std::uint64_t create_size_bytes{};
  bool read_only{};
};

namespace detail_block {

[[nodiscard]] inline error_type failure(int native_error,
                                        detail::operation operation) noexcept {
  hal::block::error_kind kind = hal::block::error_kind::io;
  if (native_error == ENOENT || native_error == ENODEV || native_error == ENXIO) {
    kind = hal::block::error_kind::no_media;
  } else if (native_error == EROFS || native_error == EACCES) {
    kind = hal::block::error_kind::write_protected;
  } else if (native_error == EINVAL) {
    kind = hal::block::error_kind::not_aligned;
  } else if (native_error == ETIMEDOUT) {
    kind = hal::block::error_kind::timeout;
  }
  return error_type{kind, native_error, operation};
}

}  // namespace detail_block

class Device {
 public:
  using error_type = linux::block::error_type;

  [[nodiscard]] static auto open(config configuration)
      -> hal::result<Device, error_type> {
    if (configuration.device == nullptr || configuration.file_block_size == 0U) {
      return failure(EINVAL, detail::operation::open);
    }
    if (configuration.create_size_bytes != 0U &&
        configuration.create_size_bytes >
            static_cast<std::uint64_t>(std::numeric_limits<off_t>::max())) {
      return failure(EOVERFLOW, detail::operation::configure);
    }
    int flags = configuration.read_only ? O_RDONLY : O_RDWR;
    flags |= O_CLOEXEC;
    if (configuration.create_size_bytes != 0U) {
      flags |= O_CREAT;
    }
    const int fd = ::open(configuration.device, flags, 0660);
    if (fd < 0) {
      return failure(errno, detail::operation::open);
    }

    struct stat metadata{};
    if (::fstat(fd, &metadata) < 0) {
      const int native_error = errno;
      (void)::close(fd);
      return failure(native_error, detail::operation::open);
    }

    const bool block_device = S_ISBLK(metadata.st_mode);
    if (!block_device && !S_ISREG(metadata.st_mode)) {
      (void)::close(fd);
      return failure(EINVAL, detail::operation::configure);
    }
    std::uint64_t size_bytes = 0U;
    std::uint32_t block_size = configuration.file_block_size;
    if (block_device) {
      unsigned int logical_size{};
      if (::ioctl(fd, BLKSSZGET, &logical_size) < 0 || logical_size == 0U) {
        const int native_error = errno == 0 ? EIO : errno;
        (void)::close(fd);
        return failure(native_error, detail::operation::ioctl);
      }
      std::uint64_t device_size{};
      if (::ioctl(fd, BLKGETSIZE64, &device_size) < 0) {
        const int native_error = errno;
        (void)::close(fd);
        return failure(native_error, detail::operation::ioctl);
      }
      size_bytes = device_size;
      block_size = static_cast<std::uint32_t>(logical_size);
    } else {
      if (configuration.create_size_bytes != 0U &&
          static_cast<std::uint64_t>(metadata.st_size) !=
              configuration.create_size_bytes) {
        if (metadata.st_size != 0 || configuration.read_only ||
            ::ftruncate(fd, static_cast<off_t>(configuration.create_size_bytes)) <
                0) {
          const int native_error = errno == 0 ? EINVAL : errno;
          (void)::close(fd);
          return failure(native_error, detail::operation::configure);
        }
        size_bytes = configuration.create_size_bytes;
      } else {
        if (metadata.st_size < 0) {
          (void)::close(fd);
          return failure(EIO, detail::operation::open);
        }
        size_bytes = static_cast<std::uint64_t>(metadata.st_size);
      }
    }
    if (block_size == 0U || size_bytes == 0U ||
        (size_bytes % static_cast<std::uint64_t>(block_size)) != 0U) {
      (void)::close(fd);
      return failure(EINVAL, detail::operation::configure);
    }
    const std::uint64_t count =
        size_bytes / static_cast<std::uint64_t>(block_size);
    return hal::result<Device, error_type>::success(Device{
        detail::owned_fd{fd},
        hal::block::geometry{block_size, count, configuration.read_only},
        block_device});
  }

  Device(const Device&) = delete;
  Device& operator=(const Device&) = delete;
  Device(Device&&) noexcept = default;
  Device& operator=(Device&&) noexcept = default;

  [[nodiscard]] auto geometry()
      -> hal::result<hal::block::geometry, error_type> {
    if (!fd_.valid()) {
      return hal::result<hal::block::geometry, error_type>::failure(
          detail_block::failure(EBADF, detail::operation::other));
    }
    return hal::result<hal::block::geometry, error_type>::success(geometry_);
  }

  [[nodiscard]] auto read_blocks(std::uint64_t lba, hal::span<std::byte> data)
      -> hal::result<void, error_type> {
    if (!data.valid() || !aligned_range(data.size())) {
      return failure_void_kind(hal::block::error_kind::not_aligned, EINVAL,
                               detail::operation::read);
    }
    if (!in_bounds(lba, data.size())) {
      return failure_void_kind(hal::block::error_kind::out_of_range, EINVAL,
                               detail::operation::read);
    }
    if (data.empty()) {
      return hal::result<void, error_type>::success();
    }
    const std::uint64_t offset = lba * geometry_.logical_block_size;
    if (!detail::pread_exact(fd_.get(), data.data(), data.size(), offset)) {
      return failure_void(errno == 0 ? EIO : errno, detail::operation::read);
    }
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] auto write_blocks(std::uint64_t lba,
                                  hal::span<const std::byte> data)
      -> hal::result<void, error_type> {
    if (geometry_.read_only) {
      return failure_void(EROFS, detail::operation::write);
    }
    if (!data.valid() || !aligned_range(data.size())) {
      return failure_void_kind(hal::block::error_kind::not_aligned, EINVAL,
                               detail::operation::write);
    }
    if (!in_bounds(lba, data.size())) {
      return failure_void_kind(hal::block::error_kind::out_of_range, EINVAL,
                               detail::operation::write);
    }
    if (data.empty()) {
      return hal::result<void, error_type>::success();
    }
    const std::uint64_t offset = lba * geometry_.logical_block_size;
    if (!detail::pwrite_exact(fd_.get(), data.data(), data.size(), offset)) {
      return failure_void(errno == 0 ? EIO : errno, detail::operation::write);
    }
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] auto sync() -> hal::result<void, error_type> {
    if (::fdatasync(fd_.get()) < 0) {
      return failure_void(errno, detail::operation::synchronize);
    }
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] auto trim(std::uint64_t first_lba, std::uint64_t count)
      -> hal::result<void, error_type> {
    if (!block_device_ || count == 0U || first_lba >= geometry_.logical_block_count ||
        count > geometry_.logical_block_count - first_lba) {
      return failure_void(EINVAL, detail::operation::configure);
    }
    const std::uint64_t range[2] = {
        first_lba * geometry_.logical_block_size,
        count * geometry_.logical_block_size};
    if (::ioctl(fd_.get(), BLKDISCARD, &range) < 0) {
      return failure_void(errno, detail::operation::ioctl);
    }
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] int native_fd() const noexcept { return fd_.get(); }

 private:
  Device(detail::owned_fd fd, hal::block::geometry geometry,
         bool block_device) noexcept
      : fd_{std::move(fd)}, geometry_{geometry}, block_device_{block_device} {}

  [[nodiscard]] bool aligned_range(std::size_t bytes) const noexcept {
    return (bytes % geometry_.logical_block_size) == 0U;
  }

  [[nodiscard]] bool in_bounds(std::uint64_t lba,
                               std::size_t bytes) const noexcept {
    const std::uint64_t count =
        static_cast<std::uint64_t>(bytes / geometry_.logical_block_size);
    return lba <= geometry_.logical_block_count &&
           count <= geometry_.logical_block_count - lba;
  }

  [[nodiscard]] static auto failure(int native_error, detail::operation operation)
      -> hal::result<Device, error_type> {
    return hal::result<Device, error_type>::failure(
        detail_block::failure(native_error, operation));
  }

  [[nodiscard]] static auto failure_void(int native_error,
                                         detail::operation operation)
      -> hal::result<void, error_type> {
    return hal::result<void, error_type>::failure(
        detail_block::failure(native_error, operation));
  }

  [[nodiscard]] static auto failure_void_kind(hal::block::error_kind kind,
                                              int native_error,
                                              detail::operation operation)
      -> hal::result<void, error_type> {
    return hal::result<void, error_type>::failure(
        error_type{kind, native_error, operation});
  }

  detail::owned_fd fd_{};
  hal::block::geometry geometry_{};
  bool block_device_{};
};

static_assert(hal::block::Device<Device>);
static_assert(hal::block::TrimDevice<Device>);

}  // namespace hal::linux::block
