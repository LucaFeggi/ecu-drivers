#pragma once

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <fcntl.h>
#include <hal/foundation/result.hpp>
#include <hal/foundation/span.hpp>
#include <hal/nv.hpp>
#include <limits>
#include <sys/stat.h>
#include <unistd.h>
#include <utility>

#include <hal/linux/detail/error.hpp>
#include <hal/linux/detail/fd.hpp>
#include <hal/linux/detail/io.hpp>

namespace hal::linux::nv {

using error_type = detail::error<hal::nv::flash_error_kind>;

struct config {
  const char* device{};
  bool create{};
};

namespace detail_nv {

[[nodiscard]] inline error_type failure(int native_error,
                                        hal::nv::flash_error_kind kind,
                                        detail::operation operation) noexcept {
  return error_type{kind, native_error, operation};
}

[[nodiscard]] inline error_type io_failure(int native_error,
                                           detail::operation operation) noexcept {
  return failure(native_error, hal::nv::flash_error_kind::io, operation);
}

}  // namespace detail_nv

template <std::size_t Capacity, std::size_t ProgramSize,
          std::size_t EraseSize, std::size_t ReadSize = 1U>
class FileNorFlash {
  static_assert(Capacity > 0U);
  static_assert(ProgramSize > 0U);
  static_assert(EraseSize > 0U);
  static_assert(ReadSize > 0U);
  static_assert((Capacity % EraseSize) == 0U);
  static_assert((EraseSize % ProgramSize) == 0U);
  static_assert((ProgramSize % ReadSize) == 0U);
  static_assert(Capacity <=
                static_cast<std::size_t>(std::numeric_limits<off_t>::max()));

 public:
  using error_type = linux::nv::error_type;

  [[nodiscard]] static constexpr hal::nv::flash_geometry geometry() noexcept {
    return {Capacity, ReadSize, ProgramSize, EraseSize};
  }

  [[nodiscard]] static auto open(config configuration)
      -> hal::result<FileNorFlash, error_type> {
    if (configuration.device == nullptr) {
      return failure(EINVAL, detail::operation::open);
    }
    int flags = O_RDWR | O_CLOEXEC;
    if (configuration.create) {
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
    if (!S_ISREG(metadata.st_mode)) {
      (void)::close(fd);
      return failure(EINVAL, detail::operation::configure);
    }
    if (metadata.st_size == 0 && configuration.create) {
      if (::ftruncate(fd, static_cast<off_t>(Capacity)) < 0) {
        const int native_error = errno;
        (void)::close(fd);
        return failure(native_error, detail::operation::configure);
      }
      std::array<std::byte, 4096U> erased{};
      erased.fill(std::byte{0xFF});
      for (std::size_t offset = 0U; offset < Capacity;) {
        const std::size_t count =
            (Capacity - offset) < erased.size() ? Capacity - offset : erased.size();
        if (!detail::pwrite_exact(fd, erased.data(), count, offset)) {
          const int native_error = errno == 0 ? EIO : errno;
          (void)::close(fd);
          return failure(native_error, detail::operation::write);
        }
        offset += count;
      }
    } else if (metadata.st_size != static_cast<off_t>(Capacity)) {
      (void)::close(fd);
      return failure(EINVAL, detail::operation::configure);
    }
    return hal::result<FileNorFlash, error_type>::success(
        FileNorFlash{detail::owned_fd{fd}});
  }

  FileNorFlash(const FileNorFlash&) = delete;
  FileNorFlash& operator=(const FileNorFlash&) = delete;
  FileNorFlash(FileNorFlash&&) noexcept = default;
  FileNorFlash& operator=(FileNorFlash&&) noexcept = default;

  [[nodiscard]] auto read(std::size_t offset, hal::span<std::byte> data)
      -> hal::result<void, error_type> {
    if (!data.valid()) {
      return failure_void(EINVAL, hal::nv::flash_error_kind::other,
                          detail::operation::read);
    }
    if (!in_bounds(offset, data.size())) {
      return failure_void(EINVAL, hal::nv::flash_error_kind::out_of_range,
                          detail::operation::read);
    }
    if (!aligned(offset, data.size(), ReadSize)) {
      return failure_void(EINVAL, hal::nv::flash_error_kind::not_aligned,
                          detail::operation::read);
    }
    if (!data.empty() && !detail::pread_exact(fd_.get(), data.data(), data.size(), offset)) {
      return failure_void(errno == 0 ? EIO : errno, hal::nv::flash_error_kind::io,
                          detail::operation::read);
    }
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] auto program(std::size_t offset,
                             hal::span<const std::byte> data)
      -> hal::result<void, error_type> {
    if (!data.valid()) {
      return failure_void(EINVAL, hal::nv::flash_error_kind::other,
                          detail::operation::write);
    }
    if (!in_bounds(offset, data.size())) {
      return failure_void(EINVAL, hal::nv::flash_error_kind::out_of_range,
                          detail::operation::write);
    }
    if (!aligned(offset, data.size(), ProgramSize)) {
      return failure_void(EINVAL, hal::nv::flash_error_kind::not_aligned,
                          detail::operation::write);
    }
    std::array<std::byte, ProgramSize> current{};
    for (std::size_t consumed = 0U; consumed < data.size();
         consumed += ProgramSize) {
      if (!detail::pread_exact(fd_.get(), current.data(), ProgramSize,
                               offset + consumed)) {
        return failure_void(errno == 0 ? EIO : errno, hal::nv::flash_error_kind::io,
                            detail::operation::read);
      }
      for (std::size_t index = 0U; index < ProgramSize; ++index) {
        if ((current[index] | data[consumed + index]) != current[index]) {
          return failure_void(EIO, hal::nv::flash_error_kind::program,
                              detail::operation::write);
        }
      }
      if (!detail::pwrite_exact(fd_.get(), data.data() + consumed, ProgramSize,
                                offset + consumed)) {
        return failure_void(errno == 0 ? EIO : errno, hal::nv::flash_error_kind::io,
                            detail::operation::write);
      }
    }
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] auto erase(std::size_t offset, std::size_t length)
      -> hal::result<void, error_type> {
    if (!in_bounds(offset, length)) {
      return failure_void(EINVAL, hal::nv::flash_error_kind::out_of_range,
                          detail::operation::write);
    }
    if (!aligned(offset, length, EraseSize)) {
      return failure_void(EINVAL, hal::nv::flash_error_kind::not_aligned,
                          detail::operation::write);
    }
    std::array<std::byte, 4096U> erased{};
    erased.fill(std::byte{0xFF});
    for (std::size_t consumed = 0U; consumed < length;) {
      const std::size_t count =
          (length - consumed) < erased.size() ? length - consumed : erased.size();
      if (!detail::pwrite_exact(fd_.get(), erased.data(), count,
                                offset + consumed)) {
        return failure_void(errno == 0 ? EIO : errno, hal::nv::flash_error_kind::erase,
                            detail::operation::write);
      }
      consumed += count;
    }
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] auto sync() -> hal::result<void, error_type> {
    if (::fdatasync(fd_.get()) < 0) {
      return failure_void(errno, hal::nv::flash_error_kind::io,
                          detail::operation::synchronize);
    }
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] int native_fd() const noexcept { return fd_.get(); }

 private:
  explicit FileNorFlash(detail::owned_fd fd) noexcept : fd_{std::move(fd)} {}

  [[nodiscard]] static bool in_bounds(std::size_t offset,
                                      std::size_t length) noexcept {
    return offset <= Capacity && length <= Capacity - offset;
  }

  [[nodiscard]] static bool aligned(std::size_t offset, std::size_t length,
                                    std::size_t alignment) noexcept {
    return (offset % alignment) == 0U && (length % alignment) == 0U;
  }

  [[nodiscard]] static auto failure(int native_error, detail::operation operation)
      -> hal::result<FileNorFlash, error_type> {
    return hal::result<FileNorFlash, error_type>::failure(
        detail_nv::io_failure(native_error, operation));
  }

  [[nodiscard]] static auto failure_void(int native_error,
                                         hal::nv::flash_error_kind kind,
                                         detail::operation operation)
      -> hal::result<void, error_type> {
    return hal::result<void, error_type>::failure(
        detail_nv::failure(native_error, kind, operation));
  }

  detail::owned_fd fd_{};
};

static_assert(hal::nv::NorFlash<FileNorFlash<4096U, 1U, 256U>>);

}  // namespace hal::linux::nv
