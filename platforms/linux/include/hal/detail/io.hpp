#pragma once

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <hal/foundation/span.hpp>
#include <sys/types.h>
#include <unistd.h>

#ifdef linux
#undef linux
#endif

namespace hal::linux::detail {

[[nodiscard]] inline bool valid_span(hal::span<const std::byte> data) noexcept {
  return data.valid();
}

[[nodiscard]] inline bool valid_span(hal::span<std::byte> data) noexcept {
  return data.valid();
}

[[nodiscard]] inline bool write_all(int fd, const std::byte* data,
                                    std::size_t size) noexcept {
  std::size_t offset = 0U;
  while (offset < size) {
    const ssize_t written = ::write(fd, data + offset, size - offset);
    if (written > 0) {
      offset += static_cast<std::size_t>(written);
      continue;
    }
    if (written == 0) {
      errno = EIO;
    }
    if (written < 0 && errno == EINTR) {
      continue;
    }
    return false;
  }
  return true;
}

[[nodiscard]] inline bool read_exact(int fd, std::byte* data,
                                     std::size_t size) noexcept {
  std::size_t offset = 0U;
  while (offset < size) {
    const ssize_t read_count = ::read(fd, data + offset, size - offset);
    if (read_count > 0) {
      offset += static_cast<std::size_t>(read_count);
      continue;
    }
    if (read_count == 0) {
      errno = EIO;
    }
    if (read_count < 0 && errno == EINTR) {
      continue;
    }
    return false;
  }
  return true;
}

[[nodiscard]] inline bool pread_exact(int fd, std::byte* data, std::size_t size,
                                      std::uint64_t offset) noexcept {
  std::size_t consumed = 0U;
  while (consumed < size) {
    const off_t position = static_cast<off_t>(offset + consumed);
    const ssize_t read_count = ::pread(fd, data + consumed, size - consumed,
                                       position);
    if (read_count > 0) {
      consumed += static_cast<std::size_t>(read_count);
      continue;
    }
    if (read_count == 0) {
      errno = EIO;
    }
    if (read_count < 0 && errno == EINTR) {
      continue;
    }
    return false;
  }
  return true;
}

[[nodiscard]] inline bool pwrite_exact(int fd, const std::byte* data,
                                       std::size_t size,
                                       std::uint64_t offset) noexcept {
  std::size_t consumed = 0U;
  while (consumed < size) {
    const off_t position = static_cast<off_t>(offset + consumed);
    const ssize_t written = ::pwrite(fd, data + consumed, size - consumed,
                                     position);
    if (written > 0) {
      consumed += static_cast<std::size_t>(written);
      continue;
    }
    if (written == 0) {
      errno = EIO;
    }
    if (written < 0 && errno == EINTR) {
      continue;
    }
    return false;
  }
  return true;
}

}  // namespace hal::linux::detail
