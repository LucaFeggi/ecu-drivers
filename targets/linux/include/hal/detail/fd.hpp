#pragma once

#include <unistd.h>

#ifdef linux
#undef linux
#endif

namespace hal::linux::detail {

class owned_fd {
 public:
  constexpr owned_fd() noexcept = default;
  explicit constexpr owned_fd(int fd) noexcept : fd_{fd} {}

  owned_fd(const owned_fd&) = delete;
  owned_fd& operator=(const owned_fd&) = delete;

  owned_fd(owned_fd&& other) noexcept : fd_{other.release()} {}

  owned_fd& operator=(owned_fd&& other) noexcept {
    if (this != &other) {
      reset(other.release());
    }
    return *this;
  }

  ~owned_fd() { reset(); }

  [[nodiscard]] constexpr bool valid() const noexcept { return fd_ >= 0; }
  [[nodiscard]] constexpr int get() const noexcept { return fd_; }

  [[nodiscard]] constexpr int release() noexcept {
    const int fd = fd_;
    fd_ = -1;
    return fd;
  }

  void reset(int fd = -1) noexcept {
    if (fd_ >= 0) {
      // Do not retry close() after EINTR: on Linux the descriptor may already
      // have been released and reused by another operation.
      (void)::close(fd_);
    }
    fd_ = fd;
  }

 private:
  int fd_{-1};
};

}  // namespace hal::linux::detail
