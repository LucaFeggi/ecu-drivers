#pragma once

#include <cstdint>

#ifdef linux
#undef linux
#endif

namespace hal::linux::detail {

enum class operation : std::uint8_t {
  open,
  configure,
  read,
  write,
  ioctl,
  poll,
  synchronize,
  other
};

template <class Kind>
class error {
 public:
  constexpr error(Kind kind, int native_error = 0,
                  operation failed_operation = operation::other) noexcept
      : kind_{kind}, native_error_{native_error},
        operation_{failed_operation} {}

  [[nodiscard]] constexpr Kind kind() const noexcept { return kind_; }
  [[nodiscard]] constexpr int native_error() const noexcept {
    return native_error_;
  }
  [[nodiscard]] constexpr operation failed_operation() const noexcept {
    return operation_;
  }

 private:
  Kind kind_{};
  int native_error_{};
  operation operation_{};
};

}  // namespace hal::linux::detail
