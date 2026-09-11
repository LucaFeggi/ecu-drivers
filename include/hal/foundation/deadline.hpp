#pragma once

#include <cstdint>
#include <limits>
#include <hal/foundation/units.hpp>

namespace hal {

// An absolute monotonic deadline. Drivers that have a running monotonic clock
// can use this instead of a CPU-frequency-dependent iteration count. Early
// startup drivers may still use a documented bounded poll budget when no
// clock service is available yet.
class deadline {
 public:
  [[nodiscard]] static constexpr deadline at(instant value) noexcept {
    return deadline{value.nanoseconds_since_boot};
  }

  [[nodiscard]] static constexpr deadline after(instant now,
                                                nanoseconds duration) noexcept {
    constexpr std::uint64_t maximum =
        std::numeric_limits<std::uint64_t>::max();
    const std::uint64_t expires =
        duration.value > maximum - now.nanoseconds_since_boot
            ? maximum
            : now.nanoseconds_since_boot + duration.value;
    return deadline{expires};
  }

  [[nodiscard]] constexpr bool expired(instant now) const noexcept {
    return now.nanoseconds_since_boot >= expires_at_;
  }

  [[nodiscard]] constexpr nanoseconds remaining(instant now) const noexcept {
    return {expired(now) ? 0U : expires_at_ - now.nanoseconds_since_boot};
  }

  [[nodiscard]] constexpr instant expires_at() const noexcept {
    return {expires_at_};
  }

 private:
  explicit constexpr deadline(std::uint64_t value) noexcept
      : expires_at_{value} {}

  std::uint64_t expires_at_{};
};

}  // namespace hal
