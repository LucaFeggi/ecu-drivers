#pragma once

#include <cstdint>

namespace hal {

struct hertz {
  std::uint64_t value{};
};

struct nanoseconds {
  std::uint64_t value{};
};

struct microvolts {
  std::int64_t value{};
};

struct instant {
  std::uint64_t nanoseconds_since_boot{};
};

struct utc_time {
  std::int64_t seconds{};
  std::uint32_t nanoseconds{};
};

}  // namespace hal
