#include <cstdint>
#include <hal/block.hpp>

namespace {

struct Error {
  hal::block::error_kind kind() const noexcept;
};

struct Device {
  using error_type = Error;
  hal::result<hal::block::geometry, error_type> geometry();
  hal::result<void, error_type> read_blocks(std::uint64_t,
                                            hal::span<std::byte>);
  hal::result<void, error_type> write_blocks(std::uint64_t,
                                             hal::span<const std::byte>);
  hal::result<void, error_type> sync();
  hal::result<void, error_type> trim(std::uint64_t, std::uint64_t);
};

static_assert(hal::block::Device<Device>);
static_assert(hal::block::TrimDevice<Device>);

}  // namespace

int main() {}
