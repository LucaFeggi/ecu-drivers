#include <cstddef>
#include <hal/nv.hpp>

namespace {

struct FlashError {
  hal::nv::flash_error_kind kind() const noexcept;
};

struct Flash {
  using error_type = FlashError;
  static hal::nv::flash_geometry geometry() noexcept;
  hal::result<void, error_type> read(std::size_t, hal::span<std::byte>);
  hal::result<void, error_type> program(std::size_t,
                                        hal::span<const std::byte>);
  hal::result<void, error_type> erase(std::size_t, std::size_t);
  hal::result<void, error_type> sync();
};

struct MemoryError {
  hal::nv::memory_error_kind kind() const noexcept;
};

struct Memory {
  using error_type = MemoryError;
  hal::nv::memory_properties properties() noexcept;
  hal::result<void, error_type> read(std::size_t, hal::span<std::byte>);
  hal::result<void, error_type> write(std::size_t, hal::span<const std::byte>);
  hal::result<void, error_type> sync();
};

static_assert(hal::nv::NorFlash<Flash>);
static_assert(hal::nv::Memory<Memory>);

}  // namespace

int main() {}
