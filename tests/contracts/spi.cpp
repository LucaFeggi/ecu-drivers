#include <hal/contracts/spi_bus.hpp>

namespace {

struct Error {
  hal::spi::error_kind kind() const noexcept;
};

struct Bus {
  using error_type = Error;
  static constexpr bool supports_lsb_first = true;
  hal::result<hal::hertz, error_type> configure(hal::spi::config8);
  hal::result<void, error_type> write(hal::span<const std::byte>);
  hal::result<void, error_type> read(hal::span<std::byte>, std::byte);
  hal::result<void, error_type> transfer(hal::span<const std::byte>,
                                         hal::span<std::byte>);
  hal::result<void, error_type> flush();
};

struct Device {
  using error_type = Error;
  hal::result<void, error_type> transaction(
      hal::span<const hal::spi::operation8>);
};

static_assert(hal::spi::Bus8<Bus>);
static_assert(hal::spi::Device8<Device>);
static_assert(hal::spi::LsbFirstBus8<Bus>);

}  // namespace

int main() {}
