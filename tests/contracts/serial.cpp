#include <cstddef>
#include <hal/contracts/serial.hpp>

namespace {

struct Error {
  hal::serial::error_kind kind() const noexcept;
};

struct Port {
  using error_type = Error;
  hal::result<std::size_t, error_type> try_write(hal::span<const std::byte>);
  hal::result<void, error_type> flush();
  hal::result<std::size_t, error_type> try_read(hal::span<std::byte>);
  hal::result<hal::hertz, error_type> configure(hal::serial::config);
};

static_assert(hal::serial::Tx<Port>);
static_assert(hal::serial::Rx<Port>);
static_assert(hal::serial::Port<Port>);
static_assert(hal::serial::ConfigurablePort<Port>);

}  // namespace

int main() {}
