#include <hal/contracts/gpio.hpp>

namespace {

struct Error {
  hal::gpio::error_kind kind() const noexcept;
};

struct Pin {
  using error_type = Error;
  hal::result<void, error_type> write(hal::gpio::level);
  hal::result<hal::gpio::level, error_type> read();
  hal::result<hal::gpio::level, error_type> output_latch();
  hal::result<void, error_type> configure_edge(hal::gpio::edge);
  bool take_event() noexcept;
};

static_assert(hal::gpio::OutputPin<Pin>);
static_assert(hal::gpio::InputPin<Pin>);
static_assert(hal::gpio::StatefulOutputPin<Pin>);
static_assert(hal::gpio::EdgeInput<Pin>);

}  // namespace

int main() {}
