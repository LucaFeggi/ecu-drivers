#include <hal/adapters/gpio_active_low.hpp>

namespace {

int failures = 0;

void check(bool condition) {
  if (!condition) {
    ++failures;
  }
}

struct Error {
  [[nodiscard]] constexpr hal::gpio::error_kind kind() const noexcept {
    return hal::gpio::error_kind::io;
  }
};

struct Pin {
  using error_type = Error;

  hal::gpio::level current{hal::gpio::level::high};
  hal::gpio::edge selected_edge{hal::gpio::edge::both};
  bool pending{};

  [[nodiscard]] auto write(hal::gpio::level value)
      -> hal::result<void, error_type> {
    current = value;
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] auto read() -> hal::result<hal::gpio::level, error_type> {
    return hal::result<hal::gpio::level, error_type>::success(current);
  }

  [[nodiscard]] auto output_latch()
      -> hal::result<hal::gpio::level, error_type> {
    return hal::result<hal::gpio::level, error_type>::success(current);
  }

  [[nodiscard]] auto configure_edge(hal::gpio::edge value)
      -> hal::result<void, error_type> {
    selected_edge = value;
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] bool take_event() noexcept {
    const bool value = pending;
    pending = false;
    return value;
  }
};

void test_active_low_gpio() {
  Pin physical{};
  hal::gpio::ActiveLowPin output{physical};
  hal::gpio::ActiveLowPin input{physical};

  static_assert(hal::gpio::StatefulOutputPin<decltype(output)>);
  static_assert(hal::gpio::EdgeInput<decltype(input)>);

  auto write = output.write(hal::gpio::level::high);
  check(write.has_value());
  check(physical.current == hal::gpio::level::low);

  auto write_low = output.write(hal::gpio::level::low);
  check(write_low.has_value());
  check(physical.current == hal::gpio::level::high);
  auto latch = output.output_latch();
  check(latch.has_value());
  check(latch.value() == hal::gpio::level::low);

  auto read = input.read();
  check(read.has_value());
  check(read.value() == hal::gpio::level::low);
  physical.current = hal::gpio::level::low;
  check(input.read().value() == hal::gpio::level::high);

  auto configured = input.configure_edge(hal::gpio::edge::rising);
  check(configured.has_value());
  check(physical.selected_edge == hal::gpio::edge::falling);
  check(input.configure_edge(hal::gpio::edge::falling).has_value());
  check(physical.selected_edge == hal::gpio::edge::rising);
  check(input.configure_edge(hal::gpio::edge::both).has_value());
  check(physical.selected_edge == hal::gpio::edge::both);

  physical.pending = true;
  check(input.take_event());
  check(!input.take_event());
}

}  // namespace

int main() {
  test_active_low_gpio();
  return failures;
}
