#include <array>
#include <cstddef>
#include <hal/spi/static_device.hpp>

namespace {

int failures = 0;

void check(bool condition) {
  if (!condition) {
    ++failures;
  }
}

struct Trace {
  std::array<char, 32U> events{};
  std::size_t size{};

  void add(char event) noexcept {
    if (size < events.size()) {
      events[size] = event;
      ++size;
    }
  }
};

struct GpioError {
  [[nodiscard]] constexpr hal::gpio::error_kind kind() const noexcept {
    return hal::gpio::error_kind::io;
  }
};

struct SpiError {
  hal::spi::error_kind value{hal::spi::error_kind::io};
  [[nodiscard]] constexpr hal::spi::error_kind kind() const noexcept {
    return value;
  }
};

struct Bus {
  using error_type = SpiError;
  static constexpr bool supports_lsb_first = false;

  Trace* trace{};
  bool fail_write{};
  bool fail_flush{};

  [[nodiscard]] auto configure(hal::spi::config8 config)
      -> hal::result<hal::hertz, error_type> {
    trace->add('C');
    return hal::result<hal::hertz, error_type>::success(config.max_frequency);
  }

  [[nodiscard]] auto write(hal::span<const std::byte>)
      -> hal::result<void, error_type> {
    trace->add('W');
    if (fail_write) {
      return hal::result<void, error_type>::failure({hal::spi::error_kind::io});
    }
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] auto read(hal::span<std::byte> data, std::byte fill)
      -> hal::result<void, error_type> {
    trace->add('R');
    for (std::byte& value : data) {
      value = fill;
    }
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] auto transfer(hal::span<const std::byte>, hal::span<std::byte>)
      -> hal::result<void, error_type> {
    trace->add('T');
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] auto flush() -> hal::result<void, error_type> {
    trace->add('F');
    if (fail_flush) {
      return hal::result<void, error_type>::failure(
          {hal::spi::error_kind::timeout});
    }
    return hal::result<void, error_type>::success();
  }
};

struct ChipSelect {
  using error_type = GpioError;

  Trace* trace{};

  [[nodiscard]] auto write(hal::gpio::level value)
      -> hal::result<void, error_type> {
    trace->add(value == hal::gpio::level::low ? 'A' : 'D');
    return hal::result<void, error_type>::success();
  }
};

struct Delay {
  Trace* trace{};

  void delay_for(hal::nanoseconds duration) noexcept {
    if (duration.value == 10U) {
      trace->add('s');
    } else if (duration.value == 20U) {
      trace->add('x');
    } else if (duration.value == 30U) {
      trace->add('h');
    }
  }
};

struct Lock {
  Trace* trace{};
  void lock() noexcept { trace->add('L'); }
  void unlock() noexcept { trace->add('U'); }
};

void test_static_spi_device() {
  Trace trace{};
  Bus bus{&trace};
  ChipSelect select{&trace};
  Delay delay{&trace};
  Lock lock{&trace};
  const hal::spi::device_config8 config{
      {hal::hertz{1'000'000U}, hal::spi::mode::mode0,
       hal::spi::bit_order::msb_first, std::byte{0xA5}},
      hal::nanoseconds{10U},
      hal::nanoseconds{30U}};
  hal::spi::StaticDevice8 device{bus, select, delay, lock, config};
  static_assert(hal::spi::Device8<decltype(device)>);

  const std::array<std::byte, 1U> tx{std::byte{0x55}};
  std::array<std::byte, 1U> rx{};
  const hal::spi::operation8 operations[] = {
      hal::spi::operation8::write(tx),
      hal::spi::operation8::delay_for(hal::nanoseconds{20U}),
      hal::spi::operation8::read(rx)};

  auto transacted = device.transaction(operations);
  check(transacted.has_value());
  check(device.actual_frequency().value == 1'000'000U);
  check(rx[0U] == std::byte{0xA5});

  constexpr char expected[] = "LCAsWFxRFhDU";
  check(trace.size == sizeof(expected) - 1U);
  for (std::size_t index = 0U; index < trace.size; ++index) {
    check(trace.events[index] == expected[index]);
  }
}

void test_static_spi_device_releases_chip_select_after_bus_failure() {
  Trace trace{};
  Bus bus{&trace, true};
  ChipSelect select{&trace};
  Delay delay{&trace};
  Lock lock{&trace};
  const hal::spi::device_config8 config{{hal::hertz{1'000'000U},
                                         hal::spi::mode::mode0,
                                         hal::spi::bit_order::msb_first,
                                         std::byte{0xFF}},
                                        hal::nanoseconds{10U},
                                        hal::nanoseconds{30U}};
  hal::spi::StaticDevice8 device{bus, select, delay, lock, config};
  const std::array<std::byte, 1U> tx{std::byte{0x55}};
  const hal::spi::operation8 operations[] = {hal::spi::operation8::write(tx)};

  const auto transacted = device.transaction(operations);
  check(!transacted.has_value());
  if (!transacted) {
    check(transacted.error().source() == hal::spi::static_device_error_source::bus);
    check(transacted.error().kind() == hal::spi::error_kind::io);
  }

  constexpr char expected[] = "LCAsWFhDU";
  check(trace.size == sizeof(expected) - 1U);
  for (std::size_t index = 0U; index < trace.size; ++index) {
    check(trace.events[index] == expected[index]);
  }
}

void test_static_spi_device_rejects_unsupported_bit_order() {
  Trace trace{};
  Bus bus{&trace};
  ChipSelect select{&trace};
  Delay delay{&trace};
  Lock lock{&trace};
  const hal::spi::device_config8 config{{hal::hertz{1'000'000U},
                                         hal::spi::mode::mode0,
                                         hal::spi::bit_order::lsb_first,
                                         std::byte{0xFF}},
                                        hal::nanoseconds{},
                                        hal::nanoseconds{}};
  hal::spi::StaticDevice8 device{bus, select, delay, lock, config};
  const std::array<std::byte, 1U> tx{std::byte{0x55}};
  const hal::spi::operation8 operations[] = {hal::spi::operation8::write(tx)};

  const auto transacted = device.transaction(operations);
  check(!transacted.has_value());
  if (!transacted) {
    check(transacted.error().source() ==
          hal::spi::static_device_error_source::unsupported_bit_order);
  }
  check(trace.size == 0U);
}

}  // namespace

int main() {
  test_static_spi_device();
  test_static_spi_device_releases_chip_select_after_bus_failure();
  test_static_spi_device_rejects_unsupported_bit_order();
  return failures;
}
