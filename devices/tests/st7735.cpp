#include <array>
#include <cstddef>
#include <cstdint>
#include <hal/devices/st7735.hpp>

namespace {

int failures = 0;

void check(bool condition) {
  if (!condition) {
    ++failures;
  }
}

struct SpiError {
  hal::spi::error_kind value{hal::spi::error_kind::io};

  [[nodiscard]] constexpr hal::spi::error_kind kind() const noexcept {
    return value;
  }
};

struct GpioError {
  [[nodiscard]] constexpr hal::gpio::error_kind kind() const noexcept {
    return hal::gpio::error_kind::io;
  }
};

struct Delay {
  std::array<std::uint64_t, 8U> durations{};
  std::size_t count{};

  void delay_for(hal::nanoseconds duration) noexcept {
    if (duration.value > 0U && count < durations.size()) {
      durations[count] = duration.value;
      ++count;
    }
  }
};

struct Pin {
  using error_type = GpioError;

  hal::gpio::level value{hal::gpio::level::high};
  std::array<hal::gpio::level, 512U> writes{};
  std::size_t write_count{};

  [[nodiscard]] auto write(hal::gpio::level next)
      -> hal::result<void, error_type> {
    value = next;
    if (write_count < writes.size()) {
      writes[write_count] = next;
      ++write_count;
    }
    return hal::result<void, error_type>::success();
  }
};

struct Bus {
  using error_type = SpiError;

  static constexpr bool supports_lsb_first{true};

  Pin* data_command{};
  Pin* chip_select{};
  std::array<std::byte, 4096U> bytes{};
  std::array<hal::gpio::level, 4096U> data_levels{};
  std::array<hal::gpio::level, 4096U> select_levels{};
  std::size_t count{};
  std::uint8_t current_command{};
  bool fail_writes{};

  [[nodiscard]] auto configure(hal::spi::config8 config)
      -> hal::result<hal::hertz, error_type> {
    return hal::result<hal::hertz, error_type>::success(config.max_frequency);
  }

  [[nodiscard]] auto write(hal::span<const std::byte> data)
      -> hal::result<void, error_type> {
    if (fail_writes) {
      return hal::result<void, error_type>::failure(
          {hal::spi::error_kind::io});
    }
    for (const std::byte value : data) {
      if (count < bytes.size()) {
        bytes[count] = value;
        data_levels[count] = data_command->value;
        select_levels[count] = chip_select->value;
        ++count;
      }
      if (data_command->value == hal::gpio::level::low) {
        current_command = static_cast<std::uint8_t>(value);
      }
    }
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] auto read(hal::span<std::byte> data, std::byte)
      -> hal::result<void, error_type> {
    const std::uint8_t response = current_command == 0xDAU
                                      ? 0x7CU
                                      : current_command == 0xDBU
                                            ? 0x89U
                                            : current_command == 0xDCU
                                                  ? 0xF0U
                                                  : 0x00U;
    for (std::byte& value : data) {
      value = static_cast<std::byte>(response);
    }
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] auto transfer(hal::span<const std::byte>,
                              hal::span<std::byte>)
      -> hal::result<void, error_type> {
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] auto flush() -> hal::result<void, error_type> {
    return hal::result<void, error_type>::success();
  }
};

using Display = hal::devices::st7735::St7735<Bus, Pin, Pin, Delay>;

Display make_display(Bus& bus, Pin& chip_select, Pin& data_command,
                     Delay& delay) {
  bus.data_command = &data_command;
  bus.chip_select = &chip_select;
  const hal::devices::st7735::configuration config{
      {10U, 8U, 1U, 2U, 0xA8U, 0x05U, true},
      {hal::hertz{4'000'000U}, hal::spi::mode::mode0,
       hal::spi::bit_order::msb_first, std::byte{0xFFU}},
      hal::nanoseconds{0U}, hal::nanoseconds{0U}};
  return Display{bus, chip_select, data_command, delay, config};
}

void test_profile() {
  constexpr auto profile = hal::devices::st7735::weact_mini_096_hannstar;
  check(profile.valid());
  check(profile.width == 160U);
  check(profile.height == 80U);
  check(profile.column_offset == 1U);
  check(profile.row_offset == 26U);
  check(profile.madctl == 0xA8U);
  check(profile.inversion_on);
}

void test_initialization_and_atomic_pixel_write() {
  Bus bus{};
  Pin chip_select{};
  Pin data_command{};
  Delay delay{};
  auto display = make_display(bus, chip_select, data_command, delay);

  check(display.initialize().has_value());
  check(display.initialized());
  check(display.width() == 10U);
  check(display.height() == 8U);
  check(delay.count == 3U);
  check(delay.durations[0U] == 120'000'000U);

  for (std::size_t index = 0U; index < bus.count; ++index) {
    check(bus.select_levels[index] == hal::gpio::level::low);
  }

  std::array<std::byte, 4U> pixels{
      std::byte{0xF8U}, std::byte{0x00U}, std::byte{0x07U}, std::byte{0xE0U}};
  const auto written = display.write_rect(2U, 3U, 2U, 1U, pixels);
  check(written.has_value());
  check(chip_select.value == hal::gpio::level::high);
  check(data_command.value == hal::gpio::level::high);
  check(bus.bytes[bus.count - 4U] == std::byte{0xF8U});
  check(bus.bytes[bus.count - 3U] == std::byte{0x00U});
  check(bus.bytes[bus.count - 2U] == std::byte{0x07U});
  check(bus.bytes[bus.count - 1U] == std::byte{0xE0U});

  check(display.set_address_window(0U, 0U, 2U, 2U).has_value());
  check(display.write_pixels(pixels).has_value());
  check(display.display_off().has_value());
  check(display.display_on().has_value());
  check(display.fill_rect(0U, 0U, 2U, 2U, 0xABCDU).has_value());
  check(display.draw_pixel(1U, 1U, 0x1234U).has_value());
}

void test_read_id_and_validation() {
  Bus bus{};
  Pin chip_select{};
  Pin data_command{};
  Delay delay{};
  auto display = make_display(bus, chip_select, data_command, delay);

  const auto before_init = display.read_id();
  check(!before_init.has_value());
  if (!before_init) {
    check(before_init.error().source() ==
          hal::devices::st7735::error_source::not_initialized);
  }

  check(display.initialize().has_value());
  const auto id = display.read_id();
  check(id.has_value());
  if (id) {
    check(id.value() == 0x7C89F0U);
  }
  check(!display.write_rect(9U, 7U, 2U, 1U, {}).has_value());
  check(!display.write_pixels({nullptr, 0U}).has_value());
}

void test_bus_failure_is_reported() {
  Bus bus{};
  Pin chip_select{};
  Pin data_command{};
  Delay delay{};
  auto display = make_display(bus, chip_select, data_command, delay);
  bus.fail_writes = true;
  const auto initialized = display.initialize();
  check(!initialized.has_value());
  if (!initialized) {
    check(initialized.error().source() ==
          hal::devices::st7735::error_source::bus);
    check(initialized.error().bus_kind() == hal::spi::error_kind::io);
  }
  check(chip_select.value == hal::gpio::level::high);
}

}  // namespace

int main() {
  test_profile();
  test_initialization_and_atomic_pixel_write();
  test_read_id_and_validation();
  test_bus_failure_is_reported();
  return failures;
}
