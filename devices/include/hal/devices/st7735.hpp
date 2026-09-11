#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <hal/contracts/gpio.hpp>
#include <hal/foundation/result.hpp>
#include <hal/foundation/span.hpp>
#include <hal/contracts/spi_bus.hpp>
#include <hal/adapters/spi_static_device.hpp>
#include <hal/contracts/time.hpp>
#include <limits>
#include <utility>

namespace hal::devices::st7735 {

enum class orientation : std::uint8_t {
  portrait,
  portrait_rot180,
  landscape,
  landscape_rot180
};

enum class error_source : std::uint8_t {
  bus,
  chip_select,
  data_command,
  invalid_argument,
  not_initialized
};

class error {
 public:
  [[nodiscard]] static constexpr error bus(hal::spi::error_kind kind) noexcept {
    return {error_source::bus, kind, hal::gpio::error_kind::other};
  }

  [[nodiscard]] static constexpr error chip_select(
      hal::gpio::error_kind kind) noexcept {
    return {error_source::chip_select, hal::spi::error_kind::io, kind};
  }

  [[nodiscard]] static constexpr error data_command(
      hal::gpio::error_kind kind) noexcept {
    return {error_source::data_command, hal::spi::error_kind::io, kind};
  }

  [[nodiscard]] static constexpr error invalid_argument() noexcept {
    return {error_source::invalid_argument, hal::spi::error_kind::configuration,
            hal::gpio::error_kind::other};
  }

  [[nodiscard]] static constexpr error not_initialized() noexcept {
    return {error_source::not_initialized, hal::spi::error_kind::configuration,
            hal::gpio::error_kind::other};
  }

  [[nodiscard]] constexpr error_source source() const noexcept {
    return source_;
  }

  [[nodiscard]] constexpr hal::spi::error_kind bus_kind() const noexcept {
    return bus_kind_;
  }

  [[nodiscard]] constexpr hal::gpio::error_kind gpio_kind() const noexcept {
    return gpio_kind_;
  }

 private:
  constexpr error(error_source source, hal::spi::error_kind bus_kind,
                  hal::gpio::error_kind gpio_kind) noexcept
      : source_{source}, bus_kind_{bus_kind}, gpio_kind_{gpio_kind} {}

  error_source source_{};
  hal::spi::error_kind bus_kind_{};
  hal::gpio::error_kind gpio_kind_{};
};

struct panel_profile {
  // Width and height describe the logical drawing surface after orientation.
  std::uint16_t width{};
  std::uint16_t height{};
  // The controller's RAM is usually larger than the visible panel. These are
  // the first column and row addresses for logical coordinate (0, 0).
  std::uint16_t column_offset{};
  std::uint16_t row_offset{};
  // MADCTL value, including the panel's RGB/BGR selection.
  std::uint8_t madctl{};
  // COLMOD value. 0x05 is 16-bit RGB565.
  std::uint8_t color_mode{0x05U};
  bool inversion_on{};

  [[nodiscard]] constexpr bool valid() const noexcept {
    const std::uint32_t last_column =
        static_cast<std::uint32_t>(column_offset) + width;
    const std::uint32_t last_row = static_cast<std::uint32_t>(row_offset) + height;
    return width > 0U && height > 0U && last_column <= 0x1'0000U &&
           last_row <= 0x1'0000U &&
           color_mode == 0x05U;
  }
};

// WeAct MiniSTM32H723 V1.2's supplied 0.96-inch HannStar panel. The vendor
// example selects landscape-rotated-180, BGR, and applies (1, 26) offsets.
inline constexpr panel_profile weact_mini_096_hannstar{
    160U, 80U, 1U, 26U, 0xA8U, 0x05U, true};

struct configuration {
  panel_profile panel{weact_mini_096_hannstar};
  hal::spi::config8 bus{hal::hertz{12'000'000U}, hal::spi::mode::mode0,
                        hal::spi::bit_order::msb_first, std::byte{0xFFU}};
  hal::nanoseconds chip_select_setup{};
  hal::nanoseconds chip_select_hold{};

  [[nodiscard]] constexpr bool valid() const noexcept {
    return panel.valid() && bus.max_frequency.value > 0U;
  }
};

inline constexpr configuration weact_mini_096_configuration{};

// The ST7735 interface has a D/C line in addition to SPI chip select. This
// driver therefore consumes a Bus8 and both control pins directly, keeping
// command, parameter, and pixel phases in one bounded transaction.
template <hal::spi::Bus8 Bus, hal::gpio::OutputPin ChipSelect,
          hal::gpio::OutputPin DataCommand, hal::time::Delay Delay,
          hal::spi::BasicLock Lock = hal::spi::NoLock>
class St7735 {
 public:
  using error_type = error;

  St7735(Bus& bus, ChipSelect& chip_select, DataCommand& data_command,
         Delay& delay, configuration config, Lock& lock) noexcept
      : bus_{bus}, chip_select_{chip_select}, data_command_{data_command},
        delay_{delay}, lock_{&lock}, configuration_{config},
        panel_{config.panel} {}

  St7735(Bus& bus, ChipSelect& chip_select, DataCommand& data_command,
         Delay& delay, configuration config) noexcept
    requires std::same_as<Lock, hal::spi::NoLock>
      : bus_{bus}, chip_select_{chip_select}, data_command_{data_command},
        delay_{delay}, lock_{&default_lock_}, configuration_{config},
        panel_{config.panel} {}

  St7735(const St7735&) = delete;
  St7735& operator=(const St7735&) = delete;

  [[nodiscard]] auto initialize() -> result<void, error_type> {
    if (!configuration_.valid()) {
      return failure(error_type::invalid_argument());
    }

    // The WeAct module's reset input is tied to the board reset net. Keep the
    // software reset sequence here so the driver also works with modules that
    // do not expose a separately controlled reset pin.
    if (auto result = send_command(command::software_reset, {}); !result) {
      return result;
    }
    delay_.delay_for(hal::nanoseconds{120'000'000U});
    if (auto result = send_command(command::software_reset, {}); !result) {
      return result;
    }
    delay_.delay_for(hal::nanoseconds{120'000'000U});

    if (auto result = send_command(command::sleep_out, {}); !result) {
      return result;
    }
    delay_.delay_for(hal::nanoseconds{120'000'000U});

    constexpr std::array<std::byte, 3U> frame_rate_normal{
        std::byte{0x01U}, std::byte{0x2CU}, std::byte{0x2DU}};
    constexpr std::array<std::byte, 3U> frame_rate_idle{
        std::byte{0x01U}, std::byte{0x2CU}, std::byte{0x2DU}};
    constexpr std::array<std::byte, 6U> frame_rate_partial{
        std::byte{0x01U}, std::byte{0x2CU}, std::byte{0x2DU},
        std::byte{0x01U}, std::byte{0x2CU}, std::byte{0x2DU}};
    constexpr std::array<std::byte, 3U> power_control_1{
        std::byte{0xA2U}, std::byte{0x02U}, std::byte{0x84U}};
    constexpr std::array<std::byte, 1U> power_control_2{std::byte{0xC5U}};
    constexpr std::array<std::byte, 2U> power_control_3{
        std::byte{0x0AU}, std::byte{0x00U}};
    constexpr std::array<std::byte, 2U> power_control_4{
        std::byte{0x8AU}, std::byte{0x2AU}};
    constexpr std::array<std::byte, 2U> power_control_5{
        std::byte{0x8AU}, std::byte{0xEEU}};
    constexpr std::array<std::byte, 1U> vcom_control{std::byte{0x0EU}};
    constexpr std::array<std::byte, 1U> inversion_value{std::byte{0x07U}};
    constexpr std::array<std::byte, 16U> positive_gamma{
        std::byte{0x02U}, std::byte{0x1CU}, std::byte{0x07U}, std::byte{0x12U},
        std::byte{0x37U}, std::byte{0x32U}, std::byte{0x29U}, std::byte{0x2DU},
        std::byte{0x29U}, std::byte{0x25U}, std::byte{0x2BU}, std::byte{0x39U},
        std::byte{0x00U}, std::byte{0x01U}, std::byte{0x03U}, std::byte{0x10U}};
    constexpr std::array<std::byte, 16U> negative_gamma{
        std::byte{0x03U}, std::byte{0x1DU}, std::byte{0x07U}, std::byte{0x06U},
        std::byte{0x2EU}, std::byte{0x2CU}, std::byte{0x29U}, std::byte{0x2DU},
        std::byte{0x2EU}, std::byte{0x2EU}, std::byte{0x37U}, std::byte{0x3FU},
        std::byte{0x00U}, std::byte{0x00U}, std::byte{0x02U}, std::byte{0x10U}};
    const std::array<std::byte, 1U> color_mode{static_cast<std::byte>(
        panel_.color_mode)};

    if (auto result = send_command(command::frame_rate_control_1, frame_rate_normal);
        !result) {
      return result;
    }
    if (auto result = send_command(command::frame_rate_control_2, frame_rate_idle);
        !result) {
      return result;
    }
    if (auto result = send_command(command::frame_rate_control_3, frame_rate_partial);
        !result) {
      return result;
    }
    if (auto result = send_command(command::frame_inversion_control,
                                   inversion_value);
        !result) {
      return result;
    }
    if (auto result = send_command(command::power_control_1, power_control_1);
        !result) {
      return result;
    }
    if (auto result = send_command(command::power_control_2, power_control_2);
        !result) {
      return result;
    }
    if (auto result = send_command(command::power_control_3, power_control_3);
        !result) {
      return result;
    }
    if (auto result = send_command(command::power_control_4, power_control_4);
        !result) {
      return result;
    }
    if (auto result = send_command(command::power_control_5, power_control_5);
        !result) {
      return result;
    }
    if (auto result = send_command(command::vcom_control, vcom_control); !result) {
      return result;
    }
    if (auto result = send_command(
            panel_.inversion_on ? command::display_inversion_on
                                : command::display_inversion_off,
            {});
        !result) {
      return result;
    }
    if (auto result = send_command(command::color_mode, color_mode); !result) {
      return result;
    }
    if (auto result = send_command(command::positive_gamma, positive_gamma); !result) {
      return result;
    }
    if (auto result = send_command(command::negative_gamma, negative_gamma); !result) {
      return result;
    }
    if (auto result = send_command(command::normal_display_on, {}); !result) {
      return result;
    }
    if (auto result = send_command(command::display_on, {}); !result) {
      return result;
    }
    if (auto result = set_window_and_madctl(); !result) {
      return result;
    }

    initialized_ = true;
    return result<void, error_type>::success();
  }

  [[nodiscard]] bool initialized() const noexcept { return initialized_; }

  [[nodiscard]] constexpr std::uint16_t width() const noexcept {
    return panel_.width;
  }

  [[nodiscard]] constexpr std::uint16_t height() const noexcept {
    return panel_.height;
  }

  [[nodiscard]] constexpr hal::spi::config8 bus_configuration() const noexcept {
    return configuration_.bus;
  }

  [[nodiscard]] constexpr panel_profile profile() const noexcept {
    return panel_;
  }

  [[nodiscard]] auto read_id() -> result<std::uint32_t, error_type> {
    if (!initialized_) {
      return result<std::uint32_t, error_type>::failure(
          error_type::not_initialized());
    }

    std::array<std::byte, 3U> response{};
    const auto transacted = transaction([&]() -> result<void, error_type> {
      if (auto result = read_register(command::read_id_1, response[0U]);
          !result) {
        return result;
      }
      if (auto result = read_register(command::read_id_2, response[1U]);
          !result) {
        return result;
      }
      return read_register(command::read_id_3, response[2U]);
    });
    if (!transacted) {
      return result<std::uint32_t, error_type>::failure(transacted.error());
    }

    const auto first = static_cast<std::uint32_t>(
        static_cast<std::uint8_t>(response[0U]));
    const auto second = static_cast<std::uint32_t>(
        static_cast<std::uint8_t>(response[1U]));
    const auto third = static_cast<std::uint32_t>(
        static_cast<std::uint8_t>(response[2U]));
    return result<std::uint32_t, error_type>::success((first << 16U) |
                                                      (second << 8U) | third);
  }

  [[nodiscard]] auto display_on() -> result<void, error_type> {
    if (!initialized_) {
      return failure(error_type::not_initialized());
    }
    if (auto result = send_command(command::normal_display_on, {}); !result) {
      return result;
    }
    if (auto result = send_command(command::display_on, {}); !result) {
      return result;
    }
    const std::array<std::byte, 1U> madctl{
        static_cast<std::byte>(panel_.madctl)};
    return send_command(command::memory_data_access_control, madctl);
  }

  [[nodiscard]] auto display_off() -> result<void, error_type> {
    if (!initialized_) {
      return failure(error_type::not_initialized());
    }
    return send_command(command::display_off, {});
  }

  [[nodiscard]] auto set_address_window(std::uint16_t x, std::uint16_t y,
                                         std::uint16_t width,
                                         std::uint16_t height)
      -> result<void, error_type> {
    if (!initialized_) {
      return failure(error_type::not_initialized());
    }
    if (!valid_window(x, y, width, height)) {
      return failure(error_type::invalid_argument());
    }
    return transaction([&]() -> result<void, error_type> {
      return write_window_active(x, y, width, height);
    });
  }

  // Writes pixel bytes after a previous set_address_window() call. Pixel data
  // is expected in controller order: big-endian RGB565 for color_mode 0x05.
  [[nodiscard]] auto write_pixels(hal::span<const std::byte> pixels)
      -> result<void, error_type> {
    if (!initialized_) {
      return failure(error_type::not_initialized());
    }
    if (!pixels.valid() || pixels.empty() || (pixels.size() % 2U) != 0U) {
      return failure(error_type::invalid_argument());
    }
    return transaction([&]() -> result<void, error_type> {
      if (auto result = write_command_active(command::memory_write, {});
          !result) {
        return result;
      }
      return write_data_active(pixels);
    });
  }

  // Sets the address window and streams the complete pixel payload while CS
  // remains asserted. This is the preferred API for DMA-backed SPI buses.
  [[nodiscard]] auto write_rect(std::uint16_t x, std::uint16_t y,
                                std::uint16_t width, std::uint16_t height,
                                hal::span<const std::byte> pixels)
      -> result<void, error_type> {
    if (!initialized_) {
      return failure(error_type::not_initialized());
    }
    if (!valid_window(x, y, width, height) || !pixels.valid()) {
      return failure(error_type::invalid_argument());
    }
    const std::uint64_t expected = static_cast<std::uint64_t>(width) * height * 2U;
    if (expected > std::numeric_limits<std::size_t>::max() ||
        pixels.size() != static_cast<std::size_t>(expected)) {
      return failure(error_type::invalid_argument());
    }
    return transaction([&]() -> result<void, error_type> {
      if (auto result = write_window_active(x, y, width, height); !result) {
        return result;
      }
      if (auto result = write_command_active(command::memory_write, {});
          !result) {
        return result;
      }
      return write_data_active(pixels);
    });
  }

  [[nodiscard]] auto fill_rect(std::uint16_t x, std::uint16_t y,
                               std::uint16_t width, std::uint16_t height,
                               std::uint16_t color)
      -> result<void, error_type> {
    if (!initialized_) {
      return failure(error_type::not_initialized());
    }
    if (!valid_window(x, y, width, height)) {
      return failure(error_type::invalid_argument());
    }

    // Keep the bounded row buffer small enough for stack-based bare-metal
    // callers. Wider rectangles are split into adjacent windows.
    constexpr std::uint16_t row_capacity_pixels{256U};
    std::array<std::byte, row_capacity_pixels * 2U> row{};
    const std::byte high = static_cast<std::byte>(color >> 8U);
    const std::byte low = static_cast<std::byte>(color & 0xFFU);
    std::uint16_t x_offset = 0U;
    while (x_offset < width) {
      const std::uint16_t remaining = static_cast<std::uint16_t>(width - x_offset);
      const std::uint16_t chunk = remaining < row_capacity_pixels
                                      ? remaining
                                      : row_capacity_pixels;
      for (std::uint16_t pixel = 0U; pixel < chunk; ++pixel) {
        row[static_cast<std::size_t>(pixel) * 2U] = high;
        row[static_cast<std::size_t>(pixel) * 2U + 1U] = low;
      }

      for (std::uint16_t row_index = 0U; row_index < height; ++row_index) {
        const auto result = write_rect(
            static_cast<std::uint16_t>(x + x_offset),
            static_cast<std::uint16_t>(y + row_index), chunk, 1U,
            {row.data(), static_cast<std::size_t>(chunk) * 2U});
        if (!result) {
          return result;
        }
      }
      x_offset = static_cast<std::uint16_t>(x_offset + chunk);
    }
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto draw_pixel(std::uint16_t x, std::uint16_t y,
                                std::uint16_t color)
      -> result<void, error_type> {
    const std::array<std::byte, 2U> pixel{
        static_cast<std::byte>(color >> 8U),
        static_cast<std::byte>(color & 0xFFU)};
    return write_rect(x, y, 1U, 1U, pixel);
  }

 private:
  enum class command : std::uint8_t {
    software_reset = 0x01U,
    read_id_1 = 0xDAU,
    read_id_2 = 0xDBU,
    read_id_3 = 0xDCU,
    sleep_out = 0x11U,
    normal_display_on = 0x13U,
    display_off = 0x28U,
    display_on = 0x29U,
    column_address_set = 0x2AU,
    row_address_set = 0x2BU,
    memory_write = 0x2CU,
    memory_data_access_control = 0x36U,
    color_mode = 0x3AU,
    frame_rate_control_1 = 0xB1U,
    frame_rate_control_2 = 0xB2U,
    frame_rate_control_3 = 0xB3U,
    frame_inversion_control = 0xB4U,
    power_control_1 = 0xC0U,
    power_control_2 = 0xC1U,
    power_control_3 = 0xC2U,
    power_control_4 = 0xC3U,
    power_control_5 = 0xC4U,
    vcom_control = 0xC5U,
    positive_gamma = 0xE0U,
    negative_gamma = 0xE1U,
    display_inversion_on = 0x21U,
    display_inversion_off = 0x20U
  };

  class lock_guard {
   public:
    explicit lock_guard(Lock& lock) noexcept : lock_{lock} { lock_.lock(); }
    lock_guard(const lock_guard&) = delete;
    lock_guard& operator=(const lock_guard&) = delete;
    ~lock_guard() { lock_.unlock(); }

   private:
    Lock& lock_;
  };

  [[nodiscard]] static auto failure(error_type value)
      -> result<void, error_type> {
    return result<void, error_type>::failure(value);
  }

  [[nodiscard]] static auto map_bus_result(
      result<void, typename Bus::error_type> value)
      -> result<void, error_type> {
    if (!value) {
      return failure(error_type::bus(value.error().kind()));
    }
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto write_bus(hal::span<const std::byte> data)
      -> result<void, error_type> {
    return map_bus_result(bus_.write(data));
  }

  [[nodiscard]] auto read_bus(hal::span<std::byte> data)
      -> result<void, error_type> {
    const auto value = bus_.read(data, configuration_.bus.read_fill);
    if (!value) {
      return failure(error_type::bus(value.error().kind()));
    }
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto set_data_command(hal::gpio::level value)
      -> result<void, error_type> {
    const auto written = data_command_.write(value);
    if (!written) {
      return failure(error_type::data_command(written.error().kind()));
    }
    return result<void, error_type>::success();
  }

  template <class Function>
  [[nodiscard]] auto transaction(Function&& function)
      -> result<void, error_type> {
    if (!configuration_.valid()) {
      return failure(error_type::invalid_argument());
    }

    lock_guard guard{*lock_};
    const auto configured = bus_.configure(configuration_.bus);
    if (!configured) {
      return failure(error_type::bus(configured.error().kind()));
    }
    actual_frequency_ = configured.value();

    const auto selected = chip_select_.write(hal::gpio::level::low);
    if (!selected) {
      return failure(error_type::chip_select(selected.error().kind()));
    }
    bool selected_now = true;
    delay_.delay_for(configuration_.chip_select_setup);

    auto value = std::forward<Function>(function)();
    if (!value) {
      release(selected_now);
      return value;
    }
    const auto flushed = bus_.flush();
    if (!flushed) {
      release(selected_now);
      return failure(error_type::bus(flushed.error().kind()));
    }
    delay_.delay_for(configuration_.chip_select_hold);
    const auto deselected = chip_select_.write(hal::gpio::level::high);
    selected_now = false;
    if (!deselected) {
      return failure(error_type::chip_select(deselected.error().kind()));
    }
    return result<void, error_type>::success();
  }

  void release(bool& selected_now) noexcept {
    if (!selected_now) {
      return;
    }
    (void)bus_.flush();
    delay_.delay_for(configuration_.chip_select_hold);
    (void)chip_select_.write(hal::gpio::level::high);
    selected_now = false;
  }

  [[nodiscard]] auto send_command(
      command selected, hal::span<const std::byte> data)
      -> result<void, error_type> {
    return transaction([&]() -> result<void, error_type> {
      return write_command_active(selected, data);
    });
  }

  [[nodiscard]] auto write_command_active(
      command selected, hal::span<const std::byte> data)
      -> result<void, error_type> {
    const std::array<std::byte, 1U> command_byte{
        static_cast<std::byte>(selected)};
    if (auto result = set_data_command(hal::gpio::level::low); !result) {
      return result;
    }
    if (auto result = write_bus(command_byte); !result) {
      return result;
    }
    if (data.empty()) {
      return result<void, error_type>::success();
    }
    if (auto result = set_data_command(hal::gpio::level::high); !result) {
      return result;
    }
    return write_bus(data);
  }

  [[nodiscard]] auto write_data_active(hal::span<const std::byte> data)
      -> result<void, error_type> {
    if (!data.valid() || data.empty()) {
      return failure(error_type::invalid_argument());
    }
    if (auto result = set_data_command(hal::gpio::level::high); !result) {
      return result;
    }
    return write_bus(data);
  }

  [[nodiscard]] auto read_register(command selected, std::byte& response)
      -> result<void, error_type> {
    const std::array<std::byte, 1U> command_byte{
        static_cast<std::byte>(selected)};
    if (auto result = set_data_command(hal::gpio::level::low); !result) {
      return result;
    }
    if (auto result = write_bus(command_byte); !result) {
      return result;
    }
    if (auto result = set_data_command(hal::gpio::level::high); !result) {
      return result;
    }
    return read_bus({&response, 1U});
  }

  [[nodiscard]] bool valid_window(std::uint16_t x, std::uint16_t y,
                                  std::uint16_t width,
                                  std::uint16_t height) const noexcept {
    return width > 0U && height > 0U && x < panel_.width && y < panel_.height &&
           width <= static_cast<std::uint16_t>(panel_.width - x) &&
           height <= static_cast<std::uint16_t>(panel_.height - y) &&
           static_cast<std::uint32_t>(panel_.column_offset) + x + width <=
               0x1'0000U &&
           static_cast<std::uint32_t>(panel_.row_offset) + y + height <=
               0x1'0000U;
  }

  [[nodiscard]] auto address_data(std::uint16_t start,
                                  std::uint16_t length)
      -> std::array<std::byte, 4U> {
    const std::uint16_t end = static_cast<std::uint16_t>(start + length - 1U);
    return {static_cast<std::byte>(start >> 8U),
            static_cast<std::byte>(start & 0xFFU),
            static_cast<std::byte>(end >> 8U),
            static_cast<std::byte>(end & 0xFFU)};
  }

  [[nodiscard]] auto write_window_active(std::uint16_t x, std::uint16_t y,
                                         std::uint16_t width,
                                         std::uint16_t height)
      -> result<void, error_type> {
    const auto columns = address_data(static_cast<std::uint16_t>(
                                          panel_.column_offset + x),
                                      width);
    if (auto result = write_command_active(command::column_address_set, columns);
        !result) {
      return result;
    }
    const auto rows = address_data(static_cast<std::uint16_t>(
                                       panel_.row_offset + y),
                                   height);
    return write_command_active(command::row_address_set, rows);
  }

  [[nodiscard]] auto set_window_and_madctl() -> result<void, error_type> {
    return transaction([&]() -> result<void, error_type> {
      if (auto result = write_window_active(0U, 0U, panel_.width, panel_.height);
          !result) {
        return result;
      }
      const std::array<std::byte, 1U> madctl{
          static_cast<std::byte>(panel_.madctl)};
      return write_command_active(command::memory_data_access_control, madctl);
    });
  }

  Bus& bus_;
  ChipSelect& chip_select_;
  DataCommand& data_command_;
  Delay& delay_;
  [[no_unique_address]] hal::spi::NoLock default_lock_{};
  Lock* lock_{};
  configuration configuration_{};
  panel_profile panel_{};
  hal::hertz actual_frequency_{};
  bool initialized_{};
};

template <hal::spi::Bus8 Bus, hal::gpio::OutputPin ChipSelect,
          hal::gpio::OutputPin DataCommand, hal::time::Delay Delay,
          hal::spi::BasicLock Lock>
St7735(Bus&, ChipSelect&, DataCommand&, Delay&, configuration, Lock&)
    -> St7735<Bus, ChipSelect, DataCommand, Delay, Lock>;

template <hal::spi::Bus8 Bus, hal::gpio::OutputPin ChipSelect,
          hal::gpio::OutputPin DataCommand, hal::time::Delay Delay>
St7735(Bus&, ChipSelect&, DataCommand&, Delay&, configuration)
    -> St7735<Bus, ChipSelect, DataCommand, Delay, hal::spi::NoLock>;

}  // namespace hal::devices::st7735
