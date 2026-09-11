#pragma once

#include <cstdint>
#include <hal/gpio/pin_capabilities.hpp>
#include <hal/foundation/result.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/gpio.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif
#include <hal/gpio/pin.hpp>
#include <soc/gpio_struct.h>

namespace hal::esp32s3_wroom_1_n16r8::gpio {

struct signal {
  std::uint16_t index{};
};

namespace signal_index {

// ESP32-S3 GPIO matrix signal numbers from gpio_sig_map.h. They are kept as
// typed constants here so board code never has to traffic in unexplained
// integer literals.
inline constexpr signal uart0_rx{12U};
inline constexpr signal uart0_tx{12U};
inline constexpr signal uart1_rx{15U};
inline constexpr signal uart1_tx{15U};
inline constexpr signal uart2_rx{18U};
inline constexpr signal uart2_tx{18U};

inline constexpr signal i2c0_scl{89U};
inline constexpr signal i2c0_sda{90U};
inline constexpr signal i2c1_scl{91U};
inline constexpr signal i2c1_sda{92U};

inline constexpr signal spi3_clk{66U};
inline constexpr signal spi3_q{67U};
inline constexpr signal spi3_d{68U};
inline constexpr signal spi3_cs0{71U};

inline constexpr signal spi2_clk{101U};
inline constexpr signal spi2_q{102U};
inline constexpr signal spi2_d{103U};
inline constexpr signal spi2_cs0{110U};

inline constexpr signal twai_rx{116U};

// SDMMC slot 0 uses the GPIO matrix for both directions. Configure the
// output route first and then the input route (or use connect_bidirectional).
inline constexpr signal sdmmc0_clk{172U};
inline constexpr signal sdmmc0_cmd{178U};
inline constexpr signal sdmmc0_d0{180U};
inline constexpr signal sdmmc0_d1{181U};
inline constexpr signal sdmmc0_d2{182U};
inline constexpr signal sdmmc0_d3{183U};

[[nodiscard]] constexpr signal ledc_channel(std::uint8_t channel) noexcept {
  return signal{static_cast<std::uint16_t>(73U + channel)};
}

}  // namespace signal_index

class matrix_error {
 public:
  [[nodiscard]] static constexpr matrix_error unavailable() noexcept {
    return matrix_error{hal::gpio::error_kind::unavailable};
  }
  [[nodiscard]] constexpr hal::gpio::error_kind kind() const noexcept {
    return kind_;
  }

 private:
  constexpr explicit matrix_error(hal::gpio::error_kind kind) noexcept
      : kind_{kind} {}
  hal::gpio::error_kind kind_;
};

[[nodiscard]] inline result<void, matrix_error> connect_input(
    gpio_dev_t& gpio, std::uint8_t pin, signal selected,
    bool invert = false,
    const hal::esp32s3::gpio::pin_capabilities& pins =
        hal::esp32s3::gpio::supported_module_pins) noexcept {
  if (!pins.valid(pin) || selected.index >= 256U) {
    return result<void, matrix_error>::failure(matrix_error::unavailable());
  }

  // A peripheral input is still physically sourced by the pad. Do not enable
  // the GPIO output driver here: the peripheral owns only the input matrix.
  detail::enable_gpio_input(pin);
  auto& route = gpio.func_in_sel_cfg[selected.index];
  route.val = (static_cast<std::uint32_t>(pin) & 0x3FU) |
              (invert ? 1U << 6U : 0U) | (1U << 7U);
  return result<void, matrix_error>::success();
}

[[nodiscard]] inline result<void, matrix_error> connect_output(
    gpio_dev_t& gpio, std::uint8_t pin, signal selected,
    bool invert = false, bool peripheral_output_enable = true,
    const hal::esp32s3::gpio::pin_capabilities& pins =
        hal::esp32s3::gpio::supported_module_pins) noexcept {
  if (!pins.output_capable(pin) || selected.index >= 512U) {
    return result<void, matrix_error>::failure(matrix_error::unavailable());
  }

  detail::select_gpio_function(pin, false);
  auto& route = gpio.func_out_sel_cfg[pin];
  route.val = (static_cast<std::uint32_t>(selected.index) & 0x1FFU) |
              (invert ? 1U << 9U : 0U) |
              (peripheral_output_enable ? 0U : 1U << 10U);
  if (!peripheral_output_enable) {
    // With oen_sel set, the GPIO_ENABLE register supplies the output enable.
    if (pin < 32U) {
      gpio.enable_w1ts = std::uint32_t{1U} << pin;
    } else {
      gpio.enable1_w1ts.val = std::uint32_t{1U} << (pin - 32U);
    }
  }
  return result<void, matrix_error>::success();
}

[[nodiscard]] inline result<void, matrix_error> connect_bidirectional(
    gpio_dev_t& gpio, std::uint8_t pin, signal selected,
    bool invert = false,
    const hal::esp32s3::gpio::pin_capabilities& pins =
        hal::esp32s3::gpio::supported_module_pins) noexcept {
  const auto output = connect_output(gpio, pin, selected, invert, true, pins);
  if (!output) {
    return output;
  }
  return connect_input(gpio, pin, selected, invert, pins);
}

}  // namespace hal::esp32s3_wroom_1_n16r8::gpio
