#pragma once

#include <cstdint>
#include <hal/mcu/esp32s3_wroom_1_n16r8/pins.hpp>
#include <hal/foundation/result.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/gpio.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif
#include <soc/gpio_struct.h>
#include <soc/io_mux_reg.h>

namespace hal::esp32s3_wroom_1_n16r8::gpio {

class error {
 public:
  [[nodiscard]] static constexpr error unavailable() noexcept {
    return error{hal::gpio::error_kind::unavailable};
  }
  [[nodiscard]] static constexpr error io() noexcept {
    return error{hal::gpio::error_kind::io};
  }
  [[nodiscard]] constexpr hal::gpio::error_kind kind() const noexcept {
    return kind_;
  }

 private:
  constexpr explicit error(hal::gpio::error_kind kind) noexcept : kind_{kind} {}
  hal::gpio::error_kind kind_;
};

struct pin_config {
  bool input{};
  bool output{};
  bool open_drain{};
};

namespace detail {

inline void select_gpio_function(std::uint8_t pin, bool input) noexcept {
  volatile std::uint32_t& mux = *reinterpret_cast<volatile std::uint32_t*>(
      static_cast<std::uintptr_t>(IO_MUX_GPIO0_REG) +
      static_cast<std::uintptr_t>(pin) * 4U);
  std::uint32_t value = mux;
  value = (value & ~static_cast<std::uint32_t>(MCU_SEL_M)) |
          (static_cast<std::uint32_t>(PIN_FUNC_GPIO) << MCU_SEL_S);
  if (input) {
    value |= static_cast<std::uint32_t>(FUN_IE_M);
  } else {
    value &= ~static_cast<std::uint32_t>(FUN_IE_M);
  }
  mux = value;
}

inline void enable_gpio_input(std::uint8_t pin) noexcept {
  volatile std::uint32_t& mux = *reinterpret_cast<volatile std::uint32_t*>(
      static_cast<std::uintptr_t>(IO_MUX_GPIO0_REG) +
      static_cast<std::uintptr_t>(pin) * 4U);
  mux = mux | static_cast<std::uint32_t>(FUN_IE_M);
}

}  // namespace detail

[[nodiscard]] inline result<void, error> configure(gpio_dev_t& gpio,
                                                   std::uint8_t pin,
                                                   pin_config configuration) {
  if (!device::is_valid_gpio(pin) ||
      (configuration.output && !device::is_output_capable(pin)) ||
      (!configuration.input && !configuration.output)) {
    return result<void, error>::failure(error::unavailable());
  }

  const std::uint32_t bit = std::uint32_t{1U} << (pin & 31U);
  if (pin < 32U) {
    if (configuration.output) {
      gpio.enable_w1ts = bit;
    } else {
      gpio.enable_w1tc = bit;
    }
  } else {
    const std::uint32_t high_bit = std::uint32_t{1U} << (pin - 32U);
    if (configuration.output) {
      gpio.enable1_w1ts.val = high_bit;
    } else {
      gpio.enable1_w1tc.val = high_bit;
    }
  }

  detail::select_gpio_function(pin, configuration.input);
  gpio.pin[pin].pad_driver = configuration.open_drain ? 1U : 0U;
  return result<void, error>::success();
}

class OutputPin {
 public:
  using error_type = error;

  OutputPin(gpio_dev_t& gpio, std::uint8_t pin, bool open_drain = false) noexcept
      : gpio_{&gpio}, pin_{pin}, open_drain_{open_drain} {}

  [[nodiscard]] result<void, error_type> configure() noexcept {
    return gpio::configure(*gpio_, pin_, {false, true, open_drain_});
  }

  [[nodiscard]] result<void, error_type> write(hal::gpio::level value) noexcept {
    if (!device::is_output_capable(pin_)) {
      return result<void, error_type>::failure(error_type::unavailable());
    }
    const std::uint32_t bit = std::uint32_t{1U} << (pin_ & 31U);
    if (pin_ < 32U) {
      if (value == hal::gpio::level::high) {
        gpio_->out_w1ts = bit;
      } else {
        gpio_->out_w1tc = bit;
      }
    } else {
      const std::uint32_t high_bit = std::uint32_t{1U} << (pin_ - 32U);
      if (value == hal::gpio::level::high) {
        gpio_->out1_w1ts.val = high_bit;
      } else {
        gpio_->out1_w1tc.val = high_bit;
      }
    }
    return result<void, error_type>::success();
  }

  [[nodiscard]] result<hal::gpio::level, error_type> output_latch()
      const noexcept {
    const std::uint32_t bit = std::uint32_t{1U} << (pin_ & 31U);
    const std::uint32_t value =
        pin_ < 32U ? gpio_->out : gpio_->out1.val;
    return result<hal::gpio::level, error_type>::success(
        (value & bit) != 0U ? hal::gpio::level::high : hal::gpio::level::low);
  }

 private:
  gpio_dev_t* gpio_;
  std::uint8_t pin_;
  bool open_drain_;
};

class InputPin {
 public:
  using error_type = error;

  InputPin(gpio_dev_t& gpio, std::uint8_t pin) noexcept
      : gpio_{&gpio}, pin_{pin} {}

  [[nodiscard]] result<void, error_type> configure() noexcept {
    return gpio::configure(*gpio_, pin_, {true, false, false});
  }

  [[nodiscard]] result<hal::gpio::level, error_type> read() const noexcept {
    if (!device::is_valid_gpio(pin_)) {
      return result<hal::gpio::level, error_type>::failure(
          error_type::unavailable());
    }
    const std::uint32_t bit = std::uint32_t{1U} << (pin_ & 31U);
    const std::uint32_t value = pin_ < 32U ? gpio_->in : gpio_->in1.val;
    return result<hal::gpio::level, error_type>::success(
        (value & bit) != 0U ? hal::gpio::level::high : hal::gpio::level::low);
  }

 protected:
  gpio_dev_t* gpio_;
  std::uint8_t pin_;
};

}  // namespace hal::esp32s3_wroom_1_n16r8::gpio
