#pragma once

#include <hal/gpio/pin.hpp>

namespace hal::esp32s3_wroom_1_n16r8::gpio {

class EdgeInput : public InputPin {
 public:
  using error_type = error;

  EdgeInput(gpio_dev_t& gpio, std::uint8_t pin) noexcept
      : InputPin{gpio, pin} {}

  [[nodiscard]] result<void, error_type> configure_edge(
      hal::gpio::edge selected_edge) noexcept {
    auto configured = InputPin::configure();
    if (!configured) {
      return configured;
    }

    // GPIO interrupt type encoding: 1 rising, 2 falling, 3 both edges.
    const std::uint32_t type =
        selected_edge == hal::gpio::edge::rising
            ? 1U
            : selected_edge == hal::gpio::edge::falling ? 2U : 3U;
    constexpr std::uint32_t int_type_mask = 0x7U << 7U;
    constexpr std::uint32_t int_enable_mask = 0x1FU << 13U;
    gpio_->pin[pin_].val =
        (gpio_->pin[pin_].val & ~(int_type_mask | int_enable_mask)) |
        (type << 7U) | (1U << 13U);  // enable the GPIO status latch
    clear_pending();
    return result<void, error_type>::success();
  }

  [[nodiscard]] bool take_event() noexcept {
    const std::uint32_t bit = std::uint32_t{1U} << (pin_ & 31U);
    const bool pending = pin_ < 32U ? (gpio_->status & bit) != 0U
                                    : (gpio_->status1.val & bit) != 0U;
    if (pending) {
      clear_pending();
    }
    return pending;
  }

  void clear_pending() noexcept {
    const std::uint32_t bit = std::uint32_t{1U} << (pin_ & 31U);
    if (pin_ < 32U) {
      gpio_->status_w1tc = bit;
    } else {
      gpio_->status1_w1tc.val = bit;
    }
  }
};

}  // namespace hal::esp32s3_wroom_1_n16r8::gpio
