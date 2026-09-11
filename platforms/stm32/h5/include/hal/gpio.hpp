#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/gpio.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include <cstdint>
#include "support.hpp"

namespace hal::stm32h5::gpio {

inline constexpr capability support{implementation_status::available,
                                    "direct GPIO BSRR/IDR access"};

class error {
public:
  [[nodiscard]] constexpr hal::gpio::error_kind kind() const noexcept {
    return hal::gpio::error_kind::unavailable;
  }
};

enum class pull : std::uint8_t { none = 0U, up = 1U, down = 2U };
enum class speed : std::uint8_t {
  low = 0U,
  medium = 1U,
  high = 2U,
  very_high = 3U
};
enum class output_type : std::uint8_t { push_pull = 0U, open_drain = 1U };

template <class Port>
[[nodiscard]] inline bool configure_input(Port &port, std::uint8_t pin,
                                          pull selected_pull) noexcept {
  if (pin >= 16U) {
    return false;
  }
  const std::uint32_t shift = static_cast<std::uint32_t>(pin) * 2U;
  const std::uint32_t field = 3U << shift;
  port.MODER = port.MODER & ~field;
  port.PUPDR = (port.PUPDR & ~field) |
               (static_cast<std::uint32_t>(selected_pull) << shift);
  return true;
}

template <class Port>
[[nodiscard]] inline bool
configure_output(Port &port, std::uint8_t pin, hal::gpio::level initial,
                 output_type selected_type = output_type::push_pull,
                 speed selected_speed = speed::low,
                 pull selected_pull = pull::none) noexcept {
  if (pin >= 16U) {
    return false;
  }
  const std::uint32_t mask = std::uint32_t{1U} << pin;
  const std::uint32_t shift = static_cast<std::uint32_t>(pin) * 2U;
  const std::uint32_t field = 3U << shift;
  // Establish the inactive/initial latch before exposing the pin as output.
  port.BSRR = initial == hal::gpio::level::high ? mask : (mask << 16U);
  port.OTYPER = (port.OTYPER & ~mask) |
                (static_cast<std::uint32_t>(selected_type) << pin);
  port.OSPEEDR = (port.OSPEEDR & ~field) |
                 (static_cast<std::uint32_t>(selected_speed) << shift);
  port.PUPDR = (port.PUPDR & ~field) |
               (static_cast<std::uint32_t>(selected_pull) << shift);
  port.MODER = (port.MODER & ~field) | (1U << shift);
  return true;
}

template <class Port>
[[nodiscard]] inline bool configure_analog(Port &port,
                                           std::uint8_t pin) noexcept {
  if (pin >= 16U) {
    return false;
  }
  const std::uint32_t shift = static_cast<std::uint32_t>(pin) * 2U;
  const std::uint32_t field = 3U << shift;
  port.PUPDR = port.PUPDR & ~field;
  port.MODER = port.MODER | field;
  return true;
}

template <class Port>
[[nodiscard]] inline bool
configure_alternate(Port &port, std::uint8_t pin,
                    std::uint8_t alternate_function,
                    output_type selected_type = output_type::push_pull,
                    speed selected_speed = speed::very_high,
                    pull selected_pull = pull::none) noexcept {
  if (pin >= 16U || alternate_function >= 16U) {
    return false;
  }
  const std::uint32_t mask = std::uint32_t{1U} << pin;
  const std::uint32_t shift = static_cast<std::uint32_t>(pin) * 2U;
  const std::uint32_t field = 3U << shift;
  const std::uint32_t afr_index = pin / 8U;
  const std::uint32_t afr_shift = static_cast<std::uint32_t>(pin % 8U) * 4U;
  const std::uint32_t afr_mask = 15U << afr_shift;
  port.OTYPER = (port.OTYPER & ~mask) |
                (static_cast<std::uint32_t>(selected_type) << pin);
  port.OSPEEDR = (port.OSPEEDR & ~field) |
                 (static_cast<std::uint32_t>(selected_speed) << shift);
  port.PUPDR = (port.PUPDR & ~field) |
               (static_cast<std::uint32_t>(selected_pull) << shift);
  port.AFR[afr_index] =
      (port.AFR[afr_index] & ~afr_mask) |
      (static_cast<std::uint32_t>(alternate_function) << afr_shift);
  port.MODER = (port.MODER & ~field) | (2U << shift);
  return true;
}

template <class Port> class OutputPin {
public:
  using error_type = error;

  constexpr OutputPin(Port &port, std::uint8_t pin) noexcept
      : port_{port}, mask_{pin < 16U ? std::uint32_t{1U} << pin : 0U},
        valid_{pin < 16U} {
    HAL_CORE_ASSERT(pin < 16U);
  }

  OutputPin(const OutputPin &) = delete;
  OutputPin &operator=(const OutputPin &) = delete;

  [[nodiscard]] auto write(hal::gpio::level value) -> result<void, error_type> {
    if (!valid_) {
      return result<void, error_type>::failure(error{});
    }
    port_.BSRR = value == hal::gpio::level::high ? mask_ : (mask_ << 16U);
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto output_latch() -> result<hal::gpio::level, error_type> {
    if (!valid_) {
      return result<hal::gpio::level, error_type>::failure(error{});
    }
    return result<hal::gpio::level, error_type>::success(
        (port_.ODR & mask_) != 0U ? hal::gpio::level::high
                                  : hal::gpio::level::low);
  }

private:
  Port &port_;
  std::uint32_t mask_{};
  bool valid_{};
};

template <class Port> class InputPin {
public:
  using error_type = error;

  constexpr InputPin(Port &port, std::uint8_t pin) noexcept
      : port_{port}, mask_{pin < 16U ? std::uint32_t{1U} << pin : 0U},
        valid_{pin < 16U} {
    HAL_CORE_ASSERT(pin < 16U);
  }

  InputPin(const InputPin &) = delete;
  InputPin &operator=(const InputPin &) = delete;

  [[nodiscard]] auto read() -> result<hal::gpio::level, error_type> {
    if (!valid_) {
      return result<hal::gpio::level, error_type>::failure(error{});
    }
    return result<hal::gpio::level, error_type>::success(
        (port_.IDR & mask_) != 0U ? hal::gpio::level::high
                                  : hal::gpio::level::low);
  }

private:
  Port &port_;
  std::uint32_t mask_{};
  bool valid_{};
};

template <class Port> OutputPin(Port &, std::uint8_t) -> OutputPin<Port>;

template <class Port> InputPin(Port &, std::uint8_t) -> InputPin<Port>;

// EXTI is a separate resource from GPIO. The BSP supplies the EXTI instance
// plus the encoded GPIO port number for the selected line.
// The pending latch deliberately coalesces edges: the portable contract
// promises an event, not an edge counter.
template <class Port, class Exti> class EdgeInput {
public:
  using error_type = error;

  EdgeInput(Port &port, Exti &exti, std::uint8_t pin,
            std::uint8_t port_code) noexcept
      : input_{port, pin}, exti_{exti},
        mask_{pin < 16U ? std::uint32_t{1U} << pin : 0U}, pin_{pin},
        port_code_{port_code} {
    HAL_CORE_ASSERT(pin < 16U && port_code < 16U);
  }

  EdgeInput(const EdgeInput &) = delete;
  EdgeInput &operator=(const EdgeInput &) = delete;

  [[nodiscard]] auto read() -> result<hal::gpio::level, error_type> {
    return input_.read();
  }

  [[nodiscard]] auto configure_edge(hal::gpio::edge selected_edge)
      -> result<void, error_type> {
    if (pin_ >= 16U || port_code_ >= 16U) {
      return result<void, error_type>::failure(error{});
    }
    const std::uint32_t exticr_index = pin_ / 4U;
    const std::uint32_t exticr_shift =
        static_cast<std::uint32_t>(pin_ % 4U) * 4U;
    const std::uint32_t exticr_mask = 0xFU << exticr_shift;
    exti_.EXTICR[exticr_index] =
        (exti_.EXTICR[exticr_index] & ~exticr_mask) |
        (static_cast<std::uint32_t>(port_code_) << exticr_shift);
    exti_.IMR1 = exti_.IMR1 | mask_;
    exti_.RTSR1 = exti_.RTSR1 & ~mask_;
    exti_.FTSR1 = exti_.FTSR1 & ~mask_;
    if (selected_edge == hal::gpio::edge::rising ||
        selected_edge == hal::gpio::edge::both) {
      exti_.RTSR1 = exti_.RTSR1 | mask_;
    }
    if (selected_edge == hal::gpio::edge::falling ||
        selected_edge == hal::gpio::edge::both) {
      exti_.FTSR1 = exti_.FTSR1 | mask_;
    }
    exti_.RPR1 = mask_;
    exti_.FPR1 = mask_;
    return result<void, error_type>::success();
  }

  [[nodiscard]] bool take_event() noexcept {
    const std::uint32_t pending = (exti_.RPR1 | exti_.FPR1) & mask_;
    if (pending != 0U) {
      exti_.RPR1 = pending;
      exti_.FPR1 = pending;
      return true;
    }
    return false;
  }

private:
  InputPin<Port> input_;
  Exti &exti_;
  std::uint32_t mask_{};
  std::uint8_t pin_{};
  std::uint8_t port_code_{};
};

} // namespace hal::stm32h5::gpio
