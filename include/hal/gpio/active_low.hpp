#pragma once

#include <hal/gpio.hpp>
#include <utility>

namespace hal::gpio {

// Presents one active-low physical signal with logical polarity. It never owns
// or reconfigures the underlying pin. Methods are available only when the
// underlying pin supports the corresponding input/output capability.
template <class Pin>
  requires(InputPin<Pin> || OutputPin<Pin>)
class ActiveLowPin {
 public:
  using error_type = typename Pin::error_type;

  explicit constexpr ActiveLowPin(Pin& pin) noexcept : pin_{pin} {}

  [[nodiscard]] auto write(level value) -> result<void, error_type>
    requires OutputPin<Pin>
  {
    return pin_.write(invert(value));
  }

  [[nodiscard]] auto output_latch() -> result<level, error_type>
    requires StatefulOutputPin<Pin>
  {
    auto current = pin_.output_latch();
    if (!current) {
      return result<level, error_type>::failure(std::move(current).error());
    }
    return result<level, error_type>::success(invert(current.value()));
  }

  [[nodiscard]] auto read() -> result<level, error_type>
    requires InputPin<Pin>
  {
    auto current = pin_.read();
    if (!current) {
      return result<level, error_type>::failure(std::move(current).error());
    }
    return result<level, error_type>::success(invert(current.value()));
  }

  [[nodiscard]] auto configure_edge(edge selected_edge)
      -> result<void, error_type>
    requires EdgeInput<Pin>
  {
    return pin_.configure_edge(invert(selected_edge));
  }

  [[nodiscard]] bool take_event() noexcept
    requires EdgeInput<Pin>
  {
    return pin_.take_event();
  }

  [[nodiscard]] constexpr Pin& underlying() noexcept { return pin_; }

 private:
  [[nodiscard]] static constexpr level invert(level value) noexcept {
    return value == level::low ? level::high : level::low;
  }

  [[nodiscard]] static constexpr edge invert(edge selected_edge) noexcept {
    switch (selected_edge) {
      case edge::rising:
        return edge::falling;
      case edge::falling:
        return edge::rising;
      case edge::both:
        return edge::both;
    }
    return edge::both;
  }

  Pin& pin_;
};

template <class Pin>
ActiveLowPin(Pin&) -> ActiveLowPin<Pin>;

}  // namespace hal::gpio
