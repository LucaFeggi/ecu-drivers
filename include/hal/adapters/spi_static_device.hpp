#pragma once

#include <hal/foundation/assert.hpp>
#include <hal/contracts/gpio.hpp>
#include <hal/contracts/spi_bus.hpp>
#include <hal/contracts/time.hpp>
#include <utility>

namespace hal::spi {

struct device_config8 {
  config8 bus{};
  nanoseconds cs_setup{};
  nanoseconds cs_hold{};
};

enum class static_device_error_source : std::uint8_t {
  bus,
  chip_select,
  invalid_operation,
  unsupported_bit_order
};

// Adapter errors retain the portable source kind. Concrete target errors remain
// available at the Bus/CS boundary where richer diagnostics are required.
class static_device_error {
 public:
  [[nodiscard]] static constexpr static_device_error bus(
      error_kind kind) noexcept {
    return {kind, static_device_error_source::bus};
  }

  [[nodiscard]] static constexpr static_device_error chip_select() noexcept {
    return {error_kind::io, static_device_error_source::chip_select};
  }

  [[nodiscard]] static constexpr static_device_error
  invalid_operation() noexcept {
    return {error_kind::configuration,
            static_device_error_source::invalid_operation};
  }

  [[nodiscard]] static constexpr static_device_error
  unsupported_bit_order() noexcept {
    return {error_kind::configuration,
            static_device_error_source::unsupported_bit_order};
  }

  [[nodiscard]] constexpr error_kind kind() const noexcept { return kind_; }
  [[nodiscard]] constexpr static_device_error_source source() const noexcept {
    return source_;
  }

 private:
  constexpr static_device_error(error_kind kind,
                                static_device_error_source source) noexcept
      : kind_{kind}, source_{source} {}

  error_kind kind_;
  static_device_error_source source_;
};

template <class T>
concept BasicLock = requires(T& lock) {
  { lock.lock() } noexcept -> std::same_as<void>;
  { lock.unlock() } noexcept -> std::same_as<void>;
};

// Explicit single-owner policy. It intentionally performs no synchronization.
struct NoLock {
  constexpr void lock() noexcept {}
  constexpr void unlock() noexcept {}
};

namespace detail {

template <BasicLock Lock>
class lock_guard {
 public:
  explicit lock_guard(Lock& lock) noexcept : lock_{lock} { lock_.lock(); }
  lock_guard(const lock_guard&) = delete;
  lock_guard& operator=(const lock_guard&) = delete;
  ~lock_guard() { lock_.unlock(); }

 private:
  Lock& lock_;
};

template <class Bus>
inline constexpr bool bus_supports_lsb_first = []() constexpr {
  if constexpr (requires {
                  {
                    Bus::supports_lsb_first
                  } -> std::convertible_to<const bool&>;
                }) {
    return static_cast<bool>(Bus::supports_lsb_first);
  } else {
    return false;
  }
}();

}  // namespace detail

// Statically composes one physical bus, one chip-select pin, timing, and a lock
// into the logical Device8 transaction boundary. The asserted level defaults to
// active-low; use gpio::ActiveLowPin or the constructor argument
// otherwise.
template <Bus8 Bus, gpio::OutputPin ChipSelect, time::Delay Delay,
          BasicLock Lock = NoLock>
class StaticDevice8 {
 public:
  using error_type = static_device_error;
  constexpr StaticDevice8(
      Bus& bus, ChipSelect& chip_select, Delay& delay, Lock& lock,
      device_config8 configuration,
      gpio::level asserted_level = gpio::level::low) noexcept
      : bus_{bus},
        chip_select_{chip_select},
        delay_{delay},
        lock_{&lock},
        configuration_{configuration},
        asserted_level_{asserted_level} {}

  constexpr StaticDevice8(
      Bus& bus, ChipSelect& chip_select, Delay& delay,
      device_config8 configuration,
      gpio::level asserted_level = gpio::level::low) noexcept
    requires std::same_as<Lock, NoLock>
      : bus_{bus},
        chip_select_{chip_select},
        delay_{delay},
        lock_{&default_lock_},
        configuration_{configuration},
        asserted_level_{asserted_level} {}

  StaticDevice8(const StaticDevice8&) = delete;
  StaticDevice8& operator=(const StaticDevice8&) = delete;

  [[nodiscard]] auto transaction(span<const operation8> operations)
      -> result<void, error_type> {
    HAL_CORE_ASSERT(operations.valid());
    if (!operations.valid()) {
      return fail(error_type::invalid_operation());
    }

    for (const operation8& operation : operations) {
      HAL_CORE_ASSERT(operation.valid());
      if (!operation.valid()) {
        return fail(error_type::invalid_operation());
      }
    }

    if (configuration_.bus.order == bit_order::lsb_first &&
        !detail::bus_supports_lsb_first<Bus>) {
      return fail(error_type::unsupported_bit_order());
    }

    detail::lock_guard guard{*lock_};

    auto configured = bus_.configure(configuration_.bus);
    if (!configured) {
      return fail(error_type::bus(configured.error().kind()));
    }
    actual_frequency_ = configured.value();

    auto selected = chip_select_.write(asserted_level_);
    if (!selected) {
      return fail(error_type::chip_select());
    }

    bool selected_now = true;
    delay_.delay_for(configuration_.cs_setup);

    for (const operation8& operation : operations) {
      auto executed = execute(operation);
      if (!executed) {
        best_effort_release(selected_now);
        return executed;
      }
    }

    auto flushed = bus_.flush();
    if (!flushed) {
      best_effort_release(selected_now);
      return fail(error_type::bus(flushed.error().kind()));
    }

    delay_.delay_for(configuration_.cs_hold);
    auto deselected = chip_select_.write(deasserted_level());
    selected_now = false;
    if (!deselected) {
      return fail(error_type::chip_select());
    }

    return result<void, error_type>::success();
  }

  [[nodiscard]] constexpr hertz actual_frequency() const noexcept {
    return actual_frequency_;
  }
  [[nodiscard]] constexpr Bus& bus() noexcept { return bus_; }
  [[nodiscard]] constexpr ChipSelect& chip_select() noexcept {
    return chip_select_;
  }

 private:
  [[nodiscard]] static auto fail(error_type error) -> result<void, error_type> {
    return result<void, error_type>::failure(error);
  }

  [[nodiscard]] constexpr gpio::level deasserted_level() const noexcept {
    return asserted_level_ == gpio::level::low ? gpio::level::high
                                               : gpio::level::low;
  }

  [[nodiscard]] auto execute(const operation8& operation)
      -> result<void, error_type> {
    switch (operation.kind()) {
      case operation_kind::write:
        return map_bus_result(bus_.write(operation.tx()));
      case operation_kind::read:
        return map_bus_result(
            bus_.read(operation.rx(), configuration_.bus.read_fill));
      case operation_kind::transfer:
        return map_bus_result(bus_.transfer(operation.tx(), operation.rx()));
      case operation_kind::delay: {
        // A delay begins only after all preceding clocks physically finish.
        auto flushed = bus_.flush();
        if (!flushed) {
          return fail(error_type::bus(flushed.error().kind()));
        }
        delay_.delay_for(operation.delay());
        return result<void, error_type>::success();
      }
    }
    return fail(error_type::invalid_operation());
  }

  [[nodiscard]] static auto map_bus_result(
      result<void, typename Bus::error_type> bus_result)
      -> result<void, error_type> {
    if (!bus_result) {
      return fail(error_type::bus(bus_result.error().kind()));
    }
    return result<void, error_type>::success();
  }

  void best_effort_release(bool& selected_now) noexcept {
    if (!selected_now) {
      return;
    }
    // Ignore cleanup errors so the first transaction failure is retained.
    (void)bus_.flush();
    delay_.delay_for(configuration_.cs_hold);
    (void)chip_select_.write(deasserted_level());
    selected_now = false;
  }

  Bus& bus_;
  ChipSelect& chip_select_;
  Delay& delay_;
  [[no_unique_address]] NoLock default_lock_{};
  Lock* lock_{};
  device_config8 configuration_{};
  gpio::level asserted_level_{gpio::level::low};
  hertz actual_frequency_{};
};

template <Bus8 Bus, gpio::OutputPin ChipSelect, time::Delay Delay,
          BasicLock Lock>
StaticDevice8(Bus&, ChipSelect&, Delay&, Lock&, device_config8, gpio::level)
    -> StaticDevice8<Bus, ChipSelect, Delay, Lock>;

template <Bus8 Bus, gpio::OutputPin ChipSelect, time::Delay Delay,
          BasicLock Lock>
StaticDevice8(Bus&, ChipSelect&, Delay&, Lock&, device_config8)
    -> StaticDevice8<Bus, ChipSelect, Delay, Lock>;

template <Bus8 Bus, gpio::OutputPin ChipSelect, time::Delay Delay>
StaticDevice8(Bus&, ChipSelect&, Delay&, device_config8, gpio::level)
    -> StaticDevice8<Bus, ChipSelect, Delay, NoLock>;

template <Bus8 Bus, gpio::OutputPin ChipSelect, time::Delay Delay>
StaticDevice8(Bus&, ChipSelect&, Delay&, device_config8)
    -> StaticDevice8<Bus, ChipSelect, Delay, NoLock>;

// Architecture-facing descriptive name; StaticDevice8 remains the concise API.
template <Bus8 Bus, gpio::OutputPin ChipSelect, time::Delay Delay,
          BasicLock Lock = NoLock>
using StaticSpiDevice = StaticDevice8<Bus, ChipSelect, Delay, Lock>;

}  // namespace hal::spi
