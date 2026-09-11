#pragma once

#include <hal/contracts/spi.hpp>

namespace hal::spi {

// Controller-facing configuration. Device drivers normally do not need this:
// they consume a Device8 whose bus settings and chip select are already bound.
enum class mode : std::uint8_t { mode0, mode1, mode2, mode3 };
enum class bit_order : std::uint8_t { msb_first, lsb_first };

struct config8 {
  hertz max_frequency{};
  mode clock_mode{mode::mode0};
  bit_order order{bit_order::msb_first};
  std::byte read_fill{std::byte{0xFF}};
};

template <class T>
concept Bus8 = Error<typename T::error_type> &&
               requires(T& bus, config8 configuration, span<const std::byte> tx,
                        span<std::byte> rx) {
                 {
                   bus.configure(configuration)
                 } -> std::same_as<result<hertz, typename T::error_type>>;
                 {
                   bus.write(tx)
                 } -> std::same_as<result<void, typename T::error_type>>;
                 {
                   bus.read(rx, std::byte{0xFF})
                 } -> std::same_as<result<void, typename T::error_type>>;
                 {
                   bus.transfer(tx, rx)
                 } -> std::same_as<result<void, typename T::error_type>>;
                 {
                   bus.flush()
                 } -> std::same_as<result<void, typename T::error_type>>;
               };

template <class T>
concept LsbFirstBus8 = Bus8<T> && requires {
  { T::supports_lsb_first } -> std::convertible_to<const bool&>;
  requires T::supports_lsb_first;
};

}  // namespace hal::spi
