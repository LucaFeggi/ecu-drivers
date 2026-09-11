#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/spi.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include <array>
#include <cstddef>
#include <cstdint>
#include <hal/contracts/spi_bus.hpp>
#include "support.hpp"

namespace hal::stm32f4::spi {

inline constexpr capability support{
    implementation_status::available,
    "8-bit SPI polling and legacy DMA stream transfers"};
class error {
public:
  explicit constexpr error(hal::spi::error_kind kind) noexcept : kind_{kind} {}
  [[nodiscard]] constexpr hal::spi::error_kind kind() const noexcept {
    return kind_;
  }

private:
  hal::spi::error_kind kind_{};
};

template <class Registers> class PollingBus8 {
public:
  using error_type = error;
  static constexpr bool supports_lsb_first{true};
  PollingBus8(Registers &r, std::uint32_t clock_hz,
              poll_budget timeout) noexcept
      : registers_{r}, clock_hz_{clock_hz}, timeout_{timeout} {}
  PollingBus8(const PollingBus8 &) = delete;
  PollingBus8 &operator=(const PollingBus8 &) = delete;
  [[nodiscard]] auto configure(hal::spi::config8 c)
      -> result<hertz, error_type> {
    if (clock_hz_ == 0U || c.max_frequency.value == 0U || !timeout_.valid() ||
        (registers_.CR1 & (1U << 6U)) != 0U)
      return failure<hertz>(hal::spi::error_kind::configuration);
    std::uint32_t divisor = 2U;
    std::uint32_t code = 0U;
    while (clock_hz_ / divisor > c.max_frequency.value && divisor < 256U) {
      divisor *= 2U;
      ++code;
    }
    if (clock_hz_ / divisor > c.max_frequency.value)
      return failure<hertz>(hal::spi::error_kind::configuration);
    std::uint32_t cr1 = (1U << 2U) | (1U << 8U) | (1U << 9U) | (code << 3U);
    if (c.clock_mode == hal::spi::mode::mode1 ||
        c.clock_mode == hal::spi::mode::mode3)
      cr1 |= 1U;
    if (c.clock_mode == hal::spi::mode::mode2 ||
        c.clock_mode == hal::spi::mode::mode3)
      cr1 |= 1U << 1U;
    if (c.order == hal::spi::bit_order::lsb_first)
      cr1 |= 1U << 7U;
    registers_.CR1 = cr1;
    registers_.CR2 = 0U;
    fill_ = c.read_fill;
    configured_ = true;
    return result<hertz, error_type>::success(hertz{clock_hz_ / divisor});
  }
  [[nodiscard]] auto write(span<const std::byte> data)
      -> result<void, error_type> {
    return exchange(data, {}, std::byte{0U});
  }
  [[nodiscard]] auto read(span<std::byte> data, std::byte fill)
      -> result<void, error_type> {
    return exchange({}, data, fill);
  }
  [[nodiscard]] auto transfer(span<const std::byte> tx, span<std::byte> rx)
      -> result<void, error_type> {
    return exchange(tx, rx, fill_);
  }
  [[nodiscard]] auto flush() -> result<void, error_type> {
    return wait(1U << 1U) ? result<void, error_type>::success()
                          : failure<void>(hal::spi::error_kind::timeout);
  }

private:
  template <class T>
  [[nodiscard]] static auto failure(hal::spi::error_kind k)
      -> result<T, error_type> {
    return result<T, error_type>::failure(error{k});
  }
  [[nodiscard]] bool wait(std::uint32_t flag) const noexcept {
    for (std::uint32_t n = timeout_.iterations; n > 0U; --n) {
      const auto status = registers_.SR;
      if ((status & flag) != 0U)
        return true;
      if ((status & ((1U << 5U) | (1U << 6U))) != 0U)
        return false;
    }
    return false;
  }
  [[nodiscard]] auto exchange(span<const std::byte> tx, span<std::byte> rx,
                              std::byte fill) -> result<void, error_type> {
    HAL_CORE_ASSERT(tx.valid());
    HAL_CORE_ASSERT(rx.valid());
    if (!configured_ || !tx.valid() || !rx.valid())
      return failure<void>(hal::spi::error_kind::configuration);
    const auto count = tx.size() > rx.size() ? tx.size() : rx.size();
    registers_.CR1 |= 1U << 6U;
    auto &dr = *reinterpret_cast<volatile std::uint8_t *>(&registers_.DR);
    for (std::size_t i = 0U; i < count; ++i) {
      if (!wait(1U << 1U)) {
        registers_.CR1 &= ~(1U << 6U);
        return failure<void>(hal::spi::error_kind::timeout);
      }
      dr = i < tx.size() ? std::to_integer<std::uint8_t>(tx[i])
                         : std::to_integer<std::uint8_t>(fill);
      if (!wait(1U)) {
        registers_.CR1 &= ~(1U << 6U);
        return failure<void>(hal::spi::error_kind::timeout);
      }
      const auto received = static_cast<std::byte>(dr);
      if (i < rx.size())
        rx[i] = received;
    }
    if (!wait(1U << 1U)) {
      registers_.CR1 &= ~(1U << 6U);
      return failure<void>(hal::spi::error_kind::timeout);
    }
    registers_.CR1 &= ~(1U << 6U);
    return result<void, error_type>::success();
  }
  Registers &registers_;
  std::uint32_t clock_hz_{};
  poll_budget timeout_{};
  std::byte fill_{0xFF};
  bool configured_{};
};

template <class Registers, class TxStream, class RxStream,
          std::size_t Capacity = 4096U>
class DmaBus8 {
  static_assert(Capacity > 0U && Capacity <= 65'535U);

public:
  using error_type = error;
  static constexpr bool supports_lsb_first{true};
  DmaBus8(Registers &r, TxStream &tx, RxStream &rx, std::uint32_t clock_hz,
          poll_budget timeout, std::uint8_t tx_channel, std::uint8_t rx_channel,
          std::array<std::byte, Capacity> &tx_storage,
          std::array<std::byte, Capacity> &rx_storage) noexcept
      : registers_{r}, tx_{tx}, rx_{rx}, clock_hz_{clock_hz}, timeout_{timeout},
        tx_channel_{tx_channel}, rx_channel_{rx_channel},
        tx_storage_{tx_storage}, rx_storage_{rx_storage} {}
  DmaBus8(const DmaBus8 &) = delete;
  DmaBus8 &operator=(const DmaBus8 &) = delete;
  [[nodiscard]] auto configure(hal::spi::config8 c)
      -> result<hertz, error_type> {
    PollingBus8<Registers> polling{registers_, clock_hz_, timeout_};
    auto result = polling.configure(c);
    if (!result)
      return result;
    fill_ = c.read_fill;
    configured_ = true;
    return result;
  }
  [[nodiscard]] auto write(span<const std::byte> data)
      -> result<void, error_type> {
    return exchange(data, {}, std::byte{0U});
  }
  [[nodiscard]] auto read(span<std::byte> data, std::byte fill)
      -> result<void, error_type> {
    return exchange({}, data, fill);
  }
  [[nodiscard]] auto transfer(span<const std::byte> tx, span<std::byte> rx)
      -> result<void, error_type> {
    return exchange(tx, rx, fill_);
  }
  [[nodiscard]] auto flush() -> result<void, error_type> {
    return wait(1U << 1U) ? result<void, error_type>::success()
                          : failure<void>(hal::spi::error_kind::timeout);
  }

private:
  template <class T>
  [[nodiscard]] static auto failure(hal::spi::error_kind k)
      -> result<T, error_type> {
    return result<T, error_type>::failure(error{k});
  }
  [[nodiscard]] bool wait(std::uint32_t flag) const noexcept {
    for (std::uint32_t n = timeout_.iterations; n > 0U; --n) {
      const auto status = registers_.SR;
      if ((status & flag) != 0U)
        return true;
      if ((status & ((1U << 5U) | (1U << 6U))) != 0U)
        return false;
    }
    return false;
  }
  template <class Stream>
  static void setup(Stream &stream, std::uint8_t channel,
                    const volatile void *peripheral, volatile void *memory,
                    std::size_t count, bool memory_to_peripheral, bool circular,
                    bool memory_increment) noexcept {
    stream.CR &= ~1U;
    stream.CR = (static_cast<std::uint32_t>(channel) << 25U) |
                (memory_to_peripheral ? (1U << 6U) : 0U) |
                (circular ? (1U << 8U) : 0U) |
                (memory_increment ? (1U << 10U) : 0U);
    stream.PAR = static_cast<std::uint32_t>(
        reinterpret_cast<std::uintptr_t>(peripheral));
    stream.M0AR =
        static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(memory));
    stream.NDTR = static_cast<std::uint32_t>(count);
    stream.FCR = 0U;
    stream.CR |= 1U;
  }
  [[nodiscard]] auto exchange(span<const std::byte> tx, span<std::byte> rx,
                              std::byte fill) -> result<void, error_type> {
    HAL_CORE_ASSERT(tx.valid());
    HAL_CORE_ASSERT(rx.valid());
    if (!configured_ || !tx.valid() || !rx.valid())
      return failure<void>(hal::spi::error_kind::configuration);
    const auto count = tx.size() > rx.size() ? tx.size() : rx.size();
    if (count == 0U)
      return result<void, error_type>::success();
    if (count > Capacity || count > 0xFFFFU)
      return failure<void>(hal::spi::error_kind::configuration);
    for (std::size_t i = 0U; i < count; ++i)
      tx_storage_[i] = i < tx.size() ? tx[i] : fill;
    setup(tx_, tx_channel_, &registers_.DR, tx_storage_.data(), count, true,
          false, true);
    setup(rx_, rx_channel_, &registers_.DR, rx_storage_.data(), count, false,
          false, true);
    registers_.CR2 |= (1U << 1U) | (1U << 0U);
    registers_.CR1 |= 1U << 6U;
    for (std::uint32_t n = timeout_.iterations; n > 0U; --n)
      if (tx_.NDTR == 0U && rx_.NDTR == 0U)
        break;
    if (tx_.NDTR != 0U || rx_.NDTR != 0U) {
      tx_.CR &= ~1U;
      rx_.CR &= ~1U;
      registers_.CR2 &= ~((1U << 1U) | (1U << 0U));
      registers_.CR1 &= ~(1U << 6U);
      return failure<void>(hal::spi::error_kind::timeout);
    }
    tx_.CR &= ~1U;
    rx_.CR &= ~1U;
    registers_.CR2 &= ~((1U << 1U) | (1U << 0U));
    registers_.CR1 &= ~(1U << 6U);
    for (std::size_t i = 0U; i < rx.size(); ++i)
      rx[i] = rx_storage_[i];
    return result<void, error_type>::success();
  }
  Registers &registers_;
  TxStream &tx_;
  RxStream &rx_;
  std::uint32_t clock_hz_{};
  poll_budget timeout_{};
  std::uint8_t tx_channel_{};
  std::uint8_t rx_channel_{};
  std::array<std::byte, Capacity> &tx_storage_;
  std::array<std::byte, Capacity> &rx_storage_;
  std::byte fill_{0xFF};
  bool configured_{};
};
} // namespace hal::stm32f4::spi
