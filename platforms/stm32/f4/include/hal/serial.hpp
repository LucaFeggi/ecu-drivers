#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/serial.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include <array>
#include <cstddef>
#include <cstdint>
#include "support.hpp"

namespace hal::stm32f4::serial {

inline constexpr capability support{
    implementation_status::available,
    "USART polling and legacy DMA stream transfers"};
class error {
public:
  explicit constexpr error(hal::serial::error_kind kind) noexcept
      : kind_{kind} {}
  [[nodiscard]] constexpr hal::serial::error_kind kind() const noexcept {
    return kind_;
  }

private:
  hal::serial::error_kind kind_{};
};

template <class Registers> class PollingPort {
public:
  using error_type = error;
  PollingPort(Registers &r, std::uint32_t clock_hz,
              poll_budget timeout) noexcept
      : registers_{r}, clock_hz_{clock_hz}, timeout_{timeout} {}
  PollingPort(const PollingPort &) = delete;
  PollingPort &operator=(const PollingPort &) = delete;
  [[nodiscard]] auto configure(hal::serial::config c)
      -> result<hertz, error_type> {
    if (clock_hz_ == 0U || c.baud_rate.value == 0U ||
        c.bits == hal::serial::data_bits::bits9 || !timeout_.valid())
      return failure<hertz>(hal::serial::error_kind::configuration);
    const auto baud = static_cast<std::uint32_t>(c.baud_rate.value);
    const auto divisor = (clock_hz_ + baud / 2U) / baud;
    if (divisor == 0U || divisor > 0xFFFFU)
      return failure<hertz>(hal::serial::error_kind::configuration);
    registers_.CR1 = 0U;
    registers_.CR2 = c.stops == hal::serial::stop_bits::two ? (2U << 12U) : 0U;
    registers_.CR3 = flow_bits(c.flow);
    registers_.BRR = divisor;
    std::uint32_t cr1 = (1U << 13U) | (1U << 2U) | (1U << 3U);
    if (c.parity_mode != hal::serial::parity::none) {
      cr1 |= 1U << 10U;
      if (c.parity_mode == hal::serial::parity::odd)
        cr1 |= 1U << 9U;
    }
    if (c.bits == hal::serial::data_bits::bits7)
      cr1 |= 1U << 12U;
    registers_.CR1 = cr1;
    configured_ = true;
    return result<hertz, error_type>::success(hertz{clock_hz_ / divisor});
  }
  [[nodiscard]] auto try_write(span<const std::byte> data)
      -> result<std::size_t, error_type> {
    HAL_CORE_ASSERT(data.valid());
    if (!configured_ || !data.valid())
      return failure<std::size_t>(hal::serial::error_kind::configuration);
    std::size_t n = 0U;
    while (n < data.size() && (registers_.SR & (1U << 7U)) != 0U)
      registers_.DR = std::to_integer<std::uint8_t>(data[n++]);
    return result<std::size_t, error_type>::success(n);
  }
  [[nodiscard]] auto try_read(span<std::byte> data)
      -> result<std::size_t, error_type> {
    HAL_CORE_ASSERT(data.valid());
    if (!configured_ || !data.valid())
      return failure<std::size_t>(hal::serial::error_kind::configuration);
    const auto status = registers_.SR;
    if ((status & (1U << 3U)) != 0U)
      return failure<std::size_t>(hal::serial::error_kind::overrun);
    if ((status & (1U << 1U)) != 0U)
      return failure<std::size_t>(hal::serial::error_kind::framing);
    if ((status & 1U) != 0U)
      return failure<std::size_t>(hal::serial::error_kind::parity);
    std::size_t n = 0U;
    while (n < data.size() && (registers_.SR & (1U << 5U)) != 0U)
      data[n++] = static_cast<std::byte>(registers_.DR & 0xFFU);
    return result<std::size_t, error_type>::success(n);
  }
  [[nodiscard]] auto flush() -> result<void, error_type> {
    if (!configured_)
      return failure<void>(hal::serial::error_kind::configuration);
    for (std::uint32_t n = timeout_.iterations; n > 0U; --n)
      if ((registers_.SR & (1U << 6U)) != 0U)
        return result<void, error_type>::success();
    return failure<void>(hal::serial::error_kind::timeout);
  }

private:
  template <class T>
  [[nodiscard]] static auto failure(hal::serial::error_kind k)
      -> result<T, error_type> {
    return result<T, error_type>::failure(error{k});
  }
  static constexpr std::uint32_t
  flow_bits(hal::serial::flow_control f) noexcept {
    return f == hal::serial::flow_control::rts       ? (1U << 8U)
           : f == hal::serial::flow_control::cts     ? (1U << 9U)
           : f == hal::serial::flow_control::rts_cts ? ((1U << 8U) | (1U << 9U))
                                                     : 0U;
  }
  Registers &registers_;
  std::uint32_t clock_hz_{};
  poll_budget timeout_{};
  bool configured_{};
};

template <class Registers, class TxStream, class RxStream,
          std::size_t TxCapacity = 1024U, std::size_t RxCapacity = 2048U>
class DmaPort {
  static_assert(TxCapacity > 0U && RxCapacity > 0U && TxCapacity <= 65'535U &&
                RxCapacity <= 65'535U);

public:
  using error_type = error;
  DmaPort(Registers &r, TxStream &tx, RxStream &rx, std::uint32_t clock_hz,
          std::uint8_t tx_channel, std::uint8_t rx_channel,
          std::array<std::byte, TxCapacity> &tx_buffer,
          std::array<std::byte, RxCapacity> &rx_buffer,
          poll_budget timeout = {100'000U}) noexcept
      : registers_{r}, tx_{tx}, rx_{rx}, clock_hz_{clock_hz},
        tx_channel_{tx_channel}, rx_channel_{rx_channel}, tx_buffer_{tx_buffer},
        rx_buffer_{rx_buffer}, timeout_{timeout} {}
  DmaPort(const DmaPort &) = delete;
  DmaPort &operator=(const DmaPort &) = delete;
  [[nodiscard]] auto configure(hal::serial::config c)
      -> result<hertz, error_type> {
    PollingPort<Registers> polling{registers_, clock_hz_, timeout_};
    auto result = polling.configure(c);
    if (!result)
      return result;
    configure_stream(rx_, rx_channel_, &registers_.DR, rx_buffer_.data(),
                     RxCapacity, false, false, true);
    rx_consumed_ = 0U;
    registers_.CR3 |= (1U << 6U);
    configured_ = true;
    return result;
  }
  [[nodiscard]] auto try_write(span<const std::byte> data)
      -> result<std::size_t, error_type> {
    HAL_CORE_ASSERT(data.valid());
    if (!configured_ || !data.valid())
      return failure<std::size_t>(hal::serial::error_kind::configuration);
    const auto count = data.size() < TxCapacity ? data.size() : TxCapacity;
    if (count == 0U || (tx_.CR & 1U) != 0U)
      return result<std::size_t, error_type>::success(0U);
    for (std::size_t i = 0U; i < count; ++i)
      tx_buffer_[i] = data[i];
    configure_stream(tx_, tx_channel_, &registers_.DR, tx_buffer_.data(), count,
                     true, false, false);
    registers_.CR3 |= 1U << 7U;
    return result<std::size_t, error_type>::success(count);
  }
  [[nodiscard]] auto try_read(span<std::byte> data)
      -> result<std::size_t, error_type> {
    HAL_CORE_ASSERT(data.valid());
    if (!configured_ || !data.valid())
      return failure<std::size_t>(hal::serial::error_kind::configuration);
    const auto count = data.size() < RxCapacity ? data.size() : RxCapacity;
    const auto produced = RxCapacity - rx_.NDTR;
    const auto available = produced - rx_consumed_;
    const auto copy = available < count ? available : count;
    for (std::size_t i = 0U; i < copy; ++i)
      data[i] = rx_buffer_[rx_consumed_ + i];
    rx_consumed_ += copy;
    if (rx_consumed_ == RxCapacity && produced == RxCapacity) {
      configure_stream(rx_, rx_channel_, &registers_.DR, rx_buffer_.data(),
                       RxCapacity, false, false, true);
      rx_consumed_ = 0U;
    }
    return result<std::size_t, error_type>::success(copy);
  }
  [[nodiscard]] auto flush() -> result<void, error_type> {
    for (std::uint32_t n = timeout_.iterations; n > 0U; --n)
      if ((tx_.NDTR == 0U) && ((registers_.SR & (1U << 6U)) != 0U)) {
        tx_.CR &= ~1U;
        registers_.CR3 &= ~(1U << 7U);
        return result<void, error_type>::success();
      }
    return failure<void>(hal::serial::error_kind::timeout);
  }
  void on_tx_dma_interrupt() noexcept {
    tx_.CR &= ~1U;
    registers_.CR3 &= ~(1U << 7U);
  }
  void on_rx_dma_interrupt() noexcept {}

private:
  template <class T>
  [[nodiscard]] static auto failure(hal::serial::error_kind k)
      -> result<T, error_type> {
    return result<T, error_type>::failure(error{k});
  }
  template <class Stream>
  static void configure_stream(Stream &stream, std::uint8_t channel,
                               const volatile void *peripheral,
                               volatile void *memory, std::size_t count,
                               bool memory_to_peripheral, bool circular,
                               bool memory_increment) noexcept {
    stream.CR &= ~1U;
    stream.CR = (static_cast<std::uint32_t>(channel) << 25U) |
                (memory_to_peripheral ? (1U << 6U) : 0U) |
                (circular ? (1U << 8U) : 0U) |
                (memory_increment ? (1U << 10U) : 0U) | (1U << 4U) | (1U << 2U);
    stream.PAR = static_cast<std::uint32_t>(
        reinterpret_cast<std::uintptr_t>(peripheral));
    stream.M0AR =
        static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(memory));
    stream.NDTR = static_cast<std::uint32_t>(count);
    stream.FCR = 0U;
    stream.CR |= 1U;
  }
  Registers &registers_;
  TxStream &tx_;
  RxStream &rx_;
  std::uint32_t clock_hz_{};
  std::uint8_t tx_channel_{};
  std::uint8_t rx_channel_{};
  std::array<std::byte, TxCapacity> &tx_buffer_;
  std::array<std::byte, RxCapacity> &rx_buffer_;
  poll_budget timeout_{};
  std::size_t rx_consumed_{};
  bool configured_{};
};
} // namespace hal::stm32f4::serial
