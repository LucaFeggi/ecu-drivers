#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <hal/serial.hpp>
#include <hal/stm32h5/support.hpp>

namespace hal::stm32h5::serial {

inline constexpr capability support{
    implementation_status::available,
    "USART polling and GPDMA payload transfers"};

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
  PollingPort(Registers &registers, std::uint32_t clock_hz,
              poll_budget timeout) noexcept
      : registers_{registers}, clock_hz_{clock_hz}, timeout_{timeout} {}
  PollingPort(const PollingPort &) = delete;
  PollingPort &operator=(const PollingPort &) = delete;

  [[nodiscard]] auto configure(hal::serial::config configuration)
      -> result<hertz, error_type> {
    if (clock_hz_ == 0U || configuration.baud_rate.value == 0U ||
        configuration.bits == hal::serial::data_bits::bits9 ||
        !timeout_.valid()) {
      return failure<hertz>(hal::serial::error_kind::configuration);
    }
    const auto baud = static_cast<std::uint32_t>(configuration.baud_rate.value);
    const std::uint32_t divisor = (clock_hz_ + baud / 2U) / baud;
    if (divisor == 0U || divisor > 0xFFFFU) {
      return failure<hertz>(hal::serial::error_kind::configuration);
    }
    registers_.CR1 = 0U;
    registers_.CR2 =
        configuration.stops == hal::serial::stop_bits::two ? (2U << 12U) : 0U;
    registers_.CR3 = flow_bits(configuration.flow);
    registers_.BRR = divisor;
    std::uint32_t cr1 = (1U << 2U) | (1U << 3U);
    if (configuration.parity_mode != hal::serial::parity::none) {
      cr1 |= 1U << 10U;
      if (configuration.parity_mode == hal::serial::parity::odd)
        cr1 |= 1U << 9U;
    }
    if (configuration.bits == hal::serial::data_bits::bits7)
      cr1 |= 1U << 28U;
    registers_.ICR = error_clear;
    registers_.CR1 = cr1 | 1U;
    configured_ = true;
    return result<hertz, error_type>::success(hertz{clock_hz_ / divisor});
  }

  [[nodiscard]] auto try_write(span<const std::byte> data)
      -> result<std::size_t, error_type> {
    HAL_CORE_ASSERT(data.valid());
    if (!configured_ || !data.valid())
      return failure<std::size_t>(hal::serial::error_kind::configuration);
    std::size_t count = 0U;
    while (count < data.size() && (registers_.ISR & tx_empty) != 0U) {
      registers_.TDR = std::to_integer<std::uint8_t>(data[count++]);
    }
    return result<std::size_t, error_type>::success(count);
  }

  [[nodiscard]] auto try_read(span<std::byte> data)
      -> result<std::size_t, error_type> {
    HAL_CORE_ASSERT(data.valid());
    if (!configured_ || !data.valid())
      return failure<std::size_t>(hal::serial::error_kind::configuration);
    const auto observed = receive_error();
    if (observed != hal::serial::error_kind::other) {
      registers_.ICR = error_clear;
      return failure<std::size_t>(observed);
    }
    std::size_t count = 0U;
    while (count < data.size() && (registers_.ISR & rx_ready) != 0U) {
      data[count++] = static_cast<std::byte>(registers_.RDR & 0xFFU);
    }
    return result<std::size_t, error_type>::success(count);
  }

  [[nodiscard]] auto flush() -> result<void, error_type> {
    if (!configured_ || !wait(transmission_complete))
      return failure<void>(hal::serial::error_kind::timeout);
    return result<void, error_type>::success();
  }

private:
  template <class T>
  [[nodiscard]] static auto failure(hal::serial::error_kind kind)
      -> result<T, error_type> {
    return result<T, error_type>::failure(error{kind});
  }
  static constexpr std::uint32_t tx_empty{1U << 7U};
  static constexpr std::uint32_t rx_ready{1U << 5U};
  static constexpr std::uint32_t transmission_complete{1U << 6U};
  static constexpr std::uint32_t parity_error{1U << 0U};
  static constexpr std::uint32_t framing_error{1U << 1U};
  static constexpr std::uint32_t noise_error{1U << 2U};
  static constexpr std::uint32_t overrun_error{1U << 3U};
  static constexpr std::uint32_t error_clear{
      (1U << 0U) | (1U << 1U) | (1U << 2U) | (1U << 3U) | (1U << 8U)};
  static constexpr std::uint32_t rts{1U << 8U};
  static constexpr std::uint32_t cts{1U << 9U};
  [[nodiscard]] static constexpr std::uint32_t
  flow_bits(hal::serial::flow_control flow) noexcept {
    return flow == hal::serial::flow_control::rts       ? rts
           : flow == hal::serial::flow_control::cts     ? cts
           : flow == hal::serial::flow_control::rts_cts ? rts | cts
                                                        : 0U;
  }
  [[nodiscard]] hal::serial::error_kind receive_error() const noexcept {
    const auto status = registers_.ISR;
    if ((status & overrun_error) != 0U)
      return hal::serial::error_kind::overrun;
    if ((status & framing_error) != 0U)
      return hal::serial::error_kind::framing;
    if ((status & parity_error) != 0U)
      return hal::serial::error_kind::parity;
    if ((status & noise_error) != 0U)
      return hal::serial::error_kind::io;
    return hal::serial::error_kind::other;
  }
  [[nodiscard]] bool wait(std::uint32_t mask) const noexcept {
    for (std::uint32_t left = timeout_.iterations; left > 0U; --left)
      if ((registers_.ISR & mask) != 0U)
        return true;
    return false;
  }
  Registers &registers_;
  std::uint32_t clock_hz_{};
  poll_budget timeout_{};
  bool configured_{};
};

// GPDMA is configured as a byte-wide, one-shot payload engine. The BSP owns
// the channel/request routing and the caller-owned staging buffers keep the
// driver independent of cache placement and lifetime policy.
template <class Registers, class TxChannel, class RxChannel,
          std::size_t TxCapacity = 1024U, std::size_t RxCapacity = 2048U>
class DmaPort {
  static_assert(TxCapacity > 0U && RxCapacity > 0U);
  static_assert(TxCapacity <= 65'535U && RxCapacity <= 65'535U);

public:
  using error_type = error;
  DmaPort(Registers &registers, TxChannel &tx, RxChannel &rx,
          std::uint32_t clock_hz, std::uint8_t tx_request,
          std::uint8_t rx_request,
          std::array<std::byte, TxCapacity> &tx_storage,
          std::array<std::byte, RxCapacity> &rx_storage,
          poll_budget timeout = {100'000U}) noexcept
      : registers_{registers}, tx_{tx}, rx_{rx}, clock_hz_{clock_hz},
        tx_request_{tx_request}, rx_request_{rx_request},
        tx_storage_{tx_storage}, rx_storage_{rx_storage}, timeout_{timeout} {}
  DmaPort(const DmaPort &) = delete;
  DmaPort &operator=(const DmaPort &) = delete;

  [[nodiscard]] auto configure(hal::serial::config configuration)
      -> result<hertz, error_type> {
    PollingPort<Registers> port{registers_, clock_hz_, timeout_};
    const auto result = port.configure(configuration);
    if (!result)
      return result;
    // Keep the USART configured, then attach the receive DMA channel.
    registers_.CR3 |= rx_dma;
    prepare_rx();
    rx_consumed_ = 0U;
    configured_ = true;
    return result;
  }

  [[nodiscard]] auto try_write(span<const std::byte> data)
      -> result<std::size_t, error_type> {
    HAL_CORE_ASSERT(data.valid());
    if (!configured_ || !data.valid())
      return failure<std::size_t>(hal::serial::error_kind::configuration);
    const auto count = data.size() < TxCapacity ? data.size() : TxCapacity;
    if (count == 0U || tx_.CCR != 0U)
      return result<std::size_t, error_type>::success(0U);
    for (std::size_t i = 0U; i < count; ++i)
      tx_storage_[i] = data[i];
    configure_channel(tx_, tx_request_, tx_storage_.data(), &registers_.TDR,
                      count, true, false);
    registers_.CR3 |= tx_dma;
    return result<std::size_t, error_type>::success(count);
  }

  [[nodiscard]] auto try_read(span<std::byte> data)
      -> result<std::size_t, error_type> {
    HAL_CORE_ASSERT(data.valid());
    if (!configured_ || !data.valid())
      return failure<std::size_t>(hal::serial::error_kind::configuration);
    const auto status = registers_.ISR;
    if ((status & overrun_error) != 0U)
      return failure<std::size_t>(hal::serial::error_kind::overrun);
    const std::size_t produced = RxCapacity - rx_.CBR1;
    const std::size_t available = produced - rx_consumed_;
    const std::size_t count = available < data.size() ? available : data.size();
    for (std::size_t i = 0U; i < count; ++i)
      data[i] = rx_storage_[rx_consumed_ + i];
    rx_consumed_ += count;
    if (rx_consumed_ == RxCapacity && produced == RxCapacity) {
      prepare_rx();
      rx_consumed_ = 0U;
    }
    return result<std::size_t, error_type>::success(count);
  }

  [[nodiscard]] auto flush() -> result<void, error_type> {
    for (std::uint32_t left = timeout_.iterations; left > 0U; --left) {
      if (tx_.CBR1 == 0U && (registers_.ISR & transmission_complete) != 0U) {
        registers_.CR3 &= ~tx_dma;
        return result<void, error_type>::success();
      }
    }
    return failure<void>(hal::serial::error_kind::timeout);
  }
  void on_tx_dma_interrupt() noexcept {
    tx_.CCR &= ~dma_enable;
    registers_.CR3 &= ~tx_dma;
  }
  void on_rx_dma_interrupt() noexcept {}

private:
  static constexpr std::uint32_t dma_enable{1U};
  static constexpr std::uint32_t rx_dma{1U << 6U};
  static constexpr std::uint32_t tx_dma{1U << 7U};
  static constexpr std::uint32_t transmission_complete{1U << 6U};
  static constexpr std::uint32_t overrun_error{1U << 3U};
  template <class T>
  [[nodiscard]] static auto failure(hal::serial::error_kind kind)
      -> result<T, error_type> {
    return result<T, error_type>::failure(error{kind});
  }
  static void barrier() noexcept {
    std::atomic_thread_fence(std::memory_order_seq_cst);
  }
  template <class Channel>
  static void configure_channel(Channel &channel, std::uint8_t request,
                                const volatile void *source,
                                volatile void *destination, std::size_t count,
                                bool source_increment,
                                bool destination_increment) noexcept {
    channel.CCR &= ~dma_enable;
    channel.CTR1 = (source_increment ? (1U << 3U) : 0U) |
                   (destination_increment ? (1U << 19U) : 0U);
    channel.CTR2 = static_cast<std::uint32_t>(request);
    channel.CBR1 = static_cast<std::uint32_t>(count);
    channel.CSAR =
        static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(source));
    channel.CDAR = static_cast<std::uint32_t>(
        reinterpret_cast<std::uintptr_t>(destination));
    barrier();
    channel.CCR = dma_enable;
  }
  void prepare_rx() noexcept {
    configure_channel(rx_, rx_request_, &registers_.RDR, rx_storage_.data(),
                      RxCapacity, false, true);
  }
  Registers &registers_;
  TxChannel &tx_;
  RxChannel &rx_;
  std::uint32_t clock_hz_{};
  std::uint8_t tx_request_{};
  std::uint8_t rx_request_{};
  std::array<std::byte, TxCapacity> &tx_storage_;
  std::array<std::byte, RxCapacity> &rx_storage_;
  poll_budget timeout_{};
  std::size_t rx_consumed_{};
  bool configured_{};
};

} // namespace hal::stm32h5::serial
