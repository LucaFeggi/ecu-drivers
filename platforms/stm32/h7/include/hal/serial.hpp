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
#include <atomic>
#include <cstddef>
#include <cstdint>
#include "support.hpp"

namespace hal::stm32h7::serial {

inline constexpr capability support{
    implementation_status::available,
    "direct USART polling and circular RX/one-shot TX DMA implementations"};

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

  PollingPort(Registers &registers, std::uint32_t kernel_clock_hz,
              poll_budget timeout) noexcept
      : registers_{registers}, kernel_clock_hz_{kernel_clock_hz},
        timeout_{timeout} {}

  PollingPort(const PollingPort &) = delete;
  PollingPort &operator=(const PollingPort &) = delete;

  [[nodiscard]] auto configure(hal::serial::config configuration)
      -> result<hertz, error_type> {
    configured_ = false;
    if (!valid(configuration) || !timeout_.valid()) {
      return result<hertz, error_type>::failure(
          error{hal::serial::error_kind::configuration});
    }

    registers_.CR1 = 0U;
    registers_.CR2 = 0U;
    registers_.CR3 = 0U;

    const std::uint32_t requested =
        static_cast<std::uint32_t>(configuration.baud_rate.value);
    const std::uint32_t divisor =
        (kernel_clock_hz_ + (requested / 2U)) / requested;
    if (divisor < 16U || divisor > 0xFFFFU) {
      return result<hertz, error_type>::failure(
          error{hal::serial::error_kind::configuration});
    }

    std::uint32_t cr1 = cr1_transmitter_enable | cr1_receiver_enable;
    if (configuration.parity_mode != hal::serial::parity::none) {
      cr1 |= cr1_parity_enable;
      if (configuration.parity_mode == hal::serial::parity::odd) {
        cr1 |= cr1_parity_odd;
      }
    }
    if (configuration.parity_mode == hal::serial::parity::none) {
      if (configuration.bits == hal::serial::data_bits::bits9) {
        cr1 |= cr1_word_length_0;
      } else if (configuration.bits == hal::serial::data_bits::bits7) {
        cr1 |= cr1_word_length_1;
      }
    } else if (configuration.bits == hal::serial::data_bits::bits8) {
      cr1 |= cr1_word_length_0;
    }

    registers_.BRR = divisor;
    registers_.CR2 = configuration.stops == hal::serial::stop_bits::two
                         ? cr2_two_stop_bits
                         : 0U;
    registers_.CR3 =
        configuration.flow == hal::serial::flow_control::rts   ? cr3_rts_enable
        : configuration.flow == hal::serial::flow_control::cts ? cr3_cts_enable
        : configuration.flow == hal::serial::flow_control::rts_cts
            ? cr3_rts_enable | cr3_cts_enable
            : 0U;
    registers_.ICR = icr_all_receive_errors;
    registers_.CR1 = cr1 | cr1_usart_enable;
    if (!wait_set(isr_transmitter_acknowledge | isr_receiver_acknowledge)) {
      registers_.CR1 = 0U;
      return result<hertz, error_type>::failure(
          error{hal::serial::error_kind::timeout});
    }
    configured_ = true;
    const std::uint32_t actual = kernel_clock_hz_ / divisor;
    return result<hertz, error_type>::success(hertz{actual});
  }

  [[nodiscard]] auto try_write(span<const std::byte> data)
      -> result<std::size_t, error_type> {
    HAL_CORE_ASSERT(data.valid());
    if (!configured_) {
      return result<std::size_t, error_type>::failure(
          error{hal::serial::error_kind::configuration});
    }
    if (!data.valid()) {
      return result<std::size_t, error_type>::failure(
          error{hal::serial::error_kind::io});
    }
    std::size_t written = 0U;
    while (written < data.size() &&
           (registers_.ISR & isr_tx_fifo_not_full) != 0U) {
      registers_.TDR = static_cast<std::uint8_t>(data[written]);
      ++written;
    }
    return result<std::size_t, error_type>::success(written);
  }

  [[nodiscard]] auto try_read(span<std::byte> data)
      -> result<std::size_t, error_type> {
    HAL_CORE_ASSERT(data.valid());
    if (!configured_) {
      return result<std::size_t, error_type>::failure(
          error{hal::serial::error_kind::configuration});
    }
    if (!data.valid()) {
      return result<std::size_t, error_type>::failure(
          error{hal::serial::error_kind::io});
    }
    if (const auto receive_error = check_receive_error();
        receive_error != hal::serial::error_kind::other) {
      registers_.ICR = icr_all_receive_errors;
      return result<std::size_t, error_type>::failure(error{receive_error});
    }

    std::size_t read = 0U;
    while (read < data.size() &&
           (registers_.ISR & isr_rx_fifo_not_empty) != 0U) {
      data[read] =
          static_cast<std::byte>(static_cast<std::uint8_t>(registers_.RDR & 0xFFU));
      ++read;
    }
    return result<std::size_t, error_type>::success(read);
  }

  [[nodiscard]] auto flush() -> result<void, error_type> {
    if (!configured_) {
      return result<void, error_type>::failure(
          error{hal::serial::error_kind::configuration});
    }
    if (wait_set(isr_transmission_complete)) {
      return result<void, error_type>::success();
    }
    return result<void, error_type>::failure(
        error{hal::serial::error_kind::timeout});
  }

private:
  static constexpr std::uint32_t cr1_usart_enable{1U << 0U};
  static constexpr std::uint32_t cr1_receiver_enable{1U << 2U};
  static constexpr std::uint32_t cr1_transmitter_enable{1U << 3U};
  static constexpr std::uint32_t cr1_parity_odd{1U << 9U};
  static constexpr std::uint32_t cr1_parity_enable{1U << 10U};
  static constexpr std::uint32_t cr1_word_length_0{1U << 12U};
  static constexpr std::uint32_t cr1_word_length_1{1U << 28U};
  static constexpr std::uint32_t cr2_two_stop_bits{2U << 12U};
  static constexpr std::uint32_t cr3_rts_enable{1U << 8U};
  static constexpr std::uint32_t cr3_cts_enable{1U << 9U};
  static constexpr std::uint32_t isr_parity_error{1U << 0U};
  static constexpr std::uint32_t isr_framing_error{1U << 1U};
  static constexpr std::uint32_t isr_noise_error{1U << 2U};
  static constexpr std::uint32_t isr_overrun_error{1U << 3U};
  static constexpr std::uint32_t isr_rx_fifo_not_empty{1U << 5U};
  static constexpr std::uint32_t isr_transmission_complete{1U << 6U};
  static constexpr std::uint32_t isr_tx_fifo_not_full{1U << 7U};
  static constexpr std::uint32_t isr_transmitter_acknowledge{1U << 21U};
  static constexpr std::uint32_t isr_receiver_acknowledge{1U << 22U};
  static constexpr std::uint32_t icr_all_receive_errors{(1U << 0U) |
                                                        (1U << 1U) |
                                                        (1U << 2U) |
                                                        (1U << 3U) |
                                                        (1U << 8U)};

  [[nodiscard]] bool wait_set(std::uint32_t mask) const noexcept {
    for (std::uint32_t remaining = timeout_.iterations; remaining > 0U;
         --remaining) {
      if ((registers_.ISR & mask) == mask) {
        return true;
      }
    }
    return false;
  }

  [[nodiscard]] bool
  valid(const hal::serial::config &configuration) const noexcept {
    if (configuration.baud_rate.value == 0U ||
        configuration.baud_rate.value > 0xFFFF'FFFFULL ||
        kernel_clock_hz_ == 0U) {
      return false;
    }
    // The portable hal-core API transports bytes, not 9-bit words. Do not
    // silently configure a 9-bit peripheral mode and discard the ninth bit.
    return configuration.bits != hal::serial::data_bits::bits9;
  }

  [[nodiscard]] hal::serial::error_kind check_receive_error() const noexcept {
    const std::uint32_t status = registers_.ISR;
    if ((status & isr_overrun_error) != 0U) {
      return hal::serial::error_kind::overrun;
    }
    if ((status & isr_framing_error) != 0U) {
      return hal::serial::error_kind::framing;
    }
    if ((status & isr_parity_error) != 0U) {
      return hal::serial::error_kind::parity;
    }
    if ((status & isr_noise_error) != 0U) {
      return hal::serial::error_kind::io;
    }
    return hal::serial::error_kind::other;
  }

  Registers &registers_;
  std::uint32_t kernel_clock_hz_{};
  poll_budget timeout_{};
  bool configured_{};
};

} // namespace hal::stm32h7::serial

namespace hal::stm32h7::serial {

// DMA USART port. TX is a one-shot DMA transfer from BSP-owned staging
// storage, so the caller's span may be temporary. RX is a circular DMA ring;
// try_read() advances a consumer cursor without touching the peripheral data
// register. The BSP dispatches DMA completion/error interrupts to the two
// handlers below. No byte-by-byte IRQ or polling path is used; flush() has a
// bounded completion fallback because the hal-core TX contract is synchronous.
template <class Registers, class TxDmaStream, class RxDmaStream,
          class TxDmamuxChannel, class RxDmamuxChannel,
          std::size_t TxCapacity = 1024U, std::size_t RxCapacity = 2048U>
class DmaPort {
  static_assert(TxCapacity > 0U);
  static_assert(RxCapacity > 1U);
  static_assert(TxCapacity <= 65'535U);
  static_assert(RxCapacity <= 65'535U);
  static_assert((RxCapacity % 2U) == 0U);
  static_assert(std::atomic_bool::is_always_lock_free);
  static_assert(std::atomic<hal::serial::error_kind>::is_always_lock_free);

public:
  using error_type = error;

  DmaPort(Registers &registers, TxDmaStream &tx_stream,
          RxDmaStream &rx_stream, TxDmamuxChannel &tx_dmamux,
          RxDmamuxChannel &rx_dmamux, std::uint32_t kernel_clock_hz,
          std::uint8_t tx_request, std::uint8_t rx_request,
          std::array<std::byte, TxCapacity> &tx_buffer,
          std::array<std::byte, RxCapacity> &rx_ring,
          poll_budget timeout = {100'000U}) noexcept
      : registers_{registers}, tx_stream_{tx_stream}, rx_stream_{rx_stream},
        tx_dmamux_{tx_dmamux}, rx_dmamux_{rx_dmamux},
        kernel_clock_hz_{kernel_clock_hz}, tx_request_{tx_request},
        rx_request_{rx_request}, tx_buffer_{tx_buffer}, rx_ring_{rx_ring},
        timeout_{timeout} {}

  DmaPort(const DmaPort &) = delete;
  DmaPort &operator=(const DmaPort &) = delete;

  [[nodiscard]] auto configure(hal::serial::config configuration)
      -> result<hertz, error_type> {
    configured_ = false;
    if (!valid(configuration) || !timeout_.valid() ||
        (registers_.CR1 & cr1_enabled) != 0U) {
      return result<hertz, error_type>::failure(
          error{hal::serial::error_kind::configuration});
    }
    const std::uint32_t requested =
        static_cast<std::uint32_t>(configuration.baud_rate.value);
    const std::uint32_t divisor =
        (kernel_clock_hz_ + requested / 2U) / requested;
    if (divisor < 16U || divisor > 0xFFFFU) {
      return result<hertz, error_type>::failure(
          error{hal::serial::error_kind::configuration});
    }

    registers_.CR1 = 0U;
    registers_.CR2 = configuration.stops == hal::serial::stop_bits::two
                         ? cr2_two_stop_bits
                         : 0U;
    registers_.CR3 = flow_bits(configuration.flow);
    std::uint32_t cr1 = cr1_transmitter_enable | cr1_receiver_enable;
    if (configuration.parity_mode != hal::serial::parity::none) {
      cr1 |= cr1_parity_enable;
      if (configuration.parity_mode == hal::serial::parity::odd) {
        cr1 |= cr1_parity_odd;
      }
    }
    if (configuration.parity_mode == hal::serial::parity::none) {
      cr1 |= configuration.bits == hal::serial::data_bits::bits9
                 ? cr1_word_length_0
             : configuration.bits == hal::serial::data_bits::bits7
                 ? cr1_word_length_1
                 : 0U;
    } else if (configuration.bits == hal::serial::data_bits::bits8) {
      cr1 |= cr1_word_length_0;
    }
    registers_.BRR = divisor;
    registers_.ICR = icr_all_receive_errors;

    tx_dmamux_.CCR = tx_request_;
    rx_dmamux_.CCR = rx_request_;
    rx_stream_.CR = 0U;
    rx_stream_.PAR = pointer_word(&registers_.RDR);
    rx_stream_.M0AR = pointer_word(rx_ring_.data());
    rx_stream_.NDTR = static_cast<std::uint32_t>(RxCapacity);
    rx_stream_.FCR = 0U;
    rx_stream_.CR = dma_enable | dma_circular | dma_memory_increment |
                    dma_peripheral_byte | dma_memory_byte |
                    dma_half_transfer_interrupt | dma_transfer_complete_interrupt |
                    dma_transfer_error_interrupt;
    rx_dma_position_ = 0U;
    rx_produced_total_.store(0U, std::memory_order_release);
    rx_read_total_ = 0U;
    fault_.store(hal::serial::error_kind::other, std::memory_order_release);
    tx_busy_.store(false, std::memory_order_release);
    registers_.CR3 = registers_.CR3 | cr3_rx_dma_enable;
    registers_.CR1 = cr1 | cr1_enabled;
    configured_ = true;
    return result<hertz, error_type>::success(hertz{kernel_clock_hz_ / divisor});
  }

  [[nodiscard]] auto try_write(hal::span<const std::byte> data)
      -> result<std::size_t, error_type> {
    HAL_CORE_ASSERT(data.valid());
    if (!configured_) {
      return result<std::size_t, error_type>::failure(
          error{hal::serial::error_kind::configuration});
    }
    if (!data.valid()) {
      return result<std::size_t, error_type>::failure(
          error{hal::serial::error_kind::io});
    }
    const std::size_t count = data.size() < TxCapacity ? data.size() : TxCapacity;
    if (count == 0U) {
      return result<std::size_t, error_type>::success(0U);
    }
    bool expected = false;
    if (!tx_busy_.compare_exchange_strong(expected, true,
                                          std::memory_order_acq_rel,
                                          std::memory_order_relaxed)) {
      return result<std::size_t, error_type>::success(0U);
    }
    if (!stop_tx_dma()) {
      tx_busy_.store(false, std::memory_order_release);
      fault_.store(hal::serial::error_kind::timeout, std::memory_order_release);
      return result<std::size_t, error_type>::failure(
          error{hal::serial::error_kind::timeout});
    }
    for (std::size_t index = 0U; index < count; ++index) {
      tx_buffer_[index] = data[index];
    }
    tx_stream_.CR = 0U;
    tx_stream_.PAR = pointer_word(&registers_.TDR);
    tx_stream_.M0AR = pointer_word(tx_buffer_.data());
    tx_stream_.NDTR = static_cast<std::uint32_t>(count);
    tx_stream_.FCR = 0U;
    tx_stream_.CR = dma_enable | dma_direction_memory_to_peripheral |
                    dma_memory_increment | dma_peripheral_byte | dma_memory_byte |
                    dma_transfer_complete_interrupt | dma_transfer_error_interrupt;
    barrier();
    registers_.CR3 = registers_.CR3 | cr3_tx_dma_enable;
    return result<std::size_t, error_type>::success(count);
  }

  [[nodiscard]] auto try_read(hal::span<std::byte> data)
      -> result<std::size_t, error_type> {
    HAL_CORE_ASSERT(data.valid());
    if (!configured_) {
      return result<std::size_t, error_type>::failure(
          error{hal::serial::error_kind::configuration});
    }
    if (!data.valid()) {
      return result<std::size_t, error_type>::failure(
          error{hal::serial::error_kind::io});
    }
    const auto observed_fault = fault_.load(std::memory_order_acquire);
    if (observed_fault != hal::serial::error_kind::other) {
      const auto observed = observed_fault;
      fault_.store(hal::serial::error_kind::other, std::memory_order_release);
      return result<std::size_t, error_type>::failure(error{observed});
    }
    const hal::serial::error_kind receive_error = check_receive_error();
    if (receive_error != hal::serial::error_kind::other) {
      registers_.ICR = icr_all_receive_errors;
      return result<std::size_t, error_type>::failure(error{receive_error});
    }

    // The BSP dispatches the half/full DMA events to on_rx_dma_interrupt().
    // Keeping an absolute producer cursor lets us detect a complete ring
    // overrun; an NDTR-only modulo cursor cannot distinguish zero bytes from
    // one or more full wraps.
    const std::uint32_t producer =
        rx_produced_total_.load(std::memory_order_acquire);
    const std::uint32_t consumed = rx_read_total_;
    const std::uint32_t pending = producer - consumed;
    if (pending > RxCapacity) {
      rx_read_total_ = producer - static_cast<std::uint32_t>(RxCapacity);
      return result<std::size_t, error_type>::failure(
          error{hal::serial::error_kind::overrun});
    }
    barrier();
    const std::size_t available = static_cast<std::size_t>(pending);
    const std::size_t count = data.size() < available ? data.size() : available;
    for (std::size_t index = 0U; index < count; ++index) {
      data[index] = rx_ring_[(static_cast<std::size_t>(consumed) + index) %
                             RxCapacity];
    }
    rx_read_total_ += static_cast<std::uint32_t>(count);
    return result<std::size_t, error_type>::success(count);
  }

  [[nodiscard]] auto flush() -> result<void, error_type> {
    if (!configured_) {
      return result<void, error_type>::failure(
          error{hal::serial::error_kind::configuration});
    }
    const auto observed_fault = fault_.load(std::memory_order_acquire);
    if (observed_fault != hal::serial::error_kind::other) {
      return result<void, error_type>::failure(
          error{observed_fault});
    }
    if (tx_busy_.load(std::memory_order_acquire)) {
      for (std::uint32_t remaining = timeout_.iterations; remaining > 0U;
           --remaining) {
        if (!tx_busy_.load(std::memory_order_acquire)) {
          break;
        }
        if (tx_stream_.NDTR == 0U && stop_tx_dma()) {
          tx_busy_.store(false, std::memory_order_release);
          break;
        }
      }
      if (tx_busy_.load(std::memory_order_acquire)) {
        return result<void, error_type>::failure(
            error{hal::serial::error_kind::timeout});
      }
    }
    for (std::uint32_t remaining = timeout_.iterations; remaining > 0U;
         --remaining) {
      if ((registers_.ISR & isr_transmission_complete) != 0U) {
        return result<void, error_type>::success();
      }
    }
    return result<void, error_type>::failure(
        error{hal::serial::error_kind::timeout});
  }

  void on_tx_dma_interrupt(bool transfer_error = false) noexcept {
    tx_stream_.CR = tx_stream_.CR & ~dma_enable;
    registers_.CR3 = registers_.CR3 & ~cr3_tx_dma_enable;
    if (transfer_error) {
      fault_.store(hal::serial::error_kind::io, std::memory_order_release);
    }
    tx_busy_.store(false, std::memory_order_release);
  }

  void on_rx_dma_interrupt(bool transfer_error = false) noexcept {
    if (transfer_error) {
      fault_.store(hal::serial::error_kind::io, std::memory_order_release);
      rx_stream_.CR = rx_stream_.CR & ~dma_enable;
      registers_.CR3 = registers_.CR3 & ~cr3_rx_dma_enable;
      return;
    }
    const hal::serial::error_kind observed = check_receive_error();
    if (observed != hal::serial::error_kind::other) {
      fault_.store(observed, std::memory_order_release);
    }
    const std::size_t position =
        (RxCapacity - static_cast<std::size_t>(rx_stream_.NDTR % RxCapacity)) %
        RxCapacity;
    barrier();
    const std::size_t previous = rx_dma_position_;
    const std::size_t delta = position >= previous
                                  ? position - previous
                                  : RxCapacity - previous + position;
    if (delta != 0U) {
      rx_produced_total_.fetch_add(static_cast<std::uint32_t>(delta),
                                   std::memory_order_release);
      rx_dma_position_ = position;
    }
  }

  [[nodiscard]] bool tx_busy() const noexcept {
    return tx_busy_.load(std::memory_order_acquire);
  }
  [[nodiscard]] hal::serial::error_kind fault() const noexcept {
    return fault_.load(std::memory_order_acquire);
  }

private:
  static constexpr std::uint32_t cr1_enabled{1U << 0U};
  static constexpr std::uint32_t cr1_receiver_enable{1U << 2U};
  static constexpr std::uint32_t cr1_transmitter_enable{1U << 3U};
  static constexpr std::uint32_t cr1_parity_odd{1U << 9U};
  static constexpr std::uint32_t cr1_parity_enable{1U << 10U};
  static constexpr std::uint32_t cr1_word_length_0{1U << 12U};
  static constexpr std::uint32_t cr1_word_length_1{1U << 28U};
  static constexpr std::uint32_t cr2_two_stop_bits{2U << 12U};
  static constexpr std::uint32_t cr3_rts_enable{1U << 8U};
  static constexpr std::uint32_t cr3_cts_enable{1U << 9U};
  static constexpr std::uint32_t cr3_rx_dma_enable{1U << 6U};
  static constexpr std::uint32_t cr3_tx_dma_enable{1U << 7U};
  static constexpr std::uint32_t isr_parity_error{1U << 0U};
  static constexpr std::uint32_t isr_framing_error{1U << 1U};
  static constexpr std::uint32_t isr_noise_error{1U << 2U};
  static constexpr std::uint32_t isr_overrun_error{1U << 3U};
  static constexpr std::uint32_t isr_break_detected{1U << 8U};
  static constexpr std::uint32_t isr_transmission_complete{1U << 6U};
  static constexpr std::uint32_t icr_all_receive_errors{(1U << 0U) |
                                                        (1U << 1U) |
                                                        (1U << 2U) |
                                                        (1U << 3U) |
                                                        (1U << 8U)};
  static constexpr std::uint32_t dma_enable{1U << 0U};
  static constexpr std::uint32_t dma_direction_memory_to_peripheral{1U << 6U};
  static constexpr std::uint32_t dma_circular{1U << 8U};
  static constexpr std::uint32_t dma_memory_increment{1U << 10U};
  static constexpr std::uint32_t dma_peripheral_byte{1U << 11U};
  static constexpr std::uint32_t dma_memory_byte{1U << 13U};
  static constexpr std::uint32_t dma_transfer_error_interrupt{1U << 2U};
  static constexpr std::uint32_t dma_half_transfer_interrupt{1U << 3U};
  static constexpr std::uint32_t dma_transfer_complete_interrupt{1U << 4U};

  [[nodiscard]] bool valid(const hal::serial::config &configuration) const noexcept {
    if (kernel_clock_hz_ == 0U || configuration.baud_rate.value == 0U ||
        configuration.baud_rate.value > 0xFFFF'FFFFULL) {
      return false;
    }
    return configuration.bits != hal::serial::data_bits::bits9;
  }

  [[nodiscard]] static constexpr std::uint32_t
  flow_bits(hal::serial::flow_control flow) noexcept {
    return flow == hal::serial::flow_control::rts   ? cr3_rts_enable
           : flow == hal::serial::flow_control::cts ? cr3_cts_enable
           : flow == hal::serial::flow_control::rts_cts
               ? cr3_rts_enable | cr3_cts_enable
               : 0U;
  }

  [[nodiscard]] hal::serial::error_kind check_receive_error() const noexcept {
    const std::uint32_t status = registers_.ISR;
    if ((status & isr_overrun_error) != 0U) {
      return hal::serial::error_kind::overrun;
    }
    if ((status & isr_framing_error) != 0U) {
      return hal::serial::error_kind::framing;
    }
    if ((status & isr_parity_error) != 0U) {
      return hal::serial::error_kind::parity;
    }
    if ((status & isr_break_detected) != 0U) {
      return hal::serial::error_kind::break_detected;
    }
    if ((status & isr_noise_error) != 0U) {
      return hal::serial::error_kind::io;
    }
    return hal::serial::error_kind::other;
  }

  [[nodiscard]] static std::uint32_t pointer_word(const volatile void *pointer)
      noexcept {
    return static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(pointer));
  }

  [[nodiscard]] bool stop_tx_dma() noexcept {
    registers_.CR3 = registers_.CR3 & ~cr3_tx_dma_enable;
    tx_stream_.CR = tx_stream_.CR & ~dma_enable;
    for (std::uint32_t remaining = timeout_.iterations; remaining > 0U;
         --remaining) {
      if ((tx_stream_.CR & dma_enable) == 0U) {
        return true;
      }
    }
    return false;
  }

  static void barrier() noexcept {
#if defined(__arm__) || defined(__thumb__)
    __asm volatile("dsb 0xF" ::: "memory");
#else
    std::atomic_thread_fence(std::memory_order_seq_cst);
#endif
  }

  Registers &registers_;
  TxDmaStream &tx_stream_;
  RxDmaStream &rx_stream_;
  TxDmamuxChannel &tx_dmamux_;
  RxDmamuxChannel &rx_dmamux_;
  std::uint32_t kernel_clock_hz_{};
  std::uint8_t tx_request_{};
  std::uint8_t rx_request_{};
  std::array<std::byte, TxCapacity> &tx_buffer_;
  std::array<std::byte, RxCapacity> &rx_ring_;
  poll_budget timeout_{};
  std::size_t rx_dma_position_{};
  std::uint32_t rx_read_total_{};
  std::atomic<std::uint32_t> rx_produced_total_{};
  bool configured_{};
  std::atomic_bool tx_busy_{};
  std::atomic<hal::serial::error_kind> fault_{hal::serial::error_kind::other};
};

} // namespace hal::stm32h7::serial
