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

namespace hal::stm32g4::serial {

inline constexpr capability support{
    implementation_status::available,
    "direct USART polling and G4 channel-DMA implementations"};

class error {
public:
  explicit constexpr error(hal::serial::error_kind kind) noexcept : kind_{kind} {}
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
      -> result<hal::hertz, error_type> {
    configured_ = false;
    if (!valid(configuration) || !timeout_.valid()) {
      return failure_rate(hal::serial::error_kind::configuration);
    }
    const std::uint32_t baud =
        static_cast<std::uint32_t>(configuration.baud_rate.value);
    const std::uint64_t rounded =
        (static_cast<std::uint64_t>(kernel_clock_hz_) + baud / 2U) / baud;
    if (rounded == 0U || rounded > 0xFFFFU) {
      return failure_rate(hal::serial::error_kind::configuration);
    }

    registers_.CR1 = 0U;
    registers_.CR2 = 0U;
    registers_.CR3 = 0U;
    std::uint32_t cr1 = cr1_te | cr1_re;
    if (configuration.parity_mode != hal::serial::parity::none) {
      cr1 |= cr1_pce;
      if (configuration.parity_mode == hal::serial::parity::odd) {
        cr1 |= cr1_ps;
      }
    }
    // The portable contract transports bytes, so do not expose the hardware's
    // 9-bit word mode through this byte-oriented port.
    if (configuration.bits == hal::serial::data_bits::bits7) {
      cr1 |= cr1_m1;
    } else if (configuration.bits == hal::serial::data_bits::bits8 &&
               configuration.parity_mode != hal::serial::parity::none) {
      cr1 |= cr1_m0;
    }
    registers_.BRR = static_cast<std::uint32_t>(rounded);
    registers_.CR2 = configuration.stops == hal::serial::stop_bits::two
                         ? cr2_stop_2
                         : 0U;
    registers_.CR3 = flow_bits(configuration.flow);
    registers_.ICR = icr_receive_errors;
    registers_.CR1 = cr1 | cr1_ue;
    configured_ = true;
    return result<hal::hertz, error_type>::success(
        hal::hertz{kernel_clock_hz_ / static_cast<std::uint32_t>(rounded)});
  }

  [[nodiscard]] auto try_write(hal::span<const std::byte> data)
      -> result<std::size_t, error_type> {
    HAL_CORE_ASSERT(data.valid());
    if (!configured_) {
      return failure_size(hal::serial::error_kind::configuration);
    }
    if (!data.valid()) {
      return failure_size(hal::serial::error_kind::io);
    }
    std::size_t written = 0U;
    while (written < data.size() && (registers_.ISR & isr_txe) != 0U) {
      registers_.TDR = std::to_integer<std::uint8_t>(data[written]);
      ++written;
    }
    return result<std::size_t, error_type>::success(written);
  }

  [[nodiscard]] auto try_read(hal::span<std::byte> data)
      -> result<std::size_t, error_type> {
    HAL_CORE_ASSERT(data.valid());
    if (!configured_) {
      return failure_size(hal::serial::error_kind::configuration);
    }
    if (!data.valid()) {
      return failure_size(hal::serial::error_kind::io);
    }
    const auto receive_error = check_receive_error();
    if (receive_error != hal::serial::error_kind::other) {
      registers_.ICR = icr_receive_errors;
      return failure_size(receive_error);
    }
    std::size_t read = 0U;
    while (read < data.size() && (registers_.ISR & isr_rxne) != 0U) {
      data[read] = static_cast<std::byte>(registers_.RDR & 0xFFU);
      ++read;
    }
    return result<std::size_t, error_type>::success(read);
  }

  [[nodiscard]] auto flush() -> result<void, error_type> {
    if (!configured_) {
      return failure(hal::serial::error_kind::configuration);
    }
    return wait(isr_tc) ? result<void, error_type>::success()
                        : failure(hal::serial::error_kind::timeout);
  }

private:
  static constexpr std::uint32_t cr1_ue{1U << 0U};
  static constexpr std::uint32_t cr1_re{1U << 2U};
  static constexpr std::uint32_t cr1_te{1U << 3U};
  static constexpr std::uint32_t cr1_ps{1U << 9U};
  static constexpr std::uint32_t cr1_pce{1U << 10U};
  static constexpr std::uint32_t cr1_m0{1U << 12U};
  static constexpr std::uint32_t cr1_m1{1U << 28U};
  static constexpr std::uint32_t cr2_stop_2{2U << 12U};
  static constexpr std::uint32_t cr3_ctse{1U << 9U};
  static constexpr std::uint32_t cr3_rtse{1U << 8U};
  static constexpr std::uint32_t isr_pe{1U << 0U};
  static constexpr std::uint32_t isr_fe{1U << 1U};
  static constexpr std::uint32_t isr_ne{1U << 2U};
  static constexpr std::uint32_t isr_ore{1U << 3U};
  static constexpr std::uint32_t isr_rxne{1U << 5U};
  static constexpr std::uint32_t isr_tc{1U << 6U};
  static constexpr std::uint32_t isr_txe{1U << 7U};
  static constexpr std::uint32_t icr_receive_errors{(1U << 0U) | (1U << 1U) |
                                                     (1U << 2U) | (1U << 3U)};

  [[nodiscard]] static constexpr bool valid(hal::serial::config configuration)
      noexcept {
    return configuration.baud_rate.value > 0U &&
           (configuration.bits == hal::serial::data_bits::bits7 ||
            configuration.bits == hal::serial::data_bits::bits8);
  }

  [[nodiscard]] static constexpr std::uint32_t
  flow_bits(hal::serial::flow_control flow) noexcept {
    return flow == hal::serial::flow_control::rts   ? cr3_rtse
           : flow == hal::serial::flow_control::cts ? cr3_ctse
           : flow == hal::serial::flow_control::rts_cts ? cr3_rtse | cr3_ctse
                                                        : 0U;
  }

  [[nodiscard]] bool wait(std::uint32_t mask) const noexcept {
    for (std::uint32_t left = timeout_.iterations; left > 0U; --left) {
      if ((registers_.ISR & mask) != 0U) {
        return true;
      }
      if ((registers_.ISR & (isr_ore | isr_fe | isr_pe)) != 0U) {
        return false;
      }
    }
    return false;
  }

  [[nodiscard]] hal::serial::error_kind check_receive_error() const noexcept {
    const auto status = registers_.ISR;
    if ((status & isr_ore) != 0U) return hal::serial::error_kind::overrun;
    if ((status & isr_fe) != 0U) return hal::serial::error_kind::framing;
    if ((status & isr_pe) != 0U) return hal::serial::error_kind::parity;
    if ((status & isr_ne) != 0U) return hal::serial::error_kind::io;
    return hal::serial::error_kind::other;
  }

  [[nodiscard]] auto failure(hal::serial::error_kind kind)
      -> result<void, error_type> {
    return result<void, error_type>::failure(error{kind});
  }

  [[nodiscard]] static auto failure_size(hal::serial::error_kind kind)
      -> result<std::size_t, error_type> {
    return result<std::size_t, error_type>::failure(error{kind});
  }

  [[nodiscard]] static auto failure_rate(hal::serial::error_kind kind)
      -> result<hal::hertz, error_type> {
    return result<hal::hertz, error_type>::failure(error{kind});
  }

  Registers &registers_;
  std::uint32_t kernel_clock_hz_{};
  poll_budget timeout_{};
  bool configured_{};
};

template <class Registers, class TxChannel, class RxChannel,
          class TxDmamux, class RxDmamux, std::size_t TxCapacity = 1024U,
          std::size_t RxCapacity = 2048U>
class DmaPort {
  static_assert(TxCapacity > 0U && TxCapacity <= 65'535U);
  static_assert(RxCapacity > 1U && RxCapacity <= 65'535U &&
                (RxCapacity % 2U) == 0U);
  static_assert(std::atomic_bool::is_always_lock_free);
  static_assert(std::atomic<std::uint32_t>::is_always_lock_free);
  static_assert(std::atomic<hal::serial::error_kind>::is_always_lock_free);

public:
  using error_type = error;

  DmaPort(Registers &registers, TxChannel &tx, RxChannel &rx,
          TxDmamux &tx_mux, RxDmamux &rx_mux, std::uint32_t clock_hz,
          std::uint8_t tx_request, std::uint8_t rx_request,
          std::array<std::byte, TxCapacity> &tx_buffer,
          std::array<std::byte, RxCapacity> &rx_ring,
          poll_budget timeout = {100'000U}) noexcept
      : registers_{registers}, tx_{tx}, rx_{rx}, tx_mux_{tx_mux},
        rx_mux_{rx_mux}, clock_hz_{clock_hz}, tx_request_{tx_request},
        rx_request_{rx_request}, tx_buffer_{tx_buffer}, rx_ring_{rx_ring},
        timeout_{timeout} {}

  DmaPort(const DmaPort &) = delete;
  DmaPort &operator=(const DmaPort &) = delete;

  [[nodiscard]] auto configure(hal::serial::config configuration)
      -> result<hal::hertz, error_type> {
    configured_ = false;
    PollingPort<Registers> polling{registers_, clock_hz_, timeout_};
    const auto configured = polling.configure(configuration);
    if (!configured) {
      return configured;
    }
    stop_channel(tx_);
    stop_channel(rx_);
    tx_mux_.CCR = tx_request_;
    rx_mux_.CCR = rx_request_;
    configure_rx();
    barrier();
    registers_.CR3 = registers_.CR3 | cr3_dmar;
    rx_read_total_ = 0U;
    rx_produced_total_.store(0U, std::memory_order_release);
    tx_busy_.store(false, std::memory_order_relaxed);
    fault_.store(hal::serial::error_kind::other, std::memory_order_relaxed);
    configured_ = true;
    return configured;
  }

  [[nodiscard]] auto try_write(hal::span<const std::byte> data)
      -> result<std::size_t, error_type> {
    HAL_CORE_ASSERT(data.valid());
    if (!configured_ || !data.valid()) {
      return failure_size(hal::serial::error_kind::configuration);
    }
    const auto fault = fault_.load(std::memory_order_acquire);
    if (fault != hal::serial::error_kind::other) {
      return failure_size(fault);
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
    for (std::size_t i = 0U; i < count; ++i) tx_buffer_[i] = data[i];
    configure_tx(count);
    barrier();
    registers_.CR3 = registers_.CR3 | cr3_dmat;
    return result<std::size_t, error_type>::success(count);
  }

  [[nodiscard]] auto try_read(hal::span<std::byte> data)
      -> result<std::size_t, error_type> {
    HAL_CORE_ASSERT(data.valid());
    if (!configured_ || !data.valid()) {
      return failure_size(hal::serial::error_kind::configuration);
    }
    const auto fault = fault_.load(std::memory_order_acquire);
    if (fault != hal::serial::error_kind::other) {
      return failure_size(fault);
    }
    const std::uint32_t produced =
        rx_produced_total_.load(std::memory_order_acquire);
    const std::uint32_t pending = produced - rx_read_total_;
    if (pending > RxCapacity) {
      rx_read_total_ = produced - static_cast<std::uint32_t>(RxCapacity);
      return failure_size(hal::serial::error_kind::overrun);
    }
    const std::size_t count =
        static_cast<std::size_t>(pending) < data.size()
            ? static_cast<std::size_t>(pending)
            : data.size();
    for (std::size_t i = 0U; i < count; ++i) {
      data[i] = rx_ring_[(rx_read_total_ + i) % RxCapacity];
    }
    rx_read_total_ += static_cast<std::uint32_t>(count);
    return result<std::size_t, error_type>::success(count);
  }

  [[nodiscard]] auto flush() -> result<void, error_type> {
    if (!configured_) return failure(hal::serial::error_kind::configuration);
    for (std::uint32_t left = timeout_.iterations; left > 0U; --left) {
      if (!tx_busy_.load(std::memory_order_acquire) &&
          (registers_.ISR & isr_tc) != 0U) {
        return result<void, error_type>::success();
      }
    }
    return failure(hal::serial::error_kind::timeout);
  }

  void on_tx_dma_interrupt(bool transfer_error = false) noexcept {
    stop_channel(tx_);
    registers_.CR3 = registers_.CR3 & ~cr3_dmat;
    if (transfer_error) {
      fault_.store(hal::serial::error_kind::io, std::memory_order_release);
    }
    tx_busy_.store(false, std::memory_order_release);
  }

  void on_rx_dma_interrupt(bool transfer_error = false) noexcept {
    if (transfer_error) {
      fault_.store(hal::serial::error_kind::io, std::memory_order_release);
      stop_channel(rx_);
      registers_.CR3 = registers_.CR3 & ~cr3_dmar;
      return;
    }
    const auto receive_error = check_receive_error();
    if (receive_error != hal::serial::error_kind::other) {
      fault_.store(receive_error, std::memory_order_release);
    }
    rx_produced_total_.fetch_add(static_cast<std::uint32_t>(RxCapacity / 2U),
                                 std::memory_order_release);
  }

private:
  static constexpr std::uint32_t cr3_dmar{1U << 6U};
  static constexpr std::uint32_t cr3_dmat{1U << 7U};
  static constexpr std::uint32_t dma_enable{1U};
  static constexpr std::uint32_t dma_transfer_complete_interrupt{1U << 1U};
  static constexpr std::uint32_t dma_half_transfer_interrupt{1U << 2U};
  static constexpr std::uint32_t dma_transfer_error_interrupt{1U << 3U};
  static constexpr std::uint32_t dma_direction_memory_to_peripheral{1U << 4U};
  static constexpr std::uint32_t dma_circular{1U << 5U};
  static constexpr std::uint32_t dma_memory_increment{1U << 7U};
  static constexpr std::uint32_t isr_pe{1U << 0U};
  static constexpr std::uint32_t isr_fe{1U << 1U};
  static constexpr std::uint32_t isr_ne{1U << 2U};
  static constexpr std::uint32_t isr_ore{1U << 3U};
  static constexpr std::uint32_t isr_tc{1U << 6U};

  static void barrier() noexcept {
#if defined(__arm__) || defined(__thumb__)
    __asm volatile("dmb 0xF" ::: "memory");
#else
    std::atomic_thread_fence(std::memory_order_seq_cst);
#endif
  }

  [[nodiscard]] hal::serial::error_kind check_receive_error() const noexcept {
    const auto status = registers_.ISR;
    if ((status & isr_ore) != 0U) return hal::serial::error_kind::overrun;
    if ((status & isr_fe) != 0U) return hal::serial::error_kind::framing;
    if ((status & isr_pe) != 0U) return hal::serial::error_kind::parity;
    if ((status & isr_ne) != 0U) return hal::serial::error_kind::io;
    return hal::serial::error_kind::other;
  }

  template <class Channel>
  static void stop_channel(Channel &channel) noexcept {
    channel.CCR = channel.CCR & ~dma_enable;
  }

  template <class Channel>
  static void configure_channel(Channel &channel, std::uint32_t peripheral,
                                std::uint32_t memory, std::size_t count,
                                bool transmit, bool circular) noexcept {
    channel.CCR = 0U;
    channel.CPAR = peripheral;
    channel.CMAR = memory;
    channel.CNDTR = static_cast<std::uint32_t>(count);
    channel.CCR = dma_transfer_complete_interrupt |
                  dma_transfer_error_interrupt |
                  (circular ? dma_half_transfer_interrupt | dma_circular : 0U) |
                  (transmit ? dma_direction_memory_to_peripheral : 0U) |
                  (circular || transmit ? dma_memory_increment : 0U);
  }

  [[nodiscard]] static std::uint32_t pointer_word(const volatile void *pointer)
      noexcept {
    return static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(pointer));
  }

  void configure_rx() noexcept {
    configure_channel(rx_, pointer_word(&registers_.RDR),
                      pointer_word(rx_ring_.data()), RxCapacity, false, true);
    rx_.CCR = rx_.CCR | dma_enable;
  }

  void configure_tx(std::size_t count) noexcept {
    stop_channel(tx_);
    configure_channel(tx_, pointer_word(&registers_.TDR),
                      pointer_word(tx_buffer_.data()), count, true, false);
    tx_.CCR = tx_.CCR | dma_enable;
  }

  [[nodiscard]] auto failure(hal::serial::error_kind kind)
      -> result<void, error_type> {
    return result<void, error_type>::failure(error{kind});
  }

  [[nodiscard]] static auto failure_size(hal::serial::error_kind kind)
      -> result<std::size_t, error_type> {
    return result<std::size_t, error_type>::failure(error{kind});
  }

  Registers &registers_;
  TxChannel &tx_;
  RxChannel &rx_;
  TxDmamux &tx_mux_;
  RxDmamux &rx_mux_;
  std::uint32_t clock_hz_{};
  std::uint8_t tx_request_{};
  std::uint8_t rx_request_{};
  std::array<std::byte, TxCapacity> &tx_buffer_;
  std::array<std::byte, RxCapacity> &rx_ring_;
  poll_budget timeout_{};
  std::uint32_t rx_read_total_{};
  std::atomic<std::uint32_t> rx_produced_total_{};
  std::atomic_bool tx_busy_{};
  std::atomic<hal::serial::error_kind> fault_{hal::serial::error_kind::other};
  bool configured_{};
};

struct no_dma {};

} // namespace hal::stm32g4::serial
