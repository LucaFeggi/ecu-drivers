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
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <hal/contracts/spi_bus.hpp>
#include "support.hpp"

namespace hal::stm32g4::spi {

inline constexpr capability support{
    implementation_status::available,
    "direct G4 SPI polling and channel-DMA master transfers"};

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

  PollingBus8(Registers &registers, std::uint32_t kernel_clock_hz,
              poll_budget timeout) noexcept
      : registers_{registers}, kernel_clock_hz_{kernel_clock_hz},
        timeout_{timeout} {}

  PollingBus8(const PollingBus8 &) = delete;
  PollingBus8 &operator=(const PollingBus8 &) = delete;

  [[nodiscard]] auto configure(hal::spi::config8 configuration)
      -> result<hal::hertz, error_type> {
    if (configuration.max_frequency.value == 0U || kernel_clock_hz_ == 0U ||
        !timeout_.valid() || (registers_.CR1 & cr1_spe) != 0U) {
      return failure_rate(hal::spi::error_kind::configuration);
    }
    const std::uint32_t requested =
        configuration.max_frequency.value > 0xFFFF'FFFFULL
            ? 0xFFFF'FFFFU
            : static_cast<std::uint32_t>(configuration.max_frequency.value);
    std::uint32_t divisor = 2U;
    std::uint32_t baud_code = 0U;
    while (kernel_clock_hz_ / divisor > requested && divisor < 256U) {
      divisor *= 2U;
      ++baud_code;
    }
    if (kernel_clock_hz_ / divisor > requested) {
      return failure_rate(hal::spi::error_kind::configuration);
    }
    std::uint32_t cr1 = cr1_mstr | cr1_ssm | cr1_ssi |
                        (baud_code << cr1_br_position);
    if (configuration.order == hal::spi::bit_order::lsb_first) {
      cr1 |= cr1_lsb_first;
    }
    if (configuration.clock_mode == hal::spi::mode::mode1 ||
        configuration.clock_mode == hal::spi::mode::mode3) {
      cr1 |= cr1_cpha;
    }
    if (configuration.clock_mode == hal::spi::mode::mode2 ||
        configuration.clock_mode == hal::spi::mode::mode3) {
      cr1 |= cr1_cpol;
    }
    registers_.CR1 = 0U;
    registers_.CR2 = cr2_eight_bit | cr2_frxth;
    registers_.CR1 = cr1;
    read_fill_ = configuration.read_fill;
    configured_ = true;
    return result<hal::hertz, error_type>::success(
        hal::hertz{kernel_clock_hz_ / divisor});
  }

  [[nodiscard]] auto write(hal::span<const std::byte> data)
      -> result<void, error_type> {
    return exchange(data, {}, std::byte{0U});
  }

  [[nodiscard]] auto read(hal::span<std::byte> data, std::byte fill)
      -> result<void, error_type> {
    return exchange({}, data, fill);
  }

  [[nodiscard]] auto transfer(hal::span<const std::byte> tx,
                               hal::span<std::byte> rx)
      -> result<void, error_type> {
    return exchange(tx, rx, read_fill_);
  }

  [[nodiscard]] auto flush() -> result<void, error_type> {
    if (!configured_ || (registers_.CR1 & cr1_spe) == 0U) {
      return result<void, error_type>::success();
    }
    return wait(sr_not_busy) ? result<void, error_type>::success()
                             : failure(hal::spi::error_kind::timeout);
  }

private:
  static constexpr std::uint32_t cr1_cpha{1U << 0U};
  static constexpr std::uint32_t cr1_cpol{1U << 1U};
  static constexpr std::uint32_t cr1_mstr{1U << 2U};
  static constexpr std::uint32_t cr1_br_position{3U};
  static constexpr std::uint32_t cr1_spe{1U << 6U};
  static constexpr std::uint32_t cr1_lsb_first{1U << 7U};
  static constexpr std::uint32_t cr1_ssi{1U << 8U};
  static constexpr std::uint32_t cr1_ssm{1U << 9U};
  static constexpr std::uint32_t cr2_eight_bit{7U << 8U};
  static constexpr std::uint32_t cr2_frxth{1U << 12U};
  static constexpr std::uint32_t sr_rxne{1U << 0U};
  static constexpr std::uint32_t sr_txe{1U << 1U};
  static constexpr std::uint32_t sr_not_busy{1U << 7U};
  static constexpr std::uint32_t sr_error{(1U << 3U) | (1U << 4U) |
                                          (1U << 5U) | (1U << 6U)};

  [[nodiscard]] bool wait(std::uint32_t mask) const noexcept {
    for (std::uint32_t left = timeout_.iterations; left > 0U; --left) {
      const auto status = registers_.SR;
      if ((status & mask) != 0U) return true;
      if ((status & sr_error) != 0U) return false;
    }
    return false;
  }

  [[nodiscard]] auto exchange(hal::span<const std::byte> tx,
                              hal::span<std::byte> rx, std::byte fill)
      -> result<void, error_type> {
    HAL_CORE_ASSERT(tx.valid());
    HAL_CORE_ASSERT(rx.valid());
    if (!configured_ || !tx.valid() || !rx.valid()) {
      return failure(hal::spi::error_kind::configuration);
    }
    const std::size_t count = tx.size() > rx.size() ? tx.size() : rx.size();
    if (count == 0U) return result<void, error_type>::success();
    registers_.CR1 = registers_.CR1 | cr1_spe;
    auto &data = *reinterpret_cast<volatile std::uint8_t *>(&registers_.DR);
    for (std::size_t i = 0U; i < count; ++i) {
      if (!wait(sr_txe)) return abort(hal::spi::error_kind::timeout);
      data = static_cast<std::uint8_t>(i < tx.size() ? tx[i] : fill);
      if (!wait(sr_rxne)) return abort(hal::spi::error_kind::timeout);
      const std::byte received{data};
      if (i < rx.size()) rx[i] = received;
    }
    if (!wait(sr_not_busy)) return abort(hal::spi::error_kind::timeout);
    registers_.CR1 = registers_.CR1 & ~cr1_spe;
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto abort(hal::spi::error_kind kind)
      -> result<void, error_type> {
    registers_.CR1 = registers_.CR1 & ~cr1_spe;
    return failure(kind);
  }

  [[nodiscard]] auto failure(hal::spi::error_kind kind)
      -> result<void, error_type> {
    return result<void, error_type>::failure(error{kind});
  }

  [[nodiscard]] static auto failure_rate(hal::spi::error_kind kind)
      -> result<hal::hertz, error_type> {
    return result<hal::hertz, error_type>::failure(error{kind});
  }

  Registers &registers_;
  std::uint32_t kernel_clock_hz_{};
  poll_budget timeout_{};
  std::byte read_fill_{std::byte{0xFFU}};
  bool configured_{};
};

template <class Registers, class TxChannel, class RxChannel,
          class TxDmamux, class RxDmamux, std::size_t ScratchCapacity = 4096U>
class DmaBus8 {
  static_assert(ScratchCapacity > 0U && ScratchCapacity <= 65'535U);

public:
  using error_type = error;
  static constexpr bool supports_lsb_first{true};

  DmaBus8(Registers &registers, TxChannel &tx, RxChannel &rx,
          TxDmamux &tx_mux, RxDmamux &rx_mux, std::uint32_t clock_hz,
          poll_budget timeout, std::uint8_t tx_request,
          std::uint8_t rx_request,
          std::array<std::byte, ScratchCapacity> &tx_scratch,
          std::array<std::byte, ScratchCapacity> &rx_scratch) noexcept
      : registers_{registers}, tx_{tx}, rx_{rx}, tx_mux_{tx_mux},
        rx_mux_{rx_mux}, clock_hz_{clock_hz}, timeout_{timeout},
        tx_request_{tx_request}, rx_request_{rx_request},
        tx_scratch_{tx_scratch}, rx_scratch_{rx_scratch} {}

  DmaBus8(const DmaBus8 &) = delete;
  DmaBus8 &operator=(const DmaBus8 &) = delete;

  [[nodiscard]] auto configure(hal::spi::config8 configuration)
      -> result<hal::hertz, error_type> {
    configured_ = false;
    tx_.CCR = tx_.CCR & ~dma_enable;
    rx_.CCR = rx_.CCR & ~dma_enable;
    PollingBus8<Registers> polling{registers_, clock_hz_, timeout_};
    const auto result = polling.configure(configuration);
    if (!result) return result;
    read_fill_ = configuration.read_fill;
    configured_ = true;
    return result;
  }

  [[nodiscard]] auto write(hal::span<const std::byte> data)
      -> result<void, error_type> {
    return exchange(data, {}, std::byte{0U});
  }
  [[nodiscard]] auto read(hal::span<std::byte> data, std::byte fill)
      -> result<void, error_type> {
    return exchange({}, data, fill);
  }
  [[nodiscard]] auto transfer(hal::span<const std::byte> tx,
                              hal::span<std::byte> rx)
      -> result<void, error_type> {
    return exchange(tx, rx, read_fill_);
  }
  [[nodiscard]] auto flush() -> result<void, error_type> {
    return !configured_ || (registers_.CR1 & cr1_spe) == 0U
               ? result<void, error_type>::success()
               : wait(sr_not_busy)
                     ? result<void, error_type>::success()
                     : failure(hal::spi::error_kind::timeout);
  }

private:
  static constexpr std::uint32_t cr1_spe{1U << 6U};
  static constexpr std::uint32_t sr_not_busy{1U << 7U};
  static constexpr std::uint32_t sr_error{(1U << 3U) | (1U << 4U) |
                                          (1U << 5U) | (1U << 6U)};
  static constexpr std::uint32_t cr2_rxdma{1U};
  static constexpr std::uint32_t cr2_txdma{1U << 1U};
  static constexpr std::uint32_t dma_enable{1U};
  static constexpr std::uint32_t dma_dir{1U << 4U};
  static constexpr std::uint32_t dma_minc{1U << 7U};

  [[nodiscard]] bool wait(std::uint32_t mask) const noexcept {
    for (std::uint32_t left = timeout_.iterations; left > 0U; --left) {
      const auto status = registers_.SR;
      if ((status & mask) != 0U) return true;
      if ((status & sr_error) != 0U) return false;
    }
    return false;
  }

  template <class Channel>
  static void setup(Channel &channel, std::uint32_t peripheral,
                    std::uint32_t memory, std::size_t count,
                    bool transmit) noexcept {
    channel.CCR = 0U;
    channel.CPAR = peripheral;
    channel.CMAR = memory;
    channel.CNDTR = static_cast<std::uint32_t>(count);
    channel.CCR = dma_minc | (transmit ? dma_dir : 0U);
  }

  [[nodiscard]] static std::uint32_t pointer_word(const volatile void *pointer)
      noexcept {
    return static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(pointer));
  }

  [[nodiscard]] auto exchange(hal::span<const std::byte> tx,
                              hal::span<std::byte> rx, std::byte fill)
      -> result<void, error_type> {
    HAL_CORE_ASSERT(tx.valid());
    HAL_CORE_ASSERT(rx.valid());
    if (!configured_ || !tx.valid() || !rx.valid()) {
      return failure(hal::spi::error_kind::configuration);
    }
    const std::size_t count = tx.size() > rx.size() ? tx.size() : rx.size();
    if (count > ScratchCapacity) {
      return failure(hal::spi::error_kind::configuration);
    }
    if (count == 0U) return result<void, error_type>::success();
    for (std::size_t i = 0U; i < count; ++i) {
      tx_scratch_[i] = i < tx.size() ? tx[i] : fill;
    }
    tx_.CCR = tx_.CCR & ~dma_enable;
    rx_.CCR = rx_.CCR & ~dma_enable;
    tx_mux_.CCR = tx_request_;
    rx_mux_.CCR = rx_request_;
    tx_.CCR = 0U;
    rx_.CCR = 0U;
    setup(tx_, pointer_word(&registers_.DR), pointer_word(tx_scratch_.data()),
          count, true);
    setup(rx_, pointer_word(&registers_.DR), pointer_word(rx_scratch_.data()),
          count, false);
    registers_.CR2 = registers_.CR2 | cr2_rxdma | cr2_txdma;
    tx_.CCR = tx_.CCR | dma_enable;
    rx_.CCR = rx_.CCR | dma_enable;
    barrier();
    registers_.CR1 = registers_.CR1 | cr1_spe;
    if (!wait_dma() || !wait(sr_not_busy)) {
      stop_dma();
      return failure(hal::spi::error_kind::timeout);
    }
    stop_dma();
    for (std::size_t i = 0U; i < rx.size(); ++i) rx[i] = rx_scratch_[i];
    return result<void, error_type>::success();
  }

  [[nodiscard]] bool wait_dma() const noexcept {
    for (std::uint32_t left = timeout_.iterations; left > 0U; --left) {
      if (tx_.CNDTR == 0U && rx_.CNDTR == 0U) return true;
    }
    return false;
  }

  static void barrier() noexcept {
#if defined(__arm__) || defined(__thumb__)
    __asm volatile("dmb 0xF" ::: "memory");
#else
    std::atomic_thread_fence(std::memory_order_seq_cst);
#endif
  }

  void stop_dma() noexcept {
    tx_.CCR = tx_.CCR & ~dma_enable;
    rx_.CCR = rx_.CCR & ~dma_enable;
    registers_.CR2 = registers_.CR2 & ~(cr2_rxdma | cr2_txdma);
    registers_.CR1 = registers_.CR1 & ~cr1_spe;
  }

  [[nodiscard]] auto failure(hal::spi::error_kind kind)
      -> result<void, error_type> {
    return result<void, error_type>::failure(error{kind});
  }

  Registers &registers_;
  TxChannel &tx_;
  RxChannel &rx_;
  TxDmamux &tx_mux_;
  RxDmamux &rx_mux_;
  std::uint32_t clock_hz_{};
  poll_budget timeout_{};
  std::uint8_t tx_request_{};
  std::uint8_t rx_request_{};
  std::array<std::byte, ScratchCapacity> &tx_scratch_;
  std::array<std::byte, ScratchCapacity> &rx_scratch_;
  std::byte read_fill_{std::byte{0xFFU}};
  bool configured_{};
};

} // namespace hal::stm32g4::spi
