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

namespace hal::stm32h7::spi {

inline constexpr capability support{
    implementation_status::available,
    "bounded polling STM32H7 SPI master for 8-bit full-duplex transfers"};

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
      -> result<hertz, error_type> {
    if (configuration.max_frequency.value == 0U ||
        configuration.max_frequency.value > 0xFFFF'FFFFULL ||
        kernel_clock_hz_ == 0U || !timeout_.valid()) {
      return result<hertz, error_type>::failure(
          error{hal::spi::error_kind::configuration});
    }
    if ((registers_.CR1 & cr1_enabled) != 0U) {
      return result<hertz, error_type>::failure(
          error{hal::spi::error_kind::busy});
    }

    const std::uint32_t maximum =
        static_cast<std::uint32_t>(configuration.max_frequency.value);
    std::uint32_t divisor = 2U;
    std::uint32_t baud_code = 0U;
    while ((kernel_clock_hz_ / divisor) > maximum && divisor < 256U) {
      divisor *= 2U;
      ++baud_code;
    }
    if ((kernel_clock_hz_ / divisor) > maximum) {
      return result<hertz, error_type>::failure(
          error{hal::spi::error_kind::configuration});
    }

    registers_.CFG1 = cfg1_eight_bit_data | (baud_code << cfg1_baud_position);
    std::uint32_t cfg2 = cfg2_alternate_function_control | cfg2_master |
                         cfg2_software_slave_management;
    if (configuration.order == hal::spi::bit_order::lsb_first) {
      cfg2 |= cfg2_lsb_first;
    }
    if (configuration.clock_mode == hal::spi::mode::mode1 ||
        configuration.clock_mode == hal::spi::mode::mode3) {
      cfg2 |= cfg2_clock_phase;
    }
    if (configuration.clock_mode == hal::spi::mode::mode2 ||
        configuration.clock_mode == hal::spi::mode::mode3) {
      cfg2 |= cfg2_clock_polarity;
    }
    registers_.CFG2 = cfg2;
    registers_.CR1 = cr1_internal_slave_select;
    read_fill_ = configuration.read_fill;
    configured_ = true;
    return result<hertz, error_type>::success(
        hertz{kernel_clock_hz_ / divisor});
  }

  [[nodiscard]] auto write(span<const std::byte> data) -> result<void, error_type> {
    return exchange(data, {}, std::byte{0U});
  }

  [[nodiscard]] auto read(span<std::byte> data, std::byte fill)
      -> result<void, error_type> {
    return exchange({}, data, fill);
  }

  [[nodiscard]] auto transfer(span<const std::byte> tx, span<std::byte> rx)
      -> result<void, error_type> {
    return exchange(tx, rx, read_fill_);
  }

  [[nodiscard]] auto flush() -> result<void, error_type> {
    if ((registers_.CR1 & cr1_enabled) == 0U) {
      return result<void, error_type>::success();
    }
    if (!wait_set(sr_transmission_complete)) {
      return result<void, error_type>::failure(
          error{hal::spi::error_kind::timeout});
    }
    return result<void, error_type>::success();
  }

private:
  static constexpr std::uint32_t cr1_enabled{1U << 0U};
  static constexpr std::uint32_t cr1_master_start{1U << 9U};
  static constexpr std::uint32_t cr1_internal_slave_select{1U << 12U};
  static constexpr std::uint32_t cfg1_eight_bit_data{7U};
  static constexpr std::uint32_t cfg1_baud_position{28U};
  static constexpr std::uint32_t cfg2_alternate_function_control{1U << 31U};
  static constexpr std::uint32_t cfg2_master{1U << 22U};
  static constexpr std::uint32_t cfg2_lsb_first{1U << 23U};
  static constexpr std::uint32_t cfg2_clock_phase{1U << 24U};
  static constexpr std::uint32_t cfg2_clock_polarity{1U << 25U};
  static constexpr std::uint32_t cfg2_software_slave_management{1U << 26U};
  static constexpr std::uint32_t sr_receive_available{1U << 0U};
  static constexpr std::uint32_t sr_transmit_space{1U << 1U};
  static constexpr std::uint32_t sr_end_of_transfer{1U << 3U};
  static constexpr std::uint32_t sr_underrun{1U << 5U};
  static constexpr std::uint32_t sr_overrun{1U << 6U};
  static constexpr std::uint32_t sr_frame_error{1U << 8U};
  static constexpr std::uint32_t sr_transmission_complete{1U << 12U};
  static constexpr std::uint32_t ifcr_all{(1U << 3U) | (1U << 4U) | (1U << 5U) |
                                          (1U << 6U) | (1U << 8U)};
  static constexpr std::uint32_t maximum_transfer{0xFFFFU};

  [[nodiscard]] bool wait_set(std::uint32_t flag) const noexcept {
    for (std::uint32_t remaining = timeout_.iterations; remaining > 0U;
         --remaining) {
      if ((registers_.SR & flag) != 0U) {
        return true;
      }
      if ((registers_.SR & (sr_underrun | sr_overrun | sr_frame_error)) != 0U) {
        return false;
      }
    }
    return false;
  }

  [[nodiscard]] auto exchange(span<const std::byte> tx, span<std::byte> rx, std::byte fill)
      -> result<void, error_type> {
    HAL_CORE_ASSERT(tx.valid());
    HAL_CORE_ASSERT(rx.valid());
    if (!configured_ || !tx.valid() || !rx.valid()) {
      return result<void, error_type>::failure(
          error{hal::spi::error_kind::configuration});
    }
    const std::size_t size = tx.size() > rx.size() ? tx.size() : rx.size();
    if (size > maximum_transfer) {
      return result<void, error_type>::failure(
          error{hal::spi::error_kind::configuration});
    }
    if (size == 0U) {
      return result<void, error_type>::success();
    }

    registers_.IFCR = ifcr_all;
    registers_.CR2 = static_cast<std::uint32_t>(size);
    registers_.CR1 = registers_.CR1 | cr1_enabled;
    registers_.CR1 = registers_.CR1 | cr1_master_start;

    auto &transmit =
        *reinterpret_cast<volatile std::uint8_t *>(&registers_.TXDR);
    auto &receive =
        *reinterpret_cast<volatile std::uint8_t *>(&registers_.RXDR);
    for (std::size_t index = 0U; index < size; ++index) {
      if (!wait_set(sr_transmit_space)) {
        abort_transfer();
        return result<void, error_type>::failure(
            error{hal::spi::error_kind::timeout});
      }
      const std::byte outgoing = index < tx.size() ? tx[index] : fill;
      transmit = static_cast<std::uint8_t>(outgoing);
      if (!wait_set(sr_receive_available)) {
        abort_transfer();
        return result<void, error_type>::failure(
            error{hal::spi::error_kind::timeout});
      }
      const std::byte received{receive};
      if (index < rx.size()) {
        rx[index] = received;
      }
    }

    if (!wait_set(sr_end_of_transfer)) {
      abort_transfer();
      return result<void, error_type>::failure(
          error{hal::spi::error_kind::timeout});
    }
    registers_.IFCR = ifcr_all;
    registers_.CR1 = registers_.CR1 & ~cr1_enabled;
    return result<void, error_type>::success();
  }

  void abort_transfer() noexcept {
    registers_.CR1 = registers_.CR1 & ~cr1_enabled;
    registers_.IFCR = ifcr_all;
  }

  Registers &registers_;
  std::uint32_t kernel_clock_hz_{};
  poll_budget timeout_{};
  std::byte read_fill_{std::byte{0xFFU}};
  bool configured_{};
};

} // namespace hal::stm32h7::spi

namespace hal::stm32h7::spi {

// Synchronous hal-core operations are completed before returning. This class
// moves the bytes through DMA and performs only bounded completion checks; it
// never spins once per byte. Chip-select remains outside this class and is
// normally provided by hal::spi::StaticDevice8.
template <class Registers, class TxDmaStream, class RxDmaStream,
          class TxDmamuxChannel, class RxDmamuxChannel,
          std::size_t ScratchCapacity = 4096U>
class DmaBus8 {
  static_assert(ScratchCapacity > 0U);

public:
  using error_type = error;
  static constexpr bool supports_lsb_first{true};

  DmaBus8(Registers &registers, TxDmaStream &tx_stream,
          RxDmaStream &rx_stream, TxDmamuxChannel &tx_dmamux,
          RxDmamuxChannel &rx_dmamux, std::uint32_t kernel_clock_hz,
          poll_budget timeout, std::uint8_t tx_request,
          std::uint8_t rx_request,
          std::array<std::byte, ScratchCapacity> &tx_scratch,
          std::array<std::byte, ScratchCapacity> &rx_scratch) noexcept
      : registers_{registers}, tx_stream_{tx_stream}, rx_stream_{rx_stream},
        tx_dmamux_{tx_dmamux}, rx_dmamux_{rx_dmamux},
        kernel_clock_hz_{kernel_clock_hz}, timeout_{timeout},
        tx_request_{tx_request}, rx_request_{rx_request},
        tx_scratch_{tx_scratch}, rx_scratch_{rx_scratch} {}

  DmaBus8(const DmaBus8 &) = delete;
  DmaBus8 &operator=(const DmaBus8 &) = delete;

  [[nodiscard]] auto configure(hal::spi::config8 configuration)
      -> result<hertz, error_type> {
    if (configuration.max_frequency.value == 0U ||
        configuration.max_frequency.value > 0xFFFF'FFFFULL ||
        kernel_clock_hz_ == 0U || !timeout_.valid() ||
        (registers_.CR1 & cr1_enabled) != 0U) {
      return result<hertz, error_type>::failure(
          error{hal::spi::error_kind::configuration});
    }
    const std::uint32_t maximum =
        static_cast<std::uint32_t>(configuration.max_frequency.value);
    std::uint32_t divisor = 2U;
    std::uint32_t baud_code = 0U;
    while ((kernel_clock_hz_ / divisor) > maximum && divisor < 256U) {
      divisor *= 2U;
      ++baud_code;
    }
    if ((kernel_clock_hz_ / divisor) > maximum) {
      return result<hertz, error_type>::failure(
          error{hal::spi::error_kind::configuration});
    }
    registers_.CFG1 = cfg1_eight_bit_data | (baud_code << cfg1_baud_position);
    std::uint32_t cfg2 = cfg2_alternate_function_control | cfg2_master |
                         cfg2_software_slave_management;
    if (configuration.order == hal::spi::bit_order::lsb_first) {
      cfg2 |= cfg2_lsb_first;
    }
    if (configuration.clock_mode == hal::spi::mode::mode1 ||
        configuration.clock_mode == hal::spi::mode::mode3) {
      cfg2 |= cfg2_clock_phase;
    }
    if (configuration.clock_mode == hal::spi::mode::mode2 ||
        configuration.clock_mode == hal::spi::mode::mode3) {
      cfg2 |= cfg2_clock_polarity;
    }
    registers_.CFG2 = cfg2;
    registers_.CR1 = cr1_internal_slave_select;
    read_fill_ = configuration.read_fill;
    configured_ = true;
    return result<hertz, error_type>::success(
        hertz{kernel_clock_hz_ / divisor});
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
    if ((registers_.CR1 & cr1_enabled) == 0U) {
      return result<void, error_type>::success();
    }
    return wait_flag(sr_end_of_transfer)
               ? result<void, error_type>::success()
               : result<void, error_type>::failure(
                     error{hal::spi::error_kind::timeout});
  }

private:
  static constexpr std::uint32_t cr1_enabled{1U << 0U};
  static constexpr std::uint32_t cr1_master_start{1U << 9U};
  static constexpr std::uint32_t cr1_internal_slave_select{1U << 12U};
  static constexpr std::uint32_t cfg1_eight_bit_data{7U};
  static constexpr std::uint32_t cfg1_baud_position{28U};
  static constexpr std::uint32_t cfg1_rx_dma_enable{1U << 14U};
  static constexpr std::uint32_t cfg1_tx_dma_enable{1U << 15U};
  static constexpr std::uint32_t cfg2_alternate_function_control{1U << 31U};
  static constexpr std::uint32_t cfg2_master{1U << 22U};
  static constexpr std::uint32_t cfg2_lsb_first{1U << 23U};
  static constexpr std::uint32_t cfg2_clock_phase{1U << 24U};
  static constexpr std::uint32_t cfg2_clock_polarity{1U << 25U};
  static constexpr std::uint32_t cfg2_software_slave_management{1U << 26U};
  static constexpr std::uint32_t sr_end_of_transfer{1U << 3U};
  static constexpr std::uint32_t sr_underrun{1U << 5U};
  static constexpr std::uint32_t sr_overrun{1U << 6U};
  static constexpr std::uint32_t sr_frame_error{1U << 8U};
  static constexpr std::uint32_t ifcr_all{(1U << 3U) | (1U << 4U) | (1U << 5U) |
                                          (1U << 6U) | (1U << 8U)};
  static constexpr std::uint32_t dma_enable{1U << 0U};
  static constexpr std::uint32_t dma_direction_memory_to_peripheral{1U << 6U};
  static constexpr std::uint32_t dma_memory_increment{1U << 10U};
  static constexpr std::uint32_t dma_peripheral_byte{1U << 11U};
  static constexpr std::uint32_t dma_memory_byte{1U << 13U};

  static void barrier() noexcept {
#if defined(__arm__) || defined(__thumb__)
    __asm volatile("dsb 0xF" ::: "memory");
#else
    std::atomic_thread_fence(std::memory_order_seq_cst);
#endif
  }

  [[nodiscard]] bool wait_flag(std::uint32_t flag) const noexcept {
    for (std::uint32_t remaining = timeout_.iterations; remaining > 0U;
         --remaining) {
      if ((registers_.SR & flag) != 0U) {
        return true;
      }
      if ((registers_.SR & (sr_underrun | sr_overrun | sr_frame_error)) != 0U) {
        return false;
      }
    }
    return false;
  }

  [[nodiscard]] static std::uint32_t pointer_word(const volatile void *pointer)
      noexcept {
    return static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(pointer));
  }

  [[nodiscard]] bool stop_dma() noexcept {
    registers_.CFG1 = registers_.CFG1 & ~(cfg1_tx_dma_enable | cfg1_rx_dma_enable);
    registers_.CR1 = registers_.CR1 & ~cr1_enabled;
    tx_stream_.CR = tx_stream_.CR & ~dma_enable;
    rx_stream_.CR = rx_stream_.CR & ~dma_enable;
    for (std::uint32_t remaining = timeout_.iterations; remaining > 0U;
         --remaining) {
      if ((tx_stream_.CR & dma_enable) == 0U &&
          (rx_stream_.CR & dma_enable) == 0U) {
        registers_.IFCR = ifcr_all;
        return true;
      }
    }
    registers_.IFCR = ifcr_all;
    return false;
  }

  [[nodiscard]] bool wait_dma() const noexcept {
    for (std::uint32_t remaining = timeout_.iterations; remaining > 0U;
         --remaining) {
      if (tx_stream_.NDTR == 0U && rx_stream_.NDTR == 0U) {
        return true;
      }
    }
    return false;
  }

  [[nodiscard]] auto exchange(hal::span<const std::byte> tx,
                              hal::span<std::byte> rx, std::byte fill)
      -> result<void, error_type> {
    HAL_CORE_ASSERT(tx.valid());
    HAL_CORE_ASSERT(rx.valid());
    if (!configured_ || !tx.valid() || !rx.valid()) {
      return result<void, error_type>::failure(
          error{hal::spi::error_kind::configuration});
    }
    const std::size_t size = tx.size() > rx.size() ? tx.size() : rx.size();
    if (size > ScratchCapacity || size > 0xFFFFU) {
      return result<void, error_type>::failure(
          error{hal::spi::error_kind::configuration});
    }
    if (size == 0U) {
      return result<void, error_type>::success();
    }
    for (std::size_t index = 0U; index < size; ++index) {
      tx_scratch_[index] = index < tx.size() ? tx[index] : fill;
    }

    if (!stop_dma()) {
      return result<void, error_type>::failure(
          error{hal::spi::error_kind::timeout});
    }
    tx_dmamux_.CCR = tx_request_;
    rx_dmamux_.CCR = rx_request_;
    tx_stream_.CR = 0U;
    rx_stream_.CR = 0U;
    tx_stream_.PAR = pointer_word(&registers_.TXDR);
    tx_stream_.M0AR = pointer_word(tx_scratch_.data());
    tx_stream_.NDTR = static_cast<std::uint32_t>(size);
    tx_stream_.FCR = 0U;
    rx_stream_.PAR = pointer_word(&registers_.RXDR);
    rx_stream_.M0AR = pointer_word(rx_scratch_.data());
    rx_stream_.NDTR = static_cast<std::uint32_t>(size);
    rx_stream_.FCR = 0U;
    tx_stream_.CR = dma_enable | dma_direction_memory_to_peripheral |
                    dma_memory_increment | dma_peripheral_byte | dma_memory_byte;
    rx_stream_.CR = dma_enable | dma_memory_increment | dma_peripheral_byte |
                    dma_memory_byte;
    barrier();
    registers_.IFCR = ifcr_all;
    registers_.CR2 = static_cast<std::uint32_t>(size);
    registers_.CFG1 = registers_.CFG1 | cfg1_tx_dma_enable | cfg1_rx_dma_enable;
    registers_.CR1 = registers_.CR1 | cr1_enabled | cr1_master_start;

    if (!wait_dma() || !wait_flag(sr_end_of_transfer)) {
      (void)stop_dma();
      return result<void, error_type>::failure(
          error{hal::spi::error_kind::timeout});
    }
    if (!stop_dma()) {
      return result<void, error_type>::failure(
          error{hal::spi::error_kind::timeout});
    }
    barrier();
    for (std::size_t index = 0U; index < rx.size(); ++index) {
      rx[index] = rx_scratch_[index];
    }
    return result<void, error_type>::success();
  }

  Registers &registers_;
  TxDmaStream &tx_stream_;
  RxDmaStream &rx_stream_;
  TxDmamuxChannel &tx_dmamux_;
  RxDmamuxChannel &rx_dmamux_;
  std::uint32_t kernel_clock_hz_{};
  poll_budget timeout_{};
  std::uint8_t tx_request_{};
  std::uint8_t rx_request_{};
  std::array<std::byte, ScratchCapacity> &tx_scratch_;
  std::array<std::byte, ScratchCapacity> &rx_scratch_;
  std::byte read_fill_{std::byte{0xFFU}};
  bool configured_{};
};

} // namespace hal::stm32h7::spi
