#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/i2c.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <hal/foundation/assert.hpp>
#include "support.hpp"
#include <type_traits>

namespace hal::stm32g4::i2c {

inline constexpr capability support{
    implementation_status::available,
    "modern I2C repeated-start transactions with channel-DMA payloads"};

class error {
public:
  explicit constexpr error(hal::i2c::error_kind kind) noexcept : kind_{kind} {}
  [[nodiscard]] constexpr hal::i2c::error_kind kind() const noexcept {
    return kind_;
  }

private:
  hal::i2c::error_kind kind_{};
};

struct config {
  std::uint32_t timing_register{};
  poll_budget timeout{};
  std::uint8_t tx_request{};
  bool use_dma{};
  std::uint8_t rx_request{};
};

struct no_dma {};

template <class Registers, unsigned Instance = 1U,
          class TxChannel = no_dma, class RxChannel = no_dma,
          class TxDmamux = no_dma, class RxDmamux = no_dma>
class Controller {
public:
  using error_type = error;

  explicit Controller(Registers &registers, config configuration) noexcept
      : registers_{registers}, configuration_{configuration} {}

  Controller(Registers &registers, TxChannel &tx, RxChannel &rx,
             TxDmamux &tx_mux, RxDmamux &rx_mux,
             config configuration) noexcept
      : registers_{registers}, configuration_{configuration}, tx_{&tx},
        rx_{&rx}, tx_mux_{&tx_mux}, rx_mux_{&rx_mux} {}

  Controller(const Controller &) = delete;
  Controller &operator=(const Controller &) = delete;

  [[nodiscard]] auto initialize() -> result<void, error_type> {
    if (configuration_.timing_register == 0U ||
        !configuration_.timeout.valid()) {
      return failure(hal::i2c::error_kind::configuration);
    }
    registers_.CR1 = 0U;
    registers_.TIMINGR = configuration_.timing_register;
    clear_flags();
    registers_.CR1 = cr1_pe;
    initialized_ = true;
    return result<void, error_type>::success();
  }

  template <class Address>
  [[nodiscard]] auto transaction(Address address,
                                 hal::span<const hal::i2c::operation> operations)
      -> result<void, error_type> {
    HAL_CORE_ASSERT(operations.valid());
    if (!initialized_ || !operations.valid() || operations.empty() ||
        !valid_address(address)) {
      return failure(hal::i2c::error_kind::configuration);
    }
    if ((registers_.ISR & isr_busy) != 0U) {
      return failure(hal::i2c::error_kind::busy);
    }
    for (std::size_t i = 0U; i < operations.size(); ++i) {
      if (!operations[i].valid()) {
        return failure(hal::i2c::error_kind::configuration);
      }
      const auto outcome = run_operation(address, operations[i],
                                         i + 1U == operations.size());
      if (!outcome) return outcome;
    }
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto recover_bus() -> result<void, error_type> {
    stop_dma();
    registers_.CR1 = 0U;
    clear_flags();
    registers_.CR1 = cr1_pe;
    return (registers_.ISR & isr_busy) == 0U
               ? result<void, error_type>::success()
               : failure(hal::i2c::error_kind::busy);
  }

  [[nodiscard]] bool initialized() const noexcept { return initialized_; }
  [[nodiscard]] static constexpr unsigned instance() noexcept { return Instance; }

private:
  static constexpr std::uint32_t cr1_pe{1U};
  static constexpr std::uint32_t cr1_txdma{1U << 14U};
  static constexpr std::uint32_t cr1_rxdma{1U << 15U};
  static constexpr std::uint32_t cr2_rd_wrn{1U << 10U};
  static constexpr std::uint32_t cr2_add10{1U << 11U};
  static constexpr std::uint32_t cr2_start{1U << 13U};
  static constexpr std::uint32_t cr2_stop{1U << 14U};
  static constexpr std::uint32_t cr2_nbytes{16U};
  static constexpr std::uint32_t cr2_reload{1U << 24U};
  static constexpr std::uint32_t cr2_autoend{1U << 25U};
  static constexpr std::uint32_t isr_txis{1U << 1U};
  static constexpr std::uint32_t isr_rxne{1U << 2U};
  static constexpr std::uint32_t isr_nack{1U << 4U};
  static constexpr std::uint32_t isr_stop{1U << 5U};
  static constexpr std::uint32_t isr_tc{1U << 6U};
  static constexpr std::uint32_t isr_tcr{1U << 7U};
  static constexpr std::uint32_t isr_berr{1U << 8U};
  static constexpr std::uint32_t isr_arlo{1U << 9U};
  static constexpr std::uint32_t isr_busy{1U << 15U};
  static constexpr std::uint32_t icr_nack{1U << 4U};
  static constexpr std::uint32_t icr_stop{1U << 5U};
  static constexpr std::uint32_t icr_berr{1U << 8U};
  static constexpr std::uint32_t icr_arlo{1U << 9U};
  static constexpr std::uint32_t dma_enable{1U};
  static constexpr std::uint32_t dma_dir{1U << 4U};
  static constexpr std::uint32_t dma_minc{1U << 7U};

  template <class Address>
  [[nodiscard]] static constexpr bool valid_address(Address address) noexcept {
    if constexpr (std::is_same_v<Address, hal::i2c::address7> ||
                  std::is_same_v<Address, hal::i2c::address10>) {
      return address.valid();
    } else {
      return false;
    }
  }

  template <class Address>
  [[nodiscard]] static constexpr std::uint32_t address_bits(Address address)
      noexcept {
    if constexpr (std::is_same_v<Address, hal::i2c::address7>) {
      return static_cast<std::uint32_t>(address.value) << 1U;
    } else {
      return static_cast<std::uint32_t>(address.value) | cr2_add10;
    }
  }

  void clear_flags() noexcept {
    registers_.ICR = icr_nack | icr_stop | icr_berr | icr_arlo;
  }

  [[nodiscard]] bool wait(std::uint32_t wanted) const noexcept {
    for (std::uint32_t left = configuration_.timeout.iterations; left > 0U;
         --left) {
      const auto status = registers_.ISR;
      if ((status & wanted) != 0U) return true;
      if ((status & (isr_nack | isr_berr | isr_arlo)) != 0U) return false;
    }
    return false;
  }

  [[nodiscard]] hal::i2c::error_kind observed_error() const noexcept {
    const auto status = registers_.ISR;
    if ((status & isr_nack) != 0U) return hal::i2c::error_kind::no_acknowledge;
    if ((status & isr_arlo) != 0U) return hal::i2c::error_kind::arbitration_lost;
    if ((status & isr_berr) != 0U) return hal::i2c::error_kind::bus_error;
    return hal::i2c::error_kind::timeout;
  }

  template <class Address>
  [[nodiscard]] auto run_operation(Address address,
                                    const hal::i2c::operation &operation,
                                    bool last_operation)
      -> result<void, error_type> {
    const bool read = operation.dir() == hal::i2c::direction::read;
    const std::size_t total = operation.size();
    if (total == 0U) {
      configure_transfer(address, read, 0U, last_operation, false);
      registers_.CR2 = registers_.CR2 | cr2_start;
      return complete(last_operation);
    }

    std::size_t offset = 0U;
    while (offset < total) {
      const std::size_t remaining = total - offset;
      const std::size_t count = remaining > 255U ? 255U : remaining;
      const bool final_chunk = last_operation && count == remaining;
      configure_transfer(address, read, count, final_chunk,
                        count != remaining);
      if (use_dma() && !prepare_dma(read ? operation.read_buffer().data() + offset
                                         : operation.write_buffer().data() + offset,
                                     count, read)) {
        return failure(hal::i2c::error_kind::io);
      }
      barrier();
      registers_.CR2 = registers_.CR2 | cr2_start;
      const auto transfer = use_dma()
                                ? wait_dma(count)
                                : read ? receive(operation.read_buffer(), offset, count)
                                       : transmit(operation.write_buffer(), offset, count);
      if (!transfer) return transfer;
      stop_dma();
      offset += count;
      if (!final_chunk && !wait(count != remaining ? isr_tcr : isr_tc)) {
        return failure(observed_error());
      }
    }
    return complete(last_operation);
  }

  template <class Address>
  void configure_transfer(Address address, bool read, std::size_t count,
                          bool final_chunk, bool reload) noexcept {
    std::uint32_t value = address_bits(address) |
                          (static_cast<std::uint32_t>(count) << cr2_nbytes);
    if (read) value |= cr2_rd_wrn;
    if (reload) value |= cr2_reload;
    else if (final_chunk) value |= cr2_autoend;
    registers_.CR2 = value;
  }

  [[nodiscard]] auto complete(bool final_operation) -> result<void, error_type> {
    const std::uint32_t wanted = final_operation ? isr_stop : isr_tc;
    if (!wait(wanted)) return failure(observed_error());
    clear_flags();
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto transmit(hal::span<const std::byte> data,
                              std::size_t offset, std::size_t count)
      -> result<void, error_type> {
    for (std::size_t i = 0U; i < count; ++i) {
      if (!wait(isr_txis)) return failure(observed_error());
      registers_.TXDR = std::to_integer<std::uint8_t>(data[offset + i]);
    }
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto receive(hal::span<std::byte> data, std::size_t offset,
                             std::size_t count) -> result<void, error_type> {
    for (std::size_t i = 0U; i < count; ++i) {
      if (!wait(isr_rxne)) return failure(observed_error());
      data[offset + i] = static_cast<std::byte>(registers_.RXDR & 0xFFU);
    }
    return result<void, error_type>::success();
  }

  [[nodiscard]] bool use_dma() const noexcept {
    return configuration_.use_dma && tx_ != nullptr && rx_ != nullptr &&
           tx_mux_ != nullptr && rx_mux_ != nullptr;
  }

  template <class Channel>
  static void configure_channel(Channel &channel, std::uint32_t peripheral,
                                std::uint32_t memory, std::size_t count,
                                bool transmit) noexcept {
    channel.CCR = 0U;
    channel.CPAR = peripheral;
    channel.CMAR = memory;
    channel.CNDTR = static_cast<std::uint32_t>(count);
    channel.CCR = dma_minc | (transmit ? dma_dir : 0U) | dma_enable;
  }

  [[nodiscard]] static std::uint32_t pointer_word(const volatile void *pointer)
      noexcept {
    return static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(pointer));
  }

  [[nodiscard]] bool prepare_dma(const volatile std::byte *data,
                                 std::size_t count, bool transmit) noexcept {
    if constexpr (std::is_same_v<TxChannel, no_dma> ||
                  std::is_same_v<RxChannel, no_dma> ||
                  std::is_same_v<TxDmamux, no_dma> ||
                  std::is_same_v<RxDmamux, no_dma>) {
      (void)data;
      (void)count;
      (void)transmit;
      return false;
    } else {
      if (!use_dma() || count == 0U) return false;
      stop_dma();
      if (transmit) {
        tx_mux_->CCR = configuration_.tx_request;
        configure_channel(*tx_, pointer_word(&registers_.TXDR), pointer_word(data),
                          count, true);
        registers_.CR1 = registers_.CR1 | cr1_txdma;
        tx_->CCR = tx_->CCR | dma_enable;
      } else {
        rx_mux_->CCR = configuration_.rx_request;
        configure_channel(*rx_, pointer_word(&registers_.RXDR), pointer_word(data),
                          count, false);
        registers_.CR1 = registers_.CR1 | cr1_rxdma;
        rx_->CCR = rx_->CCR | dma_enable;
      }
      return true;
    }
  }

  [[nodiscard]] auto wait_dma(std::size_t count)
      -> result<void, error_type> {
    if constexpr (std::is_same_v<TxChannel, no_dma> ||
                  std::is_same_v<RxChannel, no_dma>) {
      (void)count;
      return failure(hal::i2c::error_kind::io);
    } else {
      for (std::uint32_t left = configuration_.timeout.iterations; left > 0U;
           --left) {
        if ((registers_.CR2 & cr2_rd_wrn) != 0U) {
          if (rx_ != nullptr && rx_->CNDTR == 0U) {
            return result<void, error_type>::success();
          }
        } else if (tx_ != nullptr && tx_->CNDTR == 0U) {
          return result<void, error_type>::success();
        }
        if ((registers_.ISR & (isr_nack | isr_berr | isr_arlo)) != 0U) {
          return failure(observed_error());
        }
      }
      (void)count;
      return failure(hal::i2c::error_kind::timeout);
    }
  }

  static void barrier() noexcept {
#if defined(__arm__) || defined(__thumb__)
    __asm volatile("dmb 0xF" ::: "memory");
#else
    std::atomic_thread_fence(std::memory_order_seq_cst);
#endif
  }

  void stop_dma() noexcept {
    registers_.CR1 = registers_.CR1 & ~(cr1_txdma | cr1_rxdma);
    if constexpr (!std::is_same_v<TxChannel, no_dma>) {
      if (tx_ != nullptr) tx_->CCR = tx_->CCR & ~dma_enable;
    }
    if constexpr (!std::is_same_v<RxChannel, no_dma>) {
      if (rx_ != nullptr) rx_->CCR = rx_->CCR & ~dma_enable;
    }
  }

  [[nodiscard]] auto failure(hal::i2c::error_kind kind)
      -> result<void, error_type> {
    stop_dma();
    registers_.CR2 = registers_.CR2 | cr2_stop;
    clear_flags();
    return result<void, error_type>::failure(error{kind});
  }

  Registers &registers_;
  config configuration_{};
  TxChannel *tx_{};
  RxChannel *rx_{};
  TxDmamux *tx_mux_{};
  RxDmamux *rx_mux_{};
  bool initialized_{};
};

} // namespace hal::stm32g4::i2c
