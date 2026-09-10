#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <hal/foundation/assert.hpp>
#include <hal/i2c.hpp>
#include <hal/stm32h7/support.hpp>
#include <type_traits>

namespace hal::stm32h7::i2c {

inline constexpr capability support{
    implementation_status::available,
    "direct I2C controller with optional TX/RX DMA payload transfers"};

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
  // Keep dma_request as the transmit request for source compatibility with
  // the original four-field aggregate.  A receive request of zero means
  // "use dma_request", which is useful for peripherals with one request
  // direction and preserves the no-DMA/default configuration.
  std::uint8_t dma_request{};
  bool use_dma{};
  std::uint8_t dma_receive_request{};
};

struct no_dma {};

// A DMA stream moves only the payload. The I2C peripheral still owns
// START/STOP and repeated-START sequencing, so a DMA transfer cannot create a
// malformed transaction boundary.
template <class Registers, unsigned Instance = 1U,
          class DmaStream = no_dma, class DmamuxChannel = no_dma>
class Controller {
  using stream_type = std::conditional_t<std::is_same_v<DmaStream, no_dma>,
                                         no_dma, DmaStream>;
  using mux_type = std::conditional_t<std::is_same_v<DmamuxChannel, no_dma>,
                                      no_dma, DmamuxChannel>;

public:
  using error_type = error;

  explicit Controller(Registers &registers, config configuration) noexcept
      : registers_{registers}, configuration_{configuration} {}

  Controller(Registers &registers, DmaStream &stream, DmamuxChannel &dmamux,
             config configuration) noexcept
    requires(!std::is_same_v<DmaStream, no_dma> &&
             !std::is_same_v<DmamuxChannel, no_dma>)
      : registers_{registers}, configuration_{configuration}, stream_{&stream},
        dmamux_{&dmamux} {}

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
    registers_.CR1 = cr1_peripheral_enable;
    initialized_ = true;
    return result<void, error_type>::success();
  }

  template <class Address>
  [[nodiscard]] auto transaction(Address address,
                                 hal::span<const hal::i2c::operation> operations)
      -> result<void, error_type> {
    HAL_CORE_ASSERT(operations.valid());
    if ((registers_.ISR & isr_busy) != 0U) {
      return failure(hal::i2c::error_kind::busy);
    }
    if (!initialized_ || !operations.valid() || !valid_address(address) ||
        operations.empty()) {
      return failure(hal::i2c::error_kind::configuration);
    }

    for (std::size_t index = 0U; index < operations.size(); ++index) {
      const auto &operation = operations[index];
      if (!operation.valid()) {
        return failure(hal::i2c::error_kind::configuration);
      }
      const bool last_operation = index + 1U == operations.size();
      const auto outcome = run_operation(address, operation, last_operation);
      if (!outcome) {
        return outcome;
      }
    }
    return result<void, error_type>::success();
  }

  // A complete nine-clock GPIO recovery sequence belongs in the BSP because
  // the pin mux and external pull-ups are board facts. This handles only the
  // peripheral-local reset case.
  [[nodiscard]] auto recover_bus() -> result<void, error_type> {
    if ((registers_.ISR & isr_busy) == 0U) {
      return result<void, error_type>::success();
    }
    registers_.CR1 = 0U;
    clear_flags();
    registers_.CR1 = cr1_peripheral_enable;
    return (registers_.ISR & isr_busy) == 0U
               ? result<void, error_type>::success()
               : failure(hal::i2c::error_kind::busy);
  }

  [[nodiscard]] bool initialized() const noexcept { return initialized_; }
  [[nodiscard]] static constexpr unsigned instance() noexcept { return Instance; }

private:
  static constexpr std::uint32_t cr1_peripheral_enable{1U << 0U};
  static constexpr std::uint32_t cr1_tx_dma_enable{1U << 14U};
  static constexpr std::uint32_t cr1_rx_dma_enable{1U << 15U};
  static constexpr std::uint32_t cr2_read_direction{1U << 10U};
  static constexpr std::uint32_t cr2_ten_bit_address{1U << 11U};
  static constexpr std::uint32_t cr2_start{1U << 13U};
  static constexpr std::uint32_t cr2_stop{1U << 14U};
  static constexpr std::uint32_t cr2_nbytes_position{16U};
  static constexpr std::uint32_t cr2_reload{1U << 24U};
  static constexpr std::uint32_t cr2_autoend{1U << 25U};
  static constexpr std::uint32_t isr_txis{1U << 1U};
  static constexpr std::uint32_t isr_rxne{1U << 2U};
  static constexpr std::uint32_t isr_nack{1U << 4U};
  static constexpr std::uint32_t isr_stop{1U << 5U};
  static constexpr std::uint32_t isr_transfer_complete{1U << 6U};
  static constexpr std::uint32_t isr_transfer_reload{1U << 7U};
  static constexpr std::uint32_t isr_bus_error{1U << 8U};
  static constexpr std::uint32_t isr_arbitration_lost{1U << 9U};
  static constexpr std::uint32_t isr_busy{1U << 15U};
  static constexpr std::uint32_t icr_nack{1U << 4U};
  static constexpr std::uint32_t icr_stop{1U << 5U};
  static constexpr std::uint32_t icr_bus_error{1U << 8U};
  static constexpr std::uint32_t icr_arbitration_lost{1U << 9U};
  static constexpr std::uint32_t dma_enable{1U << 0U};
  static constexpr std::uint32_t dma_direction_memory_to_peripheral{1U << 6U};
  static constexpr std::uint32_t dma_memory_increment{1U << 10U};
  static constexpr std::uint32_t dma_peripheral_byte{1U << 11U};
  static constexpr std::uint32_t dma_memory_byte{1U << 13U};

  [[nodiscard]] auto failure(hal::i2c::error_kind kind)
      -> result<void, error_type> {
    abort_dma();
    clear_flags();
    registers_.CR2 = registers_.CR2 | cr2_stop;
    return result<void, error_type>::failure(error{kind});
  }

  void clear_flags() noexcept {
    registers_.ICR = icr_nack | icr_stop | icr_bus_error | icr_arbitration_lost;
  }

  template <class Address>
  [[nodiscard]] static constexpr bool valid_address(Address address) noexcept {
    if constexpr (std::is_same_v<Address, hal::i2c::address7>) {
      return address.valid();
    } else if constexpr (std::is_same_v<Address, hal::i2c::address10>) {
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
      return static_cast<std::uint32_t>(address.value) | cr2_ten_bit_address;
    }
  }

  [[nodiscard]] bool wait(std::uint32_t mask) const noexcept {
    for (std::uint32_t remaining = configuration_.timeout.iterations;
         remaining > 0U; --remaining) {
      if ((registers_.ISR & mask) != 0U) {
        return true;
      }
      if ((registers_.ISR & (isr_nack | isr_bus_error |
                             isr_arbitration_lost)) != 0U) {
        return false;
      }
    }
    return false;
  }

  [[nodiscard]] hal::i2c::error_kind observed_error() const noexcept {
    const std::uint32_t status = registers_.ISR;
    if ((status & isr_nack) != 0U) {
      return hal::i2c::error_kind::no_acknowledge;
    }
    if ((status & isr_arbitration_lost) != 0U) {
      return hal::i2c::error_kind::arbitration_lost;
    }
    if ((status & isr_bus_error) != 0U) {
      return hal::i2c::error_kind::bus_error;
    }
    return hal::i2c::error_kind::timeout;
  }

  template <class Address>
  [[nodiscard]] auto run_operation(Address address,
                                   const hal::i2c::operation &operation,
                                   bool last_operation)
      -> result<void, error_type> {
    const std::size_t total = operation.size();
    std::size_t offset = 0U;
    if (total == 0U) {
      configure_transfer(address, operation.dir() == hal::i2c::direction::read,
                         0U, last_operation, false);
      registers_.CR2 = registers_.CR2 | cr2_start;
      return wait_for_completion(last_operation);
    }

    while (offset < total) {
      const std::size_t remaining = total - offset;
      const std::size_t chunk = remaining > 255U ? 255U : remaining;
      const bool final_chunk = last_operation && chunk == remaining;
      const bool read = operation.dir() == hal::i2c::direction::read;
      configure_transfer(address, operation.dir() == hal::i2c::direction::read,
                         chunk, final_chunk, chunk != remaining);
      if (use_dma() &&
          !prepare_dma(read ? data_pointer(operation.read_buffer(), offset)
                            : data_pointer(operation.write_buffer(), offset),
                       chunk, !read)) {
        return failure(hal::i2c::error_kind::io);
      }
      barrier();
      registers_.CR2 = registers_.CR2 | cr2_start;

      auto outcome = use_dma()
                         ? wait_dma_completion()
                         : (read ? receive_chunk(operation.read_buffer(), offset,
                                                  chunk)
                                 : transmit_chunk(operation.write_buffer(), offset,
                                                  chunk));
      if (!outcome) {
        return outcome;
      }
      offset += chunk;
      if (!final_chunk) {
        const std::uint32_t boundary =
            chunk != remaining ? isr_transfer_reload : isr_transfer_complete;
        if (!wait(boundary)) {
          return failure(observed_error());
        }
      }
    }
    return wait_for_completion(last_operation);
  }

  template <class Address>
  void configure_transfer(Address address, bool read, std::size_t count,
                          bool final_chunk, bool reload) noexcept {
    std::uint32_t value = address_bits(address) |
                          (static_cast<std::uint32_t>(count)
                           << cr2_nbytes_position);
    if (read) {
      value |= cr2_read_direction;
    }
    if (reload) {
      value |= cr2_reload;
    } else if (final_chunk) {
      value |= cr2_autoend;
    }
    registers_.CR2 = value;
  }

  [[nodiscard]] auto wait_for_completion(bool final_operation)
      -> result<void, error_type> {
    if (final_operation) {
      if (!wait(isr_stop)) {
        return failure(observed_error());
      }
      clear_flags();
    } else if (!wait(isr_transfer_complete)) {
      return failure(observed_error());
    }
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto transmit_chunk(hal::span<const std::byte> data,
                                    std::size_t offset, std::size_t count)
      -> result<void, error_type> {
    if (use_dma()) {
      return result<void, error_type>::failure(
          error{hal::i2c::error_kind::io});
    }
    for (std::size_t index = 0U; index < count; ++index) {
      if (!wait(isr_txis)) {
        return failure(observed_error());
      }
      registers_.TXDR = std::to_integer<std::uint8_t>(data[offset + index]);
    }
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto receive_chunk(hal::span<std::byte> data,
                                   std::size_t offset, std::size_t count)
      -> result<void, error_type> {
    if (use_dma()) {
      return result<void, error_type>::failure(
          error{hal::i2c::error_kind::io});
    }
    for (std::size_t index = 0U; index < count; ++index) {
      if (!wait(isr_rxne)) {
        return failure(observed_error());
      }
      data[offset + index] = static_cast<std::byte>(registers_.RXDR & 0xFFU);
    }
    return result<void, error_type>::success();
  }

  [[nodiscard]] bool use_dma() const noexcept {
    return configuration_.use_dma && stream_ != nullptr && dmamux_ != nullptr;
  }

  [[nodiscard]] bool stop_dma() noexcept {
    if constexpr (std::is_same_v<DmaStream, no_dma> ||
                  std::is_same_v<DmamuxChannel, no_dma>) {
      return true;
    } else {
      registers_.CR1 = registers_.CR1 &
                       ~(cr1_tx_dma_enable | cr1_rx_dma_enable);
      stream_->CR = stream_->CR & ~dma_enable;
      for (std::uint32_t remaining = configuration_.timeout.iterations;
           remaining > 0U; --remaining) {
        if ((stream_->CR & dma_enable) == 0U) {
          return true;
        }
      }
      return false;
    }
  }

  [[nodiscard]] bool prepare_dma(const volatile std::byte *data,
                                 std::size_t count, bool transmit) noexcept {
    if constexpr (std::is_same_v<DmaStream, no_dma> ||
                  std::is_same_v<DmamuxChannel, no_dma>) {
      (void)data;
      (void)count;
      (void)transmit;
      return false;
    } else {
      if (!stop_dma()) {
        return false;
      }
      dmamux_->CCR = transmit
                         ? configuration_.dma_request
                         : (configuration_.dma_receive_request != 0U
                                ? configuration_.dma_receive_request
                                : configuration_.dma_request);
      stream_->CR = 0U;
      stream_->PAR = pointer_word(transmit ? &registers_.TXDR : &registers_.RXDR);
      stream_->M0AR = pointer_word(data);
      stream_->NDTR = static_cast<std::uint32_t>(count);
      stream_->FCR = 0U;
      stream_->CR = dma_enable | dma_memory_increment |
                    (transmit ? dma_direction_memory_to_peripheral : 0U) |
                    dma_peripheral_byte | dma_memory_byte;
      registers_.CR1 = registers_.CR1 |
                       (transmit ? cr1_tx_dma_enable : cr1_rx_dma_enable);
      return true;
    }
  }

  [[nodiscard]] auto wait_dma_completion() -> result<void, error_type> {
    if constexpr (std::is_same_v<DmaStream, no_dma> ||
                  std::is_same_v<DmamuxChannel, no_dma>) {
      return failure(hal::i2c::error_kind::io);
    } else {
    for (std::uint32_t remaining = configuration_.timeout.iterations;
         remaining > 0U; --remaining) {
      const std::uint32_t status = registers_.ISR;
      if ((status & (isr_nack | isr_bus_error | isr_arbitration_lost)) != 0U) {
        const auto kind = observed_error();
        abort_dma();
        return failure(kind);
      }
      if (stream_->NDTR == 0U) {
        abort_dma();
        return result<void, error_type>::success();
      }
    }
    abort_dma();
    return failure(hal::i2c::error_kind::timeout);
    }
  }

  void abort_dma() noexcept {
    (void)stop_dma();
  }

  [[nodiscard]] static const volatile std::byte *
  data_pointer(hal::span<const std::byte> data, std::size_t offset) noexcept {
    return data.data() + offset;
  }

  [[nodiscard]] static volatile std::byte *
  data_pointer(hal::span<std::byte> data, std::size_t offset) noexcept {
    return data.data() + offset;
  }

  [[nodiscard]] static std::uint32_t pointer_word(const volatile void *pointer)
      noexcept {
    return static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(pointer));
  }

  static void barrier() noexcept {
#if defined(__arm__) || defined(__thumb__)
    __asm volatile("dsb 0xF" ::: "memory");
#else
    std::atomic_thread_fence(std::memory_order_seq_cst);
#endif
  }

  Registers &registers_;
  config configuration_{};
  stream_type *stream_{};
  mux_type *dmamux_{};
  bool initialized_{};
};

} // namespace hal::stm32h7::i2c
