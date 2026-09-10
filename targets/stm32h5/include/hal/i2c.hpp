#pragma once

#include <cstddef>
#include <cstdint>
#include <hal/i2c.hpp>
#include <hal/stm32h5/support.hpp>
#include <type_traits>

namespace hal::stm32h5::i2c {

inline constexpr capability support{
    implementation_status::available,
    "I2C repeated-start transactions with optional GPDMA payloads"};
class error {
public:
  explicit constexpr error(hal::i2c::error_kind kind) noexcept : kind_{kind} {}
  [[nodiscard]] constexpr hal::i2c::error_kind kind() const noexcept {
    return kind_;
  }

private:
  hal::i2c::error_kind kind_{};
};
struct no_dma {};
struct config {
  std::uint32_t timing_register{};
  poll_budget timeout{};
  std::uint8_t tx_request{};
  bool use_dma{};
  std::uint8_t rx_request{};
};

template <class Registers, unsigned Instance = 1U, class DmaChannel = no_dma>
class Controller {
public:
  using error_type = error;
  Controller(Registers &r, config c) noexcept : registers_{r}, config_{c} {}
  Controller(Registers &r, DmaChannel &dma, config c) noexcept
      : registers_{r}, config_{c}, dma_{&dma} {}
  Controller(const Controller &) = delete;
  Controller &operator=(const Controller &) = delete;
  [[nodiscard]] auto initialize() -> result<void, error_type> {
    if (config_.timing_register == 0U || !config_.timeout.valid())
      return failure(hal::i2c::error_kind::configuration);
    registers_.CR1 = 0U;
    registers_.TIMINGR = config_.timing_register;
    registers_.ICR = clear_flags;
    registers_.CR1 = 1U;
    initialized_ = true;
    return result<void, error_type>::success();
  }
  template <class Address>
  [[nodiscard]] auto transaction(Address address,
                                 span<const hal::i2c::operation> operations)
      -> result<void, error_type> {
    HAL_CORE_ASSERT(operations.valid());
    if (!initialized_ || !operations.valid() || operations.empty() ||
        !valid_address(address))
      return failure(hal::i2c::error_kind::configuration);
    if ((registers_.ISR & busy) != 0U)
      return failure(hal::i2c::error_kind::busy);
    for (std::size_t i = 0U; i < operations.size(); ++i) {
      const auto &op = operations[i];
      if (!op.valid())
        return failure(hal::i2c::error_kind::configuration);
      const bool last = i + 1U == operations.size();
      auto outcome = run(address, op, last);
      if (!outcome)
        return outcome;
    }
    return result<void, error_type>::success();
  }
  [[nodiscard]] auto recover_bus() -> result<void, error_type> {
    registers_.CR1 = 0U;
    registers_.ICR = clear_flags;
    registers_.CR1 = 1U;
    return (registers_.ISR & busy) == 0U ? result<void, error_type>::success()
                                         : failure(hal::i2c::error_kind::busy);
  }
  [[nodiscard]] constexpr unsigned instance() const noexcept {
    return Instance;
  }

private:
  static constexpr std::uint32_t tx_dma{1U << 14U};
  static constexpr std::uint32_t rx_dma{1U << 15U};
  static constexpr std::uint32_t read_direction{1U << 10U};
  static constexpr std::uint32_t ten_bit{1U << 11U};
  static constexpr std::uint32_t start{1U << 13U};
  static constexpr std::uint32_t stop{1U << 14U};
  static constexpr std::uint32_t nbytes{16U};
  static constexpr std::uint32_t autoend{1U << 25U};
  static constexpr std::uint32_t txis{1U << 1U};
  static constexpr std::uint32_t rxne{1U << 2U};
  static constexpr std::uint32_t nack{1U << 4U};
  static constexpr std::uint32_t stopf{1U << 5U};
  static constexpr std::uint32_t tc{1U << 6U};
  static constexpr std::uint32_t error_flags{(1U << 8U) | (1U << 9U)};
  static constexpr std::uint32_t busy{1U << 15U};
  static constexpr std::uint32_t clear_flags{(1U << 4U) | (1U << 5U) |
                                             (1U << 8U) | (1U << 9U)};
  [[nodiscard]] auto failure(hal::i2c::error_kind k)
      -> result<void, error_type> {
    abort_dma();
    registers_.ICR = clear_flags;
    registers_.CR2 |= stop;
    return result<void, error_type>::failure(error{k});
  }
  template <class T>
  [[nodiscard]] auto failure(hal::i2c::error_kind k) -> result<T, error_type> {
    abort_dma();
    registers_.ICR = clear_flags;
    registers_.CR2 |= stop;
    return result<T, error_type>::failure(error{k});
  }
  template <class A> static constexpr bool valid_address(A a) noexcept {
    return (std::is_same_v<A, hal::i2c::address7> ||
            std::is_same_v<A, hal::i2c::address10>) &&
           a.valid();
  }
  template <class A> static constexpr std::uint32_t address_bits(A a) noexcept {
    if constexpr (std::is_same_v<A, hal::i2c::address7>)
      return static_cast<std::uint32_t>(a.value) << 1U;
    else
      return static_cast<std::uint32_t>(a.value) | ten_bit;
  }
  [[nodiscard]] bool wait(std::uint32_t mask) const noexcept {
    for (std::uint32_t n = config_.timeout.iterations; n > 0U; --n) {
      const auto status = registers_.ISR;
      if ((status & mask) != 0U)
        return true;
      if ((status & (nack | error_flags)) != 0U)
        return false;
    }
    return false;
  }
  [[nodiscard]] hal::i2c::error_kind observed() const noexcept {
    const auto s = registers_.ISR;
    return (s & nack) != 0U         ? hal::i2c::error_kind::no_acknowledge
           : (s & (1U << 9U)) != 0U ? hal::i2c::error_kind::arbitration_lost
           : (s & (1U << 8U)) != 0U ? hal::i2c::error_kind::bus_error
                                    : hal::i2c::error_kind::timeout;
  }
  template <class A>
  [[nodiscard]] auto run(A address, const hal::i2c::operation &op, bool last)
      -> result<void, error_type> {
    const auto count = op.size();
    if (count > 255U)
      return failure<void>(hal::i2c::error_kind::configuration);
    const bool read = op.dir() == hal::i2c::direction::read;
    registers_.CR2 = address_bits(address) | (read ? read_direction : 0U) |
                     (static_cast<std::uint32_t>(count) << nbytes) |
                     (last ? autoend : 0U);
    if (count == 0U)
      registers_.CR2 |= autoend;
    if (use_dma() && count != 0U) {
      if (!prepare_dma(
              read ? static_cast<const volatile void *>(op.read_buffer().data())
                   : static_cast<const volatile void *>(
                         op.write_buffer().data()),
              count, read))
        return failure<void>(hal::i2c::error_kind::io);
    }
    registers_.CR2 |= start;
    if (!use_dma() || count == 0U) {
      for (std::size_t i = 0U; i < count; ++i) {
        if (read) {
          if (!wait(rxne))
            return failure<void>(observed());
          op.read_buffer()[i] = static_cast<std::byte>(registers_.RXDR & 0xFFU);
        } else {
          if (!wait(txis))
            return failure<void>(observed());
          registers_.TXDR = std::to_integer<std::uint8_t>(op.write_buffer()[i]);
        }
      }
    } else if (!wait_dma())
      return failure<void>(hal::i2c::error_kind::timeout);
    if (!wait(last ? stopf : tc))
      return failure<void>(observed());
    abort_dma();
    registers_.ICR = clear_flags;
    return result<void, error_type>::success();
  }
  [[nodiscard]] bool use_dma() const noexcept {
    if constexpr (std::is_same_v<DmaChannel, no_dma>)
      return false;
    else
      return config_.use_dma && dma_ != nullptr;
  }
  bool prepare_dma(const volatile void *memory, std::size_t count,
                   bool read) noexcept {
    if constexpr (std::is_same_v<DmaChannel, no_dma>) {
      (void)memory;
      (void)count;
      (void)read;
      return false;
    } else {
      dma_->CCR &= ~1U;
      dma_->CTR1 = read ? (1U << 19U) : (1U << 3U);
      dma_->CTR2 = read ? (config_.rx_request != 0U ? config_.rx_request
                                                    : config_.tx_request)
                        : config_.tx_request;
      dma_->CBR1 = static_cast<std::uint32_t>(count);
      dma_->CSAR = static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(
          read ? static_cast<const volatile void *>(&registers_.RXDR)
               : memory));
      dma_->CDAR = static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(
          read ? memory
               : static_cast<const volatile void *>(&registers_.TXDR)));
      dma_->CCR = 1U;
      registers_.CR1 |= read ? rx_dma : tx_dma;
      return true;
    }
  }
  [[nodiscard]] bool wait_dma() const noexcept {
    if constexpr (std::is_same_v<DmaChannel, no_dma>)
      return false;
    else {
      for (std::uint32_t n = config_.timeout.iterations; n > 0U; --n)
        if (dma_->CBR1 == 0U)
          return true;
      return false;
    }
  }
  void abort_dma() noexcept {
    if constexpr (!std::is_same_v<DmaChannel, no_dma>) {
      if (dma_ != nullptr)
        dma_->CCR &= ~1U;
    }
    registers_.CR1 &= ~(tx_dma | rx_dma);
  }
  Registers &registers_;
  config config_{};
  DmaChannel *dma_{};
  bool initialized_{};
};
} // namespace hal::stm32h5::i2c
