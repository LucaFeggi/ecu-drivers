#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/i2c.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include <cstddef>
#include <cstdint>
#include "support.hpp"
#include <type_traits>

namespace hal::stm32f4::i2c {
inline constexpr capability support{
    implementation_status::available,
    "classic I2C event sequencing with optional DMA payloads"};
class error {
public:
  explicit constexpr error(hal::i2c::error_kind k) noexcept : kind_{k} {}
  [[nodiscard]] constexpr hal::i2c::error_kind kind() const noexcept {
    return kind_;
  }

private:
  hal::i2c::error_kind kind_{};
};
struct no_dma {};
struct config {
  std::uint32_t peripheral_clock_hz{};
  std::uint32_t bus_hz{100'000U};
  poll_budget timeout{};
  bool use_dma{};
};

template <class Registers, unsigned Instance = 1U, class TxStream = no_dma,
          class RxStream = no_dma>
class Controller {
public:
  using error_type = error;
  Controller(Registers &r, config c) noexcept : registers_{r}, config_{c} {}
  Controller(Registers &r, TxStream &tx, RxStream &rx, config c) noexcept
      : registers_{r}, config_{c}, tx_{&tx}, rx_{&rx} {}
  Controller(const Controller &) = delete;
  Controller &operator=(const Controller &) = delete;
  [[nodiscard]] auto initialize() -> result<void, error_type> {
    if (config_.peripheral_clock_hz < 2'000'000U ||
        config_.peripheral_clock_hz > 50'000'000U || config_.bus_hz == 0U ||
        config_.bus_hz > 400'000U || !config_.timeout.valid())
      return failure(hal::i2c::error_kind::configuration);
    registers_.CR1 = 0U;
    registers_.CR2 = config_.peripheral_clock_hz / 1'000'000U;
    const bool fast_mode = config_.bus_hz > 100'000U;
    const auto ccr = fast_mode
                         ? config_.peripheral_clock_hz / (config_.bus_hz * 3U)
                         : config_.peripheral_clock_hz / (config_.bus_hz * 2U);
    registers_.CCR = (ccr == 0U ? 1U : ccr) | (fast_mode ? (1U << 15U) : 0U);
    registers_.TRISE = config_.bus_hz <= 100'000U
                           ? registers_.CR2 + 1U
                           : (registers_.CR2 * 300U / 1000U) + 1U;
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
        !valid_address(address) || (registers_.SR2 & busy) != 0U)
      return failure(hal::i2c::error_kind::configuration);
    for (std::size_t i = 0U; i < operations.size(); ++i) {
      auto outcome = run(address, operations[i], i + 1U == operations.size());
      if (!outcome)
        return outcome;
    }
    return result<void, error_type>::success();
  }
  [[nodiscard]] auto recover_bus() -> result<void, error_type> {
    registers_.CR1 |= 1U << 15U;
    registers_.CR1 &= ~(1U << 15U);
    registers_.CR1 |= 1U;
    return result<void, error_type>::success();
  }

private:
  static constexpr std::uint32_t start{1U << 8U};
  static constexpr std::uint32_t stop{1U << 9U};
  static constexpr std::uint32_t ack{1U << 10U};
  static constexpr std::uint32_t pe{1U};
  static constexpr std::uint32_t sb{1U << 0U};
  static constexpr std::uint32_t addr{1U << 1U};
  static constexpr std::uint32_t btf{1U << 2U};
  static constexpr std::uint32_t rxne{1U << 6U};
  static constexpr std::uint32_t txe{1U << 7U};
  static constexpr std::uint32_t berr{1U << 8U};
  static constexpr std::uint32_t arlo{1U << 9U};
  static constexpr std::uint32_t af{1U << 10U};
  static constexpr std::uint32_t busy{1U << 1U};
  [[nodiscard]] auto failure(hal::i2c::error_kind k)
      -> result<void, error_type> {
    abort_dma();
    registers_.CR1 |= stop;
    return result<void, error_type>::failure(error{k});
  }
  template <class T>
  [[nodiscard]] auto failure(hal::i2c::error_kind k) -> result<T, error_type> {
    abort_dma();
    registers_.CR1 |= stop;
    return result<T, error_type>::failure(error{k});
  }
  template <class A> static constexpr bool valid_address(A a) noexcept {
    return (std::is_same_v<A, hal::i2c::address7> ||
            std::is_same_v<A, hal::i2c::address10>) &&
           a.valid();
  }
  template <class A> static constexpr std::uint8_t address(A a) noexcept {
    if constexpr (std::is_same_v<A, hal::i2c::address7>)
      return static_cast<std::uint8_t>(a.value << 1U);
    else
      return static_cast<std::uint8_t>((a.value >> 8U) & 0x06U);
  }
  [[nodiscard]] bool wait(std::uint32_t flag) const noexcept {
    for (std::uint32_t n = config_.timeout.iterations; n > 0U; --n) {
      const auto s = registers_.SR1;
      if ((s & flag) != 0U)
        return true;
      if ((s & (berr | arlo | af)) != 0U)
        return false;
    }
    return false;
  }
  [[nodiscard]] hal::i2c::error_kind observed() const noexcept {
    const auto s = registers_.SR1;
    return (s & af) != 0U     ? hal::i2c::error_kind::no_acknowledge
           : (s & arlo) != 0U ? hal::i2c::error_kind::arbitration_lost
           : (s & berr) != 0U ? hal::i2c::error_kind::bus_error
                              : hal::i2c::error_kind::timeout;
  }
  template <class A>
  [[nodiscard]] auto run(A a, const hal::i2c::operation &op, bool last)
      -> result<void, error_type> {
    registers_.CR1 |= start;
    const bool read = op.dir() == hal::i2c::direction::read;
    if (read)
      registers_.CR1 |= ack;
    if (!wait(sb))
      return failure<void>(observed());
    if constexpr (std::is_same_v<A, hal::i2c::address10>) {
      registers_.DR =
          static_cast<std::uint8_t>(0xF0U | ((a.value >> 7U) & 0x06U));
      if (!wait(addr))
        return failure<void>(observed());
      (void)registers_.SR2;
      registers_.DR = static_cast<std::uint8_t>(a.value & 0xFFU);
      if (!wait(btf))
        return failure<void>(observed());
      if (read) {
        registers_.CR1 |= start;
        if (!wait(sb))
          return failure<void>(observed());
        registers_.DR =
            static_cast<std::uint8_t>(0xF1U | ((a.value >> 7U) & 0x06U));
        if (!wait(addr))
          return failure<void>(observed());
        (void)registers_.SR2;
      }
    } else {
      registers_.DR = address(a) | (read ? 1U : 0U);
      if (!wait(addr))
        return failure<void>(observed());
      (void)registers_.SR2;
    }
    const auto count = op.size();
    if (count > 65'535U)
      return failure<void>(hal::i2c::error_kind::configuration);
    if (config_.use_dma && tx_ != nullptr && rx_ != nullptr && count != 0U) {
      configure_dma(op, count);
      if (!wait_dma(op.dir()))
        return failure<void>(hal::i2c::error_kind::timeout);
    } else {
      if (!read)
        for (std::size_t i = 0U; i < count; ++i) {
          if (!wait(txe))
            return failure<void>(observed());
          registers_.DR = std::to_integer<std::uint8_t>(op.write_buffer()[i]);
        }
      else {
        for (std::size_t i = 0U; i < count; ++i) {
          if (i + 1U == count)
            registers_.CR1 &= ~ack;
          if (!wait(rxne))
            return failure<void>(observed());
          op.read_buffer()[i] = static_cast<std::byte>(registers_.DR & 0xFFU);
        }
        registers_.CR1 |= ack;
      }
    }
    if ((!read && !wait(btf)) && count != 0U)
      return failure<void>(observed());
    abort_dma();
    if (last)
      registers_.CR1 |= stop;
    return result<void, error_type>::success();
  }
  void configure_dma(const hal::i2c::operation &op,
                     std::size_t count) noexcept {
    if constexpr (!std::is_same_v<TxStream, no_dma>) {
      if (op.dir() == hal::i2c::direction::write) {
        tx_->CR &= ~1U;
        tx_->PAR = static_cast<std::uint32_t>(
            reinterpret_cast<std::uintptr_t>(&registers_.DR));
        tx_->M0AR = static_cast<std::uint32_t>(
            reinterpret_cast<std::uintptr_t>(op.write_buffer().data()));
        tx_->NDTR = static_cast<std::uint32_t>(count);
        tx_->CR = (1U << 10U) | (1U << 4U) | (1U << 6U) | 1U;
        registers_.CR2 |= 1U << 11U;
      }
    }
    if constexpr (!std::is_same_v<RxStream, no_dma>) {
      if (op.dir() == hal::i2c::direction::read) {
        rx_->CR &= ~1U;
        rx_->PAR = static_cast<std::uint32_t>(
            reinterpret_cast<std::uintptr_t>(&registers_.DR));
        rx_->M0AR = static_cast<std::uint32_t>(
            reinterpret_cast<std::uintptr_t>(op.read_buffer().data()));
        rx_->NDTR = static_cast<std::uint32_t>(count);
        rx_->CR = (1U << 10U) | (1U << 4U) | 1U;
        registers_.CR2 |= (1U << 11U) | (1U << 12U);
      }
    }
  }
  [[nodiscard]] bool wait_dma(hal::i2c::direction direction) const noexcept {
    if (direction == hal::i2c::direction::write) {
      if constexpr (std::is_same_v<TxStream, no_dma>)
        return false;
      else
        for (std::uint32_t n = config_.timeout.iterations; n > 0U; --n)
          if (tx_->NDTR == 0U)
            return true;
    } else {
      if constexpr (std::is_same_v<RxStream, no_dma>)
        return false;
      else
        for (std::uint32_t n = config_.timeout.iterations; n > 0U; --n)
          if (rx_->NDTR == 0U)
            return true;
    }
    return false;
  }
  void abort_dma() noexcept {
    if constexpr (!std::is_same_v<TxStream, no_dma>) {
      if (tx_ != nullptr)
        tx_->CR &= ~1U;
    }
    if constexpr (!std::is_same_v<RxStream, no_dma>) {
      if (rx_ != nullptr)
        rx_->CR &= ~1U;
    }
    registers_.CR2 &= ~((1U << 11U) | (1U << 12U));
  }
  Registers &registers_;
  config config_{};
  TxStream *tx_{};
  RxStream *rx_{};
  bool initialized_{};
};
} // namespace hal::stm32f4::i2c
