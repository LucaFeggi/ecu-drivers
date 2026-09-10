#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <hal/spi/bus.hpp>
#include <hal/stm32h5/support.hpp>

namespace hal::stm32h5::spi {

inline constexpr capability support{implementation_status::available,
                                    "8-bit SPI polling and GPDMA transfers"};
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
  PollingBus8(Registers &registers, std::uint32_t clock_hz,
              poll_budget timeout) noexcept
      : registers_{registers}, clock_hz_{clock_hz}, timeout_{timeout} {}
  PollingBus8(const PollingBus8 &) = delete;
  PollingBus8 &operator=(const PollingBus8 &) = delete;
  [[nodiscard]] auto configure(hal::spi::config8 configuration)
      -> result<hertz, error_type> {
    if (clock_hz_ == 0U || configuration.max_frequency.value == 0U ||
        !timeout_.valid())
      return failure<hertz>(hal::spi::error_kind::configuration);
    std::uint32_t divisor = 2U;
    std::uint32_t code = 0U;
    while (clock_hz_ / divisor > configuration.max_frequency.value &&
           divisor < 256U) {
      divisor *= 2U;
      ++code;
    }
    if (clock_hz_ / divisor > configuration.max_frequency.value)
      return failure<hertz>(hal::spi::error_kind::configuration);
    registers_.CFG1 = 7U | (code << 28U);
    std::uint32_t cfg2 = (1U << 31U) | (1U << 22U) | (1U << 26U);
    if (configuration.order == hal::spi::bit_order::lsb_first)
      cfg2 |= 1U << 23U;
    if (configuration.clock_mode == hal::spi::mode::mode1 ||
        configuration.clock_mode == hal::spi::mode::mode3)
      cfg2 |= 1U << 24U;
    if (configuration.clock_mode == hal::spi::mode::mode2 ||
        configuration.clock_mode == hal::spi::mode::mode3)
      cfg2 |= 1U << 25U;
    registers_.CFG2 = cfg2;
    registers_.CR1 = 1U << 12U;
    fill_ = configuration.read_fill;
    configured_ = true;
    return result<hertz, error_type>::success(hertz{clock_hz_ / divisor});
  }
  [[nodiscard]] auto write(span<const std::byte> data)
      -> result<void, error_type> {
    return exchange(data, {}, std::byte{0U});
  }
  [[nodiscard]] auto read(span<std::byte> data, std::byte fill)
      -> result<void, error_type> {
    return exchange({}, data, fill);
  }
  [[nodiscard]] auto transfer(span<const std::byte> tx, span<std::byte> rx)
      -> result<void, error_type> {
    return exchange(tx, rx, fill_);
  }
  [[nodiscard]] auto flush() -> result<void, error_type> {
    return wait(end_of_transfer) ? result<void, error_type>::success()
                                 : failure<void>(hal::spi::error_kind::timeout);
  }

private:
  template <class T>
  [[nodiscard]] static auto failure(hal::spi::error_kind kind)
      -> result<T, error_type> {
    return result<T, error_type>::failure(error{kind});
  }
  static constexpr std::uint32_t tx_space{1U << 1U};
  static constexpr std::uint32_t rx_ready{1U << 0U};
  static constexpr std::uint32_t end_of_transfer{1U << 3U};
  static constexpr std::uint32_t errors{(1U << 5U) | (1U << 6U) | (1U << 8U)};
  [[nodiscard]] bool wait(std::uint32_t flag) const noexcept {
    for (std::uint32_t left = timeout_.iterations; left > 0U; --left) {
      const auto status = registers_.SR;
      if ((status & flag) != 0U)
        return true;
      if ((status & errors) != 0U)
        return false;
    }
    return false;
  }
  [[nodiscard]] auto exchange(span<const std::byte> tx, span<std::byte> rx,
                              std::byte fill) -> result<void, error_type> {
    HAL_CORE_ASSERT(tx.valid());
    HAL_CORE_ASSERT(rx.valid());
    if (!configured_ || !tx.valid() || !rx.valid())
      return failure<void>(hal::spi::error_kind::configuration);
    const auto count = tx.size() > rx.size() ? tx.size() : rx.size();
    if (count > 0xFFFFU)
      return failure<void>(hal::spi::error_kind::configuration);
    registers_.CR2 = static_cast<std::uint32_t>(count);
    registers_.CR1 |= 1U | (1U << 9U);
    auto &txdr = *reinterpret_cast<volatile std::uint8_t *>(&registers_.TXDR);
    auto &rxdr = *reinterpret_cast<volatile std::uint8_t *>(&registers_.RXDR);
    for (std::size_t i = 0U; i < count; ++i) {
      if (!wait(tx_space))
        return failure<void>(hal::spi::error_kind::timeout);
      txdr = i < tx.size() ? std::to_integer<std::uint8_t>(tx[i])
                           : std::to_integer<std::uint8_t>(fill);
      if (!wait(rx_ready))
        return failure<void>(hal::spi::error_kind::timeout);
      if (i < rx.size())
        rx[i] = static_cast<std::byte>(rxdr);
    }
    if (!wait(end_of_transfer))
      return failure<void>(hal::spi::error_kind::timeout);
    registers_.CR1 &= ~1U;
    return result<void, error_type>::success();
  }
  Registers &registers_;
  std::uint32_t clock_hz_{};
  poll_budget timeout_{};
  std::byte fill_{0xFF};
  bool configured_{};
};

template <class Registers, class TxChannel, class RxChannel,
          std::size_t Capacity = 4096U>
class DmaBus8 {
  static_assert(Capacity > 0U && Capacity <= 65'535U);

public:
  using error_type = error;
  static constexpr bool supports_lsb_first{true};
  DmaBus8(Registers &registers, TxChannel &tx, RxChannel &rx,
          std::uint32_t clock_hz, poll_budget timeout, std::uint8_t tx_request,
          std::uint8_t rx_request, std::array<std::byte, Capacity> &tx_storage,
          std::array<std::byte, Capacity> &rx_storage) noexcept
      : registers_{registers}, tx_{tx}, rx_{rx}, clock_hz_{clock_hz},
        timeout_{timeout}, tx_request_{tx_request}, rx_request_{rx_request},
        tx_storage_{tx_storage}, rx_storage_{rx_storage} {}
  DmaBus8(const DmaBus8 &) = delete;
  DmaBus8 &operator=(const DmaBus8 &) = delete;
  [[nodiscard]] auto configure(hal::spi::config8 c)
      -> result<hertz, error_type> {
    PollingBus8<Registers> polling{registers_, clock_hz_, timeout_};
    auto result = polling.configure(c);
    if (!result)
      return result;
    fill_ = c.read_fill;
    configured_ = true;
    return result;
  }
  [[nodiscard]] auto write(span<const std::byte> data)
      -> result<void, error_type> {
    return exchange(data, {}, std::byte{0U});
  }
  [[nodiscard]] auto read(span<std::byte> data, std::byte fill)
      -> result<void, error_type> {
    return exchange({}, data, fill);
  }
  [[nodiscard]] auto transfer(span<const std::byte> tx, span<std::byte> rx)
      -> result<void, error_type> {
    return exchange(tx, rx, fill_);
  }
  [[nodiscard]] auto flush() -> result<void, error_type> {
    return wait(end_of_transfer) ? result<void, error_type>::success()
                                 : failure<void>(hal::spi::error_kind::timeout);
  }

private:
  template <class T>
  [[nodiscard]] static auto failure(hal::spi::error_kind kind)
      -> result<T, error_type> {
    return result<T, error_type>::failure(error{kind});
  }
  static constexpr std::uint32_t end_of_transfer{1U << 3U};
  static constexpr std::uint32_t error_flags{(1U << 5U) | (1U << 6U) |
                                             (1U << 8U)};
  [[nodiscard]] bool wait(std::uint32_t flag) const noexcept {
    for (std::uint32_t left = timeout_.iterations; left > 0U; --left) {
      const auto status = registers_.SR;
      if ((status & flag) != 0U)
        return true;
      if ((status & error_flags) != 0U)
        return false;
    }
    return false;
  }
  static void barrier() noexcept {
    std::atomic_thread_fence(std::memory_order_seq_cst);
  }
  template <class Channel>
  static void setup(Channel &channel, std::uint8_t request,
                    const volatile void *source, volatile void *destination,
                    std::size_t count, bool source_increment,
                    bool destination_increment) noexcept {
    channel.CCR &= ~1U;
    channel.CTR1 = (source_increment ? (1U << 3U) : 0U) |
                   (destination_increment ? (1U << 19U) : 0U);
    channel.CTR2 = request;
    channel.CBR1 = static_cast<std::uint32_t>(count);
    channel.CSAR =
        static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(source));
    channel.CDAR = static_cast<std::uint32_t>(
        reinterpret_cast<std::uintptr_t>(destination));
    barrier();
    channel.CCR = 1U;
  }
  [[nodiscard]] auto exchange(span<const std::byte> tx, span<std::byte> rx,
                              std::byte fill) -> result<void, error_type> {
    HAL_CORE_ASSERT(tx.valid());
    HAL_CORE_ASSERT(rx.valid());
    if (!configured_ || !tx.valid() || !rx.valid())
      return failure<void>(hal::spi::error_kind::configuration);
    const auto count = tx.size() > rx.size() ? tx.size() : rx.size();
    if (count == 0U)
      return result<void, error_type>::success();
    if (count > Capacity || count > 0xFFFFU)
      return failure<void>(hal::spi::error_kind::configuration);
    for (std::size_t i = 0U; i < count; ++i)
      tx_storage_[i] = i < tx.size() ? tx[i] : fill;
    setup(tx_, tx_request_, tx_storage_.data(), &registers_.TXDR, count, true,
          false);
    setup(rx_, rx_request_, &registers_.RXDR, rx_storage_.data(), count, false,
          true);
    registers_.CFG1 |= (1U << 14U) | (1U << 15U);
    registers_.CR2 = static_cast<std::uint32_t>(count);
    registers_.CR1 |= 1U | (1U << 9U);
    for (std::uint32_t left = timeout_.iterations; left > 0U; --left)
      if (tx_.CBR1 == 0U && rx_.CBR1 == 0U)
        break;
    if (tx_.CBR1 != 0U || rx_.CBR1 != 0U || !wait(end_of_transfer))
      return failure<void>(hal::spi::error_kind::timeout);
    tx_.CCR &= ~1U;
    rx_.CCR &= ~1U;
    registers_.CFG1 &= ~((1U << 14U) | (1U << 15U));
    registers_.CR1 &= ~1U;
    barrier();
    for (std::size_t i = 0U; i < rx.size(); ++i)
      rx[i] = rx_storage_[i];
    return result<void, error_type>::success();
  }
  Registers &registers_;
  TxChannel &tx_;
  RxChannel &rx_;
  std::uint32_t clock_hz_{};
  poll_budget timeout_{};
  std::uint8_t tx_request_{};
  std::uint8_t rx_request_{};
  std::array<std::byte, Capacity> &tx_storage_;
  std::array<std::byte, Capacity> &rx_storage_;
  std::byte fill_{0xFF};
  bool configured_{};
};

} // namespace hal::stm32h5::spi
