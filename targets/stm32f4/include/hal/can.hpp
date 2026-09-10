#pragma once

#include <cstddef>
#include <cstdint>
#include <hal/can.hpp>
#include <hal/stm32f4/support.hpp>

namespace hal::stm32f4::can {
inline constexpr capability support{implementation_status::available,
                                    "bxCAN classic CAN controller"};
class error {
public:
  explicit constexpr error(hal::can::error_kind k) noexcept : kind_{k} {}
  [[nodiscard]] constexpr hal::can::error_kind kind() const noexcept {
    return kind_;
  }

private:
  hal::can::error_kind kind_{};
};
struct config {
  std::uint32_t bit_timing{};
  poll_budget timeout{};
  bool silent{};
};

template <class Registers> class Controller {
public:
  using error_type = error;
  Controller(Registers &r, config c) noexcept : registers_{r}, config_{c} {}
  Controller(const Controller &) = delete;
  Controller &operator=(const Controller &) = delete;
  [[nodiscard]] auto initialize() -> result<void, error_type> {
    if (config_.bit_timing == 0U || !config_.timeout.valid())
      return failure(hal::can::error_kind::configuration);
    registers_.MCR = 1U;
    if (!wait(1U << 0U))
      return failure(hal::can::error_kind::configuration);
    registers_.BTR = config_.bit_timing | (config_.silent ? (1U << 31U) : 0U);
    registers_.MCR = 1U << 6U;
    if (!wait_clear(1U << 0U))
      return failure(hal::can::error_kind::configuration);
    initialized_ = true;
    return result<void, error_type>::success();
  }
  [[nodiscard]] auto try_send(const hal::can::classic_frame &frame)
      -> result<bool, error_type> {
    if (!initialized_)
      return failure<bool>(hal::can::error_kind::configuration);
    if ((registers_.TSR & 1U) == 0U && (registers_.TSR & (1U << 8U)) == 0U &&
        (registers_.TSR & (1U << 16U)) == 0U)
      return result<bool, error_type>::success(false);
    const auto id = frame.frame_id();
    unsigned mailbox = (registers_.TSR & 1U) != 0U           ? 0U
                       : (registers_.TSR & (1U << 8U)) != 0U ? 1U
                                                             : 2U;
    auto &box = registers_.sTxMailBox[mailbox];
    box.TIR = id.extended ? (id.value << 3U) | 4U : id.value << 21U;
    box.TDTR = frame.size();
    box.TDLR = 0U;
    box.TDHR = 0U;
    for (std::size_t i = 0U; i < frame.data().size(); ++i) {
      if (i < 4U)
        box.TDLR |= static_cast<std::uint32_t>(
                        std::to_integer<std::uint8_t>(frame.data()[i]))
                    << (8U * i);
      else
        box.TDHR |= static_cast<std::uint32_t>(
                        std::to_integer<std::uint8_t>(frame.data()[i]))
                    << (8U * (i - 4U));
    }
    box.TIR |= 1U;
    return result<bool, error_type>::success(true);
  }
  [[nodiscard]] auto try_receive()
      -> result<hal::can::polled_frame<hal::can::classic_frame>, error_type> {
    if (!initialized_)
      return failure<hal::can::polled_frame<hal::can::classic_frame>>(
          hal::can::error_kind::configuration);
    if ((registers_.RF0R & 3U) == 0U)
      return result<hal::can::polled_frame<hal::can::classic_frame>,
                    error_type>::
          success(
              hal::can::polled_frame<hal::can::classic_frame>::unavailable());
    const auto &box = registers_.sFIFOMailBox[0];
    const bool extended = (box.RIR & 4U) != 0U;
    const auto id = extended ? box.RIR >> 3U : box.RIR >> 21U;
    std::byte data[8U]{};
    const auto length = static_cast<std::uint8_t>(box.RDTR & 0xFU);
    if (length > 8U) {
      registers_.RF0R |= 1U << 5U;
      return failure<hal::can::polled_frame<hal::can::classic_frame>>(
          hal::can::error_kind::protocol_error);
    }
    for (std::size_t i = 0U; i < length; ++i)
      data[i] = static_cast<std::byte>(i < 4U ? (box.RDLR >> (8U * i)) & 0xFFU
                                              : (box.RDHR >> (8U * (i - 4U))) &
                                                    0xFFU);
    registers_.RF0R |= 1U << 5U;
    auto frame = hal::can::classic_frame::make({id, extended}, {data, length});
    if (!frame)
      return failure<hal::can::polled_frame<hal::can::classic_frame>>(
          hal::can::error_kind::io);
    return result<hal::can::polled_frame<hal::can::classic_frame>, error_type>::
        success(hal::can::polled_frame<hal::can::classic_frame>::available(
            frame.value()));
  }
  [[nodiscard]] auto set_filters(span<const hal::can::filter> filters)
      -> result<void, error_type> {
    if (filters.size() > 14U)
      return failure(hal::can::error_kind::configuration);
    if (!filters.valid())
      return failure(hal::can::error_kind::configuration);
    for (std::size_t i = 0U; i < filters.size(); ++i) {
      if (!filters[i].valid())
        return failure(hal::can::error_kind::configuration);
    }
    registers_.FMR |= 1U << 0U;
    registers_.FA1R = 0U;
    registers_.FS1R =
        filters.size() == 0U
            ? 0U
            : (static_cast<std::uint32_t>(1U << filters.size()) - 1U);
    for (std::size_t i = 0U; i < filters.size(); ++i) {
      registers_.sFilterRegister[i].FR1 = filter_word(filters[i].frame_id);
      registers_.sFilterRegister[i].FR2 = filter_mask_word(filters[i]);
      registers_.FA1R |= 1U << i;
    }
    registers_.FMR &= ~(1U << 0U);
    return result<void, error_type>::success();
  }

private:
  [[nodiscard]] auto failure(hal::can::error_kind k)
      -> result<void, error_type> {
    return result<void, error_type>::failure(error{k});
  }
  template <class T>
  [[nodiscard]] auto failure(hal::can::error_kind k) -> result<T, error_type> {
    return result<T, error_type>::failure(error{k});
  }
  [[nodiscard]] bool wait(std::uint32_t mask) const noexcept {
    for (std::uint32_t n = config_.timeout.iterations; n > 0U; --n)
      if ((registers_.MSR & mask) != 0U)
        return true;
    return false;
  }
  [[nodiscard]] bool wait_clear(std::uint32_t mask) const noexcept {
    for (std::uint32_t n = config_.timeout.iterations; n > 0U; --n)
      if ((registers_.MSR & mask) == 0U)
        return true;
    return false;
  }
  [[nodiscard]] static constexpr std::uint32_t
  filter_word(hal::can::id identifier) noexcept {
    return identifier.extended ? (identifier.value << 3U) | (1U << 2U)
                               : identifier.value << 21U;
  }
  [[nodiscard]] static constexpr std::uint32_t
  filter_mask_word(const hal::can::filter &filter) noexcept {
    return (filter.frame_id.extended ? (filter.mask << 3U)
                                     : (filter.mask << 21U)) |
           (1U << 2U);
  }
  Registers &registers_;
  config config_{};
  bool initialized_{};
};
} // namespace hal::stm32f4::can
