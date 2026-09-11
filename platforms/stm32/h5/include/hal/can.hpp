#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/can.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include "support.hpp"
#include <utility>

namespace hal::stm32h5::can {

inline constexpr capability classic_support{
    implementation_status::available,
    "direct Bosch M_CAN/FDCAN message-RAM controller"};
inline constexpr capability fd_support{
    implementation_status::available,
    "direct FDCAN flexible-data-rate controller with hardware FIFOs"};

class error {
public:
  explicit constexpr error(hal::can::error_kind kind) noexcept : kind_{kind} {}

  [[nodiscard]] constexpr hal::can::error_kind kind() const noexcept {
    return kind_;
  }

private:
  hal::can::error_kind kind_{};
};

struct config {
  std::uint32_t nominal_bit_timing{};
  std::uint32_t data_bit_timing{};
  std::uint32_t timestamp_prescaler{};
  std::uint32_t message_ram_offset_words{};
  std::uint16_t standard_filter_count{1U};
  std::uint16_t extended_filter_count{1U};
  std::uint16_t rx_fifo_elements{8U};
  std::uint16_t tx_fifo_elements{8U};
  poll_budget timeout{};
  bool fd_enabled{};
  bool bit_rate_switch{};
};

// FDCAN does not use the STM32 DMA controllers for normal frame movement: its
// dedicated message RAM is the data plane. The driver therefore uses the
// hardware RX/TX FIFOs and only exposes nonblocking operations, matching the
// hal-core CAN contract exactly. The BSP owns the shared message-RAM storage.
template <class Registers, unsigned Instance = 1U> class Controller {
public:
  using error_type = error;

  Controller(Registers &registers, volatile std::uint32_t *message_ram,
             std::size_t message_ram_words, config configuration) noexcept
      : registers_{registers}, message_ram_{message_ram},
        message_ram_words_{message_ram_words}, configuration_{configuration} {}

  Controller(const Controller &) = delete;
  Controller &operator=(const Controller &) = delete;

  [[nodiscard]] auto initialize() -> result<void, error_type> {
    if (message_ram_ == nullptr ||
        (reinterpret_cast<std::uintptr_t>(message_ram_) %
         alignof(std::uint32_t)) != 0U ||
        !configuration_.timeout.valid() ||
        configuration_.nominal_bit_timing == 0U ||
        (configuration_.fd_enabled && configuration_.data_bit_timing == 0U) ||
        (!configuration_.fd_enabled && configuration_.bit_rate_switch) ||
        !layout_fits()) {
      return failure(hal::can::error_kind::configuration);
    }
    registers_.CCCR = registers_.CCCR | cccr_init;
    if (!wait_set(cccr_init)) {
      return failure(hal::can::error_kind::io);
    }
    registers_.CCCR = registers_.CCCR | cccr_config_change_enable;
    registers_.NBTP = configuration_.nominal_bit_timing;
    registers_.DBTP = configuration_.data_bit_timing;
    registers_.TSCC = configuration_.timestamp_prescaler;
    registers_.CCCR =
        registers_.CCCR & ~(cccr_fd_operation | cccr_bit_rate_switch);
    if (configuration_.fd_enabled) {
      registers_.CCCR =
          registers_.CCCR | cccr_fd_operation |
          (configuration_.bit_rate_switch ? cccr_bit_rate_switch : 0U);
    }
    // STM32H5 implements the reduced M_CAN variant. Its message-RAM blocks
    // are fixed in silicon; unlike STM32H7, there are no SIDFC/XIDFC/RXF0C
    // register fields to program.
    registers_.RXGFC = reject_remote_frames | reject_nonmatching_frames;
    registers_.TXBC = 0U;
    // The core API is deliberately nonblocking/polled. Do not enable an IRQ
    // source for which this class does not provide an ISR dispatcher; the BSP
    // can add that policy around the same message-RAM engine later.
    registers_.IE = 0U;
    registers_.ILE = 0U;
    registers_.CCCR = registers_.CCCR & ~cccr_init;
    if (!wait_clear(cccr_init)) {
      return failure(hal::can::error_kind::io);
    }
    initialized_ = true;
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto try_send(const hal::can::classic_frame &frame)
      -> result<bool, error_type> {
    if (!initialized_) {
      return result<bool, error_type>::failure(
          error{hal::can::error_kind::configuration});
    }
    return write_tx(frame.frame_id(), frame.data(), frame.size(), false, false,
                    false);
  }

  [[nodiscard]] auto try_send_fd(const hal::can::fd_frame &frame)
      -> result<bool, error_type> {
    if (!initialized_ || !configuration_.fd_enabled) {
      return result<bool, error_type>::failure(
          error{hal::can::error_kind::configuration});
    }
    return write_tx(frame.frame_id(), frame.data(), frame.dlc(), true,
                    frame.bit_rate_switch(), frame.error_state_indicator());
  }

  [[nodiscard]] auto try_receive()
      -> result<hal::can::polled_frame<hal::can::classic_frame>, error_type> {
    if (!initialized_) {
      return result<hal::can::polled_frame<hal::can::classic_frame>,
                    error_type>::failure(error{
          hal::can::error_kind::configuration});
    }
    if (const auto fault = status_error();
        fault != hal::can::error_kind::other) {
      return result<hal::can::polled_frame<hal::can::classic_frame>,
                    error_type>::failure(error{fault});
    }
    if (fifo_fill() == 0U) {
      return result<hal::can::polled_frame<hal::can::classic_frame>,
                    error_type>::
          success(
              hal::can::polled_frame<hal::can::classic_frame>::unavailable());
    }
    const std::uint32_t index = fifo_get_index();
    if (index >= configuration_.rx_fifo_elements) {
      return result<hal::can::polled_frame<hal::can::classic_frame>,
                    error_type>::failure(error{hal::can::error_kind::io});
    }
    volatile std::uint32_t *element =
        message_ram_ + rx_fifo_offset() +
        static_cast<std::size_t>(index) * element_words;
    const auto frame_id = decode_id(element[0U]);
    const std::uint8_t length =
        dlc_to_length(static_cast<std::uint8_t>((element[1U] >> 16U) & 0xFU));
    std::array<std::byte, 64U> payload{};
    load_payload(element + 2U, payload, length);
    registers_.RXF0A = index;
    if (length > 8U) {
      return result<hal::can::polled_frame<hal::can::classic_frame>,
                    error_type>::failure(error{
          hal::can::error_kind::protocol_error});
    }
    const auto made =
        hal::can::classic_frame::make(frame_id, {payload.data(), length});
    if (!made) {
      return result<hal::can::polled_frame<hal::can::classic_frame>,
                    error_type>::failure(error{
          hal::can::error_kind::protocol_error});
    }
    return result<hal::can::polled_frame<hal::can::classic_frame>, error_type>::
        success(hal::can::polled_frame<hal::can::classic_frame>::available(
            std::move(made).value()));
  }

  [[nodiscard]] auto try_receive_fd()
      -> result<hal::can::polled_frame<hal::can::fd_frame>, error_type> {
    if (!initialized_ || !configuration_.fd_enabled) {
      return result<hal::can::polled_frame<hal::can::fd_frame>,
                    error_type>::failure(error{
          hal::can::error_kind::configuration});
    }
    if (const auto fault = status_error();
        fault != hal::can::error_kind::other) {
      return result<hal::can::polled_frame<hal::can::fd_frame>,
                    error_type>::failure(error{fault});
    }
    if (fifo_fill() == 0U) {
      return result<hal::can::polled_frame<hal::can::fd_frame>, error_type>::
          success(hal::can::polled_frame<hal::can::fd_frame>::unavailable());
    }
    const std::uint32_t index = fifo_get_index();
    if (index >= configuration_.rx_fifo_elements) {
      return result<hal::can::polled_frame<hal::can::fd_frame>,
                    error_type>::failure(error{hal::can::error_kind::io});
    }
    volatile std::uint32_t *element =
        message_ram_ + rx_fifo_offset() +
        static_cast<std::size_t>(index) * element_words;
    const auto frame_id = decode_id(element[0U]);
    const std::uint8_t dlc =
        static_cast<std::uint8_t>((element[1U] >> 16U) & 0xFU);
    const std::uint8_t length = dlc_to_length(dlc);
    std::array<std::byte, 64U> payload{};
    load_payload(element + 2U, payload, length);
    registers_.RXF0A = index;
    const auto made = hal::can::fd_frame::make(
        frame_id, {payload.data(), length}, (element[1U] & fd_brs) != 0U,
        (element[1U] & fd_esi) != 0U);
    if (!made) {
      return result<hal::can::polled_frame<hal::can::fd_frame>,
                    error_type>::failure(error{
          hal::can::error_kind::protocol_error});
    }
    return result<hal::can::polled_frame<hal::can::fd_frame>, error_type>::
        success(hal::can::polled_frame<hal::can::fd_frame>::available(
            std::move(made).value()));
  }

  [[nodiscard]] auto set_filters(hal::span<const hal::can::filter> filters)
      -> result<void, error_type> {
    if (!initialized_ || !filters.valid() ||
        filters.size() >
            static_cast<std::size_t>(configuration_.standard_filter_count +
                                     configuration_.extended_filter_count)) {
      return result<void, error_type>::failure(
          error{hal::can::error_kind::configuration});
    }
    std::size_t standard = 0U;
    std::size_t extended = 0U;
    for (const auto &filter : filters) {
      if (!filter.valid()) {
        return result<void, error_type>::failure(
            error{hal::can::error_kind::configuration});
      }
      if (filter.frame_id.extended) {
        if (extended >= configuration_.extended_filter_count) {
          return result<void, error_type>::failure(
              error{hal::can::error_kind::configuration});
        }
        ++extended;
      } else {
        if (standard >= configuration_.standard_filter_count) {
          return result<void, error_type>::failure(
              error{hal::can::error_kind::configuration});
        }
        ++standard;
      }
    }

    if (!enter_configuration()) {
      return result<void, error_type>::failure(error{hal::can::error_kind::io});
    }
    for (std::size_t index = 0U; index < configuration_.standard_filter_count;
         ++index) {
      message_ram_[standard_filter_offset() + index] = 0U;
    }
    for (std::size_t index = 0U;
         index < configuration_.extended_filter_count * 2U; ++index) {
      message_ram_[extended_filter_offset() + index] = 0U;
    }
    standard = 0U;
    extended = 0U;
    for (const auto &filter : filters) {
      if (filter.frame_id.extended) {
        volatile std::uint32_t *entry =
            message_ram_ + extended_filter_offset() + extended * 2U;
        // Extended filter: EFID1 in word 0, EFID2 in word 1. EFT=classic
        // mask and EFEC=route to FIFO0.
        entry[0U] = (filter.frame_id.value & extended_id_mask) | (1U << 29U);
        entry[1U] = (filter.mask & extended_id_mask) | (2U << 30U);
        ++extended;
      } else {
        volatile std::uint32_t *entry =
            message_ram_ + standard_filter_offset() + standard;
        entry[0U] = ((filter.frame_id.value & standard_id_mask) << 16U) |
                    (filter.mask & standard_id_mask) | (2U << 30U) |
                    (1U << 27U);
        ++standard;
      }
    }
    barrier();
    if (!leave_configuration()) {
      return result<void, error_type>::failure(error{hal::can::error_kind::io});
    }
    return result<void, error_type>::success();
  }

  [[nodiscard]] static constexpr unsigned instance() noexcept {
    return Instance;
  }

private:
  static constexpr std::size_t element_words{18U};
  static constexpr std::size_t fixed_standard_filter_count{28U};
  static constexpr std::size_t fixed_extended_filter_count{8U};
  static constexpr std::size_t fixed_rx_fifo_elements{3U};
  static constexpr std::size_t fixed_tx_fifo_elements{3U};
  static constexpr std::size_t fixed_extended_filter_offset{28U};
  static constexpr std::size_t fixed_rx_fifo0_offset{44U};
  static constexpr std::size_t fixed_tx_fifo_offset{158U};
  static constexpr std::size_t fixed_message_ram_words{212U};
  static constexpr std::uint32_t standard_id_mask{0x7FFU};
  static constexpr std::uint32_t extended_id_mask{0x1FFF'FFFFU};
  static constexpr std::uint32_t cccr_init{1U << 0U};
  static constexpr std::uint32_t cccr_config_change_enable{1U << 1U};
  static constexpr std::uint32_t cccr_fd_operation{1U << 8U};
  static constexpr std::uint32_t cccr_bit_rate_switch{1U << 9U};
  static constexpr std::uint32_t reject_remote_frames{(1U << 0U) | (1U << 1U)};
  static constexpr std::uint32_t reject_nonmatching_frames{(2U << 2U) |
                                                           (2U << 4U)};
  static constexpr std::uint32_t interrupt_bus_off{1U << 25U};
  static constexpr std::uint32_t interrupt_error_passive{1U << 23U};
  static constexpr std::uint32_t interrupt_rx_fifo_lost{1U << 3U};
  static constexpr std::uint32_t interrupt_message_ram_failure{1U << 17U};
  static constexpr std::uint32_t interrupt_protocol_error{(1U << 27U) |
                                                          (1U << 28U)};
  static constexpr std::uint32_t interrupt_line0_enable{1U};
  static constexpr std::uint32_t fd_brs{1U << 20U};
  static constexpr std::uint32_t fd_esi{1U << 31U};
  static constexpr std::uint32_t tx_fifo_free_level_mask{0x3FU};
  static constexpr std::uint32_t tx_fifo_full{1U << 21U};
  static constexpr std::uint32_t tx_fifo_put_index_position{16U};
  static constexpr std::uint32_t tx_fifo_index_mask{0x1FU};

  [[nodiscard]] bool wait_set(std::uint32_t mask) const noexcept {
    for (std::uint32_t remaining = configuration_.timeout.iterations;
         remaining > 0U; --remaining) {
      if ((registers_.CCCR & mask) == mask) {
        return true;
      }
    }
    return false;
  }

  [[nodiscard]] bool wait_clear(std::uint32_t mask) const noexcept {
    for (std::uint32_t remaining = configuration_.timeout.iterations;
         remaining > 0U; --remaining) {
      if ((registers_.CCCR & mask) == 0U) {
        return true;
      }
    }
    return false;
  }

  [[nodiscard]] bool enter_configuration() noexcept {
    registers_.CCCR = registers_.CCCR | cccr_init;
    if (!wait_set(cccr_init)) {
      return false;
    }
    registers_.CCCR = registers_.CCCR | cccr_config_change_enable;
    return true;
  }

  [[nodiscard]] bool leave_configuration() noexcept {
    registers_.CCCR = registers_.CCCR & ~cccr_init;
    return wait_clear(cccr_init);
  }

  static void barrier() noexcept {
#if defined(__arm__) || defined(__thumb__)
    __asm volatile("dsb 0xF" ::: "memory");
#else
    std::atomic_thread_fence(std::memory_order_seq_cst);
#endif
  }

  [[nodiscard]] bool layout_fits() const noexcept {
    if (configuration_.standard_filter_count > fixed_standard_filter_count ||
        configuration_.extended_filter_count > fixed_extended_filter_count ||
        configuration_.rx_fifo_elements > fixed_rx_fifo_elements ||
        configuration_.tx_fifo_elements > fixed_tx_fifo_elements ||
        configuration_.message_ram_offset_words != 0U ||
        message_ram_words_ < fixed_message_ram_words) {
      return false;
    }
    return true;
  }

  [[nodiscard]] constexpr std::size_t standard_filter_offset() const noexcept {
    return 0U;
  }
  [[nodiscard]] constexpr std::size_t extended_filter_offset() const noexcept {
    return fixed_extended_filter_offset;
  }
  [[nodiscard]] constexpr std::size_t rx_fifo_offset() const noexcept {
    return fixed_rx_fifo0_offset;
  }
  [[nodiscard]] constexpr std::size_t tx_fifo_offset() const noexcept {
    return fixed_tx_fifo_offset;
  }

  [[nodiscard]] std::uint32_t fifo_status() const noexcept {
    return registers_.RXF0S;
  }
  [[nodiscard]] std::uint32_t fifo_fill() const noexcept {
    return fifo_status() & 0x7FU;
  }
  [[nodiscard]] std::uint32_t fifo_get_index() const noexcept {
    return (fifo_status() >> 8U) & 0x3FU;
  }

  [[nodiscard]] hal::can::error_kind status_error() noexcept {
    const std::uint32_t status = registers_.IR;
    if ((status & interrupt_bus_off) != 0U) {
      registers_.IR = interrupt_bus_off;
      return hal::can::error_kind::bus_off;
    }
    if ((status & interrupt_rx_fifo_lost) != 0U) {
      registers_.IR = interrupt_rx_fifo_lost;
      return hal::can::error_kind::rx_overflow;
    }
    if ((status & interrupt_message_ram_failure) != 0U) {
      registers_.IR = interrupt_message_ram_failure;
      return hal::can::error_kind::io;
    }
    if ((status & (interrupt_protocol_error | interrupt_error_passive)) != 0U) {
      registers_.IR =
          status & (interrupt_protocol_error | interrupt_error_passive);
      return hal::can::error_kind::protocol_error;
    }
    return hal::can::error_kind::other;
  }

  [[nodiscard]] auto write_tx(hal::can::id id, hal::span<const std::byte> data,
                              std::uint8_t dlc, bool fd, bool brs, bool esi)
      -> result<bool, error_type> {
    const std::uint32_t status = registers_.TXFQS;
    if ((status & tx_fifo_full) != 0U ||
        (status & tx_fifo_free_level_mask) == 0U) {
      return result<bool, error_type>::success(false);
    }
    const std::uint32_t index =
        (status >> tx_fifo_put_index_position) & tx_fifo_index_mask;
    if (index >= configuration_.tx_fifo_elements) {
      return result<bool, error_type>::failure(error{hal::can::error_kind::io});
    }
    volatile std::uint32_t *element =
        message_ram_ + tx_fifo_offset() +
        static_cast<std::size_t>(index) * element_words;
    element[0U] = id.extended ? id.value | (1U << 30U) : id.value << 18U;
    element[1U] = static_cast<std::uint32_t>(dlc) << 16U |
                  (fd ? 1U << 21U : 0U) | (brs ? 1U << 20U : 0U) |
                  (esi ? fd_esi : 0U);
    for (std::size_t index_byte = 0U; index_byte < data.size(); ++index_byte) {
      const std::size_t word = index_byte / 4U;
      const std::size_t shift = (index_byte % 4U) * 8U;
      if (shift == 0U) {
        element[2U + word] = 0U;
      }
      element[2U + word] |= static_cast<std::uint32_t>(
                                std::to_integer<std::uint8_t>(data[index_byte]))
                            << shift;
    }
    barrier();
    registers_.TXBAR = 1U << index;
    return result<bool, error_type>::success(true);
  }

  [[nodiscard]] static hal::can::id decode_id(std::uint32_t word) noexcept {
    if ((word & (1U << 30U)) != 0U) {
      return {word & 0x1FFF'FFFFU, true};
    }
    return {(word >> 18U) & 0x7FFU, false};
  }

  [[nodiscard]] static constexpr std::uint8_t
  dlc_to_length(std::uint8_t dlc) noexcept {
    constexpr std::uint8_t lengths[] = {0U, 1U,  2U,  3U,  4U,  5U,  6U,  7U,
                                        8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U};
    return lengths[dlc & 0xFU];
  }

  static void load_payload(const volatile std::uint32_t *source,
                           std::array<std::byte, 64U> &destination,
                           std::uint8_t length) noexcept {
    for (std::size_t index = 0U; index < length; ++index) {
      destination[index] = static_cast<std::byte>(
          (source[index / 4U] >> ((index % 4U) * 8U)) & 0xFFU);
    }
  }

  [[nodiscard]] auto failure(hal::can::error_kind kind)
      -> result<void, error_type> {
    return result<void, error_type>::failure(error{kind});
  }

  Registers &registers_;
  volatile std::uint32_t *message_ram_{};
  std::size_t message_ram_words_{};
  config configuration_{};
  bool initialized_{};
};

} // namespace hal::stm32h5::can
