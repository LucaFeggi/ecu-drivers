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

namespace hal::stm32g4::can {

inline constexpr capability classic_support{
    implementation_status::available,
    "direct Bosch M_CAN/FDCAN message-RAM classic controller"};
inline constexpr capability fd_support{
    implementation_status::available,
    "direct FDCAN flexible-data-rate controller with fixed G4 message RAM"};

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
  std::uint16_t rx_fifo_elements{3U};
  std::uint16_t tx_fifo_elements{3U};
  poll_budget timeout{};
  bool fd_enabled{};
  bool bit_rate_switch{};
};

// STM32G4 uses the reduced FDCAN variant. The message RAM is fixed in the
// peripheral: 28 standard filters, 8 extended filters, three elements in
// each RX FIFO, a three-element TX event FIFO, and a three-element TX FIFO.
// The BSP supplies the controller's dedicated message-RAM base.
template <class Registers, unsigned Instance = 1U> class Controller {
  static_assert(Instance >= 1U && Instance <= 3U);

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
        (reinterpret_cast<std::uintptr_t>(message_ram_) % alignof(std::uint32_t)) !=
            0U ||
        !configuration_.timeout.valid() ||
        configuration_.nominal_bit_timing == 0U ||
        (configuration_.fd_enabled && configuration_.data_bit_timing == 0U) ||
        (!configuration_.fd_enabled && configuration_.bit_rate_switch) ||
        !layout_fits()) {
      return failure(hal::can::error_kind::configuration);
    }

    registers_.CCCR = registers_.CCCR & ~cccr_clock_stop_request;
    if (!wait_clear(cccr_clock_stop_acknowledge)) {
      return failure(hal::can::error_kind::io);
    }
    registers_.CCCR = registers_.CCCR | cccr_init;
    if (!wait_set(cccr_init)) return failure(hal::can::error_kind::io);
    registers_.CCCR = registers_.CCCR | cccr_config_change_enable;
    registers_.NBTP = configuration_.nominal_bit_timing;
    registers_.DBTP = configuration_.data_bit_timing;
    registers_.TSCC = configuration_.timestamp_prescaler;
    registers_.CCCR = registers_.CCCR & ~(cccr_fdoe | cccr_brse);
    if (configuration_.fd_enabled) {
      registers_.CCCR = registers_.CCCR | cccr_fdoe |
                        (configuration_.bit_rate_switch ? cccr_brse : 0U);
    }
    registers_.RXGFC = reject_remote_frames | reject_nonmatching_frames |
                       (static_cast<std::uint32_t>(
                            configuration_.standard_filter_count)
                        << rxgfc_standard_filter_count_position) |
                       (static_cast<std::uint32_t>(
                            configuration_.extended_filter_count)
                        << rxgfc_extended_filter_count_position);
    registers_.TXBC = 0U;
    registers_.IE = 0U;
    registers_.ILE = 0U;
    registers_.IR = interrupt_flags;
    clear_message_ram();
    registers_.CCCR = registers_.CCCR & ~cccr_init;
    if (!wait_clear(cccr_init)) return failure(hal::can::error_kind::io);
    initialized_ = true;
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto try_send(const hal::can::classic_frame &frame)
      -> result<bool, error_type> {
    if (!initialized_) return failure_bool(hal::can::error_kind::configuration);
    return write_tx(frame.frame_id(), frame.data(), frame.size(), false, false,
                    false);
  }

  [[nodiscard]] auto try_send_fd(const hal::can::fd_frame &frame)
      -> result<bool, error_type> {
    if (!initialized_ || !configuration_.fd_enabled) {
      return failure_bool(hal::can::error_kind::configuration);
    }
    return write_tx(frame.frame_id(), frame.data(), frame.dlc(), true,
                    frame.bit_rate_switch(), frame.error_state_indicator());
  }

  [[nodiscard]] auto try_receive()
      -> result<hal::can::polled_frame<hal::can::classic_frame>, error_type> {
    if (!initialized_) return failure_frame<hal::can::classic_frame>(
        hal::can::error_kind::configuration);
    if (const auto status = status_error();
        status != hal::can::error_kind::other) {
      return failure_frame<hal::can::classic_frame>(status);
    }
    if (fifo_fill() == 0U) {
      return result<hal::can::polled_frame<hal::can::classic_frame>, error_type>::
          success(hal::can::polled_frame<hal::can::classic_frame>::unavailable());
    }
    const std::uint32_t index = fifo_get_index();
    if (index >= rx_fifo_capacity) {
      return failure_frame<hal::can::classic_frame>(hal::can::error_kind::io);
    }
    volatile std::uint32_t *element =
        message_ram_ + rx_fifo0_offset + static_cast<std::size_t>(index) * element_words;
    const auto id = decode_id(element[0U]);
    const auto length = dlc_to_length(static_cast<std::uint8_t>(element[1U] >> 16U));
    std::array<std::byte, 64U> payload{};
    load_payload(element + 2U, payload, length);
    registers_.RXF0A = index;
    if (length > 8U) {
      return failure_frame<hal::can::classic_frame>(
          hal::can::error_kind::protocol_error);
    }
    const auto made = hal::can::classic_frame::make(id, {payload.data(), length});
    if (!made) {
      return failure_frame<hal::can::classic_frame>(
          hal::can::error_kind::protocol_error);
    }
    return result<hal::can::polled_frame<hal::can::classic_frame>, error_type>::
        success(hal::can::polled_frame<hal::can::classic_frame>::available(
            std::move(made).value()));
  }

  [[nodiscard]] auto try_receive_fd()
      -> result<hal::can::polled_frame<hal::can::fd_frame>, error_type> {
    if (!initialized_ || !configuration_.fd_enabled) {
      return failure_frame<hal::can::fd_frame>(hal::can::error_kind::configuration);
    }
    if (const auto status = status_error();
        status != hal::can::error_kind::other) {
      return failure_frame<hal::can::fd_frame>(status);
    }
    if (fifo_fill() == 0U) {
      return result<hal::can::polled_frame<hal::can::fd_frame>, error_type>::
          success(hal::can::polled_frame<hal::can::fd_frame>::unavailable());
    }
    const std::uint32_t index = fifo_get_index();
    if (index >= rx_fifo_capacity) {
      return failure_frame<hal::can::fd_frame>(hal::can::error_kind::io);
    }
    volatile std::uint32_t *element =
        message_ram_ + rx_fifo0_offset + static_cast<std::size_t>(index) * element_words;
    const auto id = decode_id(element[0U]);
    const auto control = element[1U];
    const auto length = dlc_to_length(static_cast<std::uint8_t>(control >> 16U));
    std::array<std::byte, 64U> payload{};
    load_payload(element + 2U, payload, length);
    registers_.RXF0A = index;
    const auto made = hal::can::fd_frame::make(
        id, {payload.data(), length}, (control & fd_brs) != 0U,
        (control & fd_esi) != 0U);
    if (!made) return failure_frame<hal::can::fd_frame>(
        hal::can::error_kind::protocol_error);
    return result<hal::can::polled_frame<hal::can::fd_frame>, error_type>::success(
        hal::can::polled_frame<hal::can::fd_frame>::available(
            std::move(made).value()));
  }

  [[nodiscard]] auto set_filters(hal::span<const hal::can::filter> filters)
      -> result<void, error_type> {
    if (!initialized_ || !filters.valid() ||
        filters.size() > static_cast<std::size_t>(
                             configuration_.standard_filter_count +
                             configuration_.extended_filter_count)) {
      return failure(hal::can::error_kind::configuration);
    }
    std::size_t standard = 0U;
    std::size_t extended = 0U;
    for (const auto &filter : filters) {
      if (!filter.valid()) return failure(hal::can::error_kind::configuration);
      if (filter.frame_id.extended) {
        if (extended >= configuration_.extended_filter_count) {
          return failure(hal::can::error_kind::configuration);
        }
        ++extended;
      } else {
        if (standard >= configuration_.standard_filter_count) {
          return failure(hal::can::error_kind::configuration);
        }
        ++standard;
      }
    }

    if (!enter_configuration()) return failure(hal::can::error_kind::io);
    for (std::size_t i = 0U; i < standard_filter_capacity; ++i) {
      message_ram_[standard_filter_offset + i] = 0U;
    }
    for (std::size_t i = 0U; i < extended_filter_capacity * 2U; ++i) {
      message_ram_[extended_filter_offset + i] = 0U;
    }
    standard = 0U;
    extended = 0U;
    for (const auto &filter : filters) {
      if (filter.frame_id.extended) {
        volatile std::uint32_t *entry =
            message_ram_ + extended_filter_offset + extended * 2U;
        entry[0U] = (filter.frame_id.value & extended_id_mask) | (1U << 29U);
        entry[1U] = (filter.mask & extended_id_mask) | (2U << 30U);
        ++extended;
      } else {
        volatile std::uint32_t *entry =
            message_ram_ + standard_filter_offset + standard;
        entry[0U] = ((filter.frame_id.value & standard_id_mask) << 16U) |
                    (filter.mask & standard_id_mask) | (2U << 30U) |
                    (1U << 27U);
        ++standard;
      }
    }
    barrier();
    if (!leave_configuration()) return failure(hal::can::error_kind::io);
    return result<void, error_type>::success();
  }

  [[nodiscard]] static constexpr unsigned instance() noexcept { return Instance; }

private:
  static constexpr std::size_t element_words{18U};
  static constexpr std::size_t standard_filter_capacity{28U};
  static constexpr std::size_t extended_filter_capacity{8U};
  static constexpr std::size_t rx_fifo_capacity{3U};
  static constexpr std::size_t tx_fifo_capacity{3U};
  static constexpr std::size_t standard_filter_offset{0U};
  static constexpr std::size_t extended_filter_offset{28U};
  static constexpr std::size_t rx_fifo0_offset{44U};
  static constexpr std::size_t rx_fifo1_offset{98U};
  static constexpr std::size_t tx_event_offset{152U};
  static constexpr std::size_t tx_fifo_offset{158U};
  static constexpr std::size_t message_ram_capacity{212U};
  static constexpr std::uint32_t standard_id_mask{0x7FFU};
  static constexpr std::uint32_t extended_id_mask{0x1FFF'FFFFU};
  static constexpr std::uint32_t cccr_init{1U << 0U};
  static constexpr std::uint32_t cccr_config_change_enable{1U << 1U};
  static constexpr std::uint32_t cccr_clock_stop_acknowledge{1U << 3U};
  static constexpr std::uint32_t cccr_clock_stop_request{1U << 4U};
  static constexpr std::uint32_t cccr_fdoe{1U << 8U};
  static constexpr std::uint32_t cccr_brse{1U << 9U};
  static constexpr std::uint32_t reject_remote_frames{(1U << 0U) | (1U << 1U)};
  static constexpr std::uint32_t reject_nonmatching_frames{(3U << 2U) | (3U << 4U)};
  static constexpr std::uint32_t rxgfc_standard_filter_count_position{16U};
  static constexpr std::uint32_t rxgfc_extended_filter_count_position{24U};
  static constexpr std::uint32_t fd_brs{1U << 20U};
  static constexpr std::uint32_t fd_esi{1U << 31U};
  static constexpr std::uint32_t tx_fifo_free_level_mask{0x07U};
  static constexpr std::uint32_t tx_fifo_full{1U << 21U};
  static constexpr std::uint32_t tx_fifo_put_index_position{16U};
  static constexpr std::uint32_t tx_fifo_index_mask{0x03U};
  static constexpr std::uint32_t ir_rx_fifo_lost{1U << 2U};
  static constexpr std::uint32_t ir_message_ram_failure{1U << 14U};
  static constexpr std::uint32_t ir_error_passive{1U << 17U};
  static constexpr std::uint32_t ir_bus_off{1U << 19U};
  static constexpr std::uint32_t ir_protocol_error{(1U << 21U) | (1U << 22U)};
  static constexpr std::uint32_t interrupt_flags{(1U << 24U) - 1U};

  [[nodiscard]] bool wait_set(std::uint32_t mask) const noexcept {
    for (std::uint32_t left = configuration_.timeout.iterations; left > 0U;
         --left) {
      if ((registers_.CCCR & mask) == mask) return true;
    }
    return false;
  }

  [[nodiscard]] bool wait_clear(std::uint32_t mask) const noexcept {
    for (std::uint32_t left = configuration_.timeout.iterations; left > 0U;
         --left) {
      if ((registers_.CCCR & mask) == 0U) return true;
    }
    return false;
  }

  [[nodiscard]] bool enter_configuration() noexcept {
    registers_.CCCR = registers_.CCCR | cccr_init;
    if (!wait_set(cccr_init)) return false;
    registers_.CCCR = registers_.CCCR | cccr_config_change_enable;
    return true;
  }

  [[nodiscard]] bool leave_configuration() noexcept {
    registers_.CCCR = registers_.CCCR & ~cccr_init;
    return wait_clear(cccr_init);
  }

  [[nodiscard]] bool layout_fits() const noexcept {
    return configuration_.standard_filter_count <= standard_filter_capacity &&
           configuration_.extended_filter_count <= extended_filter_capacity &&
           configuration_.rx_fifo_elements == rx_fifo_capacity &&
           configuration_.tx_fifo_elements == tx_fifo_capacity &&
           configuration_.message_ram_offset_words == 0U &&
           message_ram_words_ >= message_ram_capacity;
  }

  [[nodiscard]] std::uint32_t fifo_fill() const noexcept {
    return registers_.RXF0S & 0x0FU;
  }

  [[nodiscard]] std::uint32_t fifo_get_index() const noexcept {
    return (registers_.RXF0S >> 8U) & 0x03U;
  }

  [[nodiscard]] hal::can::error_kind status_error() noexcept {
    const std::uint32_t status = registers_.IR;
    if ((status & ir_bus_off) != 0U) {
      registers_.IR = ir_bus_off;
      return hal::can::error_kind::bus_off;
    }
    if ((status & ir_rx_fifo_lost) != 0U) {
      registers_.IR = ir_rx_fifo_lost;
      return hal::can::error_kind::rx_overflow;
    }
    if ((status & ir_message_ram_failure) != 0U) {
      registers_.IR = ir_message_ram_failure;
      return hal::can::error_kind::io;
    }
    if ((status & (ir_protocol_error | ir_error_passive)) != 0U) {
      registers_.IR = status & (ir_protocol_error | ir_error_passive);
      return hal::can::error_kind::protocol_error;
    }
    return hal::can::error_kind::other;
  }

  [[nodiscard]] auto write_tx(hal::can::id id, hal::span<const std::byte> data,
                              std::uint8_t dlc, bool fd, bool brs, bool esi)
      -> result<bool, error_type> {
    const auto status = registers_.TXFQS;
    if ((status & tx_fifo_full) != 0U ||
        (status & tx_fifo_free_level_mask) == 0U) {
      return result<bool, error_type>::success(false);
    }
    const std::uint32_t index =
        (status >> tx_fifo_put_index_position) & tx_fifo_index_mask;
    if (index >= configuration_.tx_fifo_elements) {
      return failure_bool(hal::can::error_kind::io);
    }
    volatile std::uint32_t *element =
        message_ram_ + tx_fifo_offset + static_cast<std::size_t>(index) * element_words;
    element[0U] = id.extended ? id.value | (1U << 30U) : id.value << 18U;
    element[1U] = static_cast<std::uint32_t>(dlc) << 16U |
                  (fd ? 1U << 21U : 0U) | (brs ? fd_brs : 0U) |
                  (esi ? fd_esi : 0U);
    for (std::size_t word = 0U; word < (data.size() + 3U) / 4U; ++word) {
      std::uint32_t value = 0U;
      for (std::size_t byte = 0U; byte < 4U; ++byte) {
        const std::size_t index = word * 4U + byte;
        if (index < data.size()) {
          value |= static_cast<std::uint32_t>(
                       std::to_integer<std::uint8_t>(data[index]))
                   << (byte * 8U);
        }
      }
      element[2U + word] = value;
    }
    barrier();
    registers_.TXBAR = 1U << index;
    return result<bool, error_type>::success(true);
  }

  [[nodiscard]] static hal::can::id decode_id(std::uint32_t word) noexcept {
    return (word & (1U << 30U)) != 0U
               ? hal::can::id{word & extended_id_mask, true}
               : hal::can::id{(word >> 18U) & standard_id_mask, false};
  }

  [[nodiscard]] static constexpr std::uint8_t dlc_to_length(
      std::uint8_t dlc) noexcept {
    constexpr std::uint8_t lengths[] = {0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U,
                                        8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U};
    return lengths[dlc & 0x0FU];
  }

  static void load_payload(const volatile std::uint32_t *source,
                           std::array<std::byte, 64U> &destination,
                           std::uint8_t length) noexcept {
    for (std::size_t i = 0U; i < length; ++i) {
      destination[i] = static_cast<std::byte>(
          (source[i / 4U] >> ((i % 4U) * 8U)) & 0xFFU);
    }
  }

  void clear_message_ram() noexcept {
    for (std::size_t i = 0U; i < message_ram_capacity; ++i) {
      message_ram_[i] = 0U;
    }
    barrier();
  }

  static void barrier() noexcept {
#if defined(__arm__) || defined(__thumb__)
    __asm volatile("dmb 0xF" ::: "memory");
#else
    std::atomic_thread_fence(std::memory_order_seq_cst);
#endif
  }

  [[nodiscard]] auto failure(hal::can::error_kind kind)
      -> result<void, error_type> {
    return result<void, error_type>::failure(error{kind});
  }

  [[nodiscard]] static auto failure_bool(hal::can::error_kind kind)
      -> result<bool, error_type> {
    return result<bool, error_type>::failure(error{kind});
  }

  template <class Frame>
  [[nodiscard]] static auto failure_frame(hal::can::error_kind kind)
      -> result<hal::can::polled_frame<Frame>, error_type> {
    return result<hal::can::polled_frame<Frame>, error_type>::failure(error{kind});
  }

  Registers &registers_;
  volatile std::uint32_t *message_ram_{};
  std::size_t message_ram_words_{};
  config configuration_{};
  bool initialized_{};
};

} // namespace hal::stm32g4::can
