#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/can.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif
#include <hal/foundation/result.hpp>
#include <hal/foundation/span.hpp>
#include <hal/foundation/units.hpp>
#include <soc/twai_struct.h>

namespace hal::esp32s3_wroom_1_n16r8::can {

class error {
 public:
  [[nodiscard]] static constexpr error configuration() noexcept {
    return error{hal::can::error_kind::configuration};
  }
  [[nodiscard]] static constexpr error bus_off() noexcept {
    return error{hal::can::error_kind::bus_off};
  }
  [[nodiscard]] static constexpr error arbitration_lost() noexcept {
    return error{hal::can::error_kind::arbitration_lost};
  }
  [[nodiscard]] static constexpr error protocol() noexcept {
    return error{hal::can::error_kind::protocol_error};
  }
  [[nodiscard]] static constexpr error rx_overflow() noexcept {
    return error{hal::can::error_kind::rx_overflow};
  }
  [[nodiscard]] constexpr hal::can::error_kind kind() const noexcept {
    return kind_;
  }

 private:
  constexpr explicit error(hal::can::error_kind kind) noexcept : kind_{kind} {}
  hal::can::error_kind kind_;
};

struct config {
  hertz bitrate{500'000U};
  std::uint16_t sample_point_per_mille{800U};
  std::uint32_t max_error_ppm{2'000U};
  bool listen_only{};
  bool self_test{};
};

struct timing {
  std::uint16_t prescaler{};
  std::uint8_t time_segment_one{};
  std::uint8_t time_segment_two{};
  std::uint8_t synchronization_jump_width{1U};
  std::uint32_t bitrate{};
};

[[nodiscard]] constexpr std::uint32_t byte_swap32(std::uint32_t value) noexcept {
  return ((value & 0x000000FFU) << 24U) |
         ((value & 0x0000FF00U) << 8U) |
         ((value & 0x00FF0000U) >> 8U) |
         ((value & 0xFF000000U) >> 24U);
}

[[nodiscard]] constexpr timing find_timing(
    config configuration, std::uint32_t source_hz = 80'000'000U) noexcept {
  timing selected{};
  std::uint64_t best_bitrate_error = UINT64_MAX;
  std::uint32_t best_sample_error = UINT32_MAX;
  if (source_hz == 0U || configuration.bitrate.value == 0U ||
      configuration.sample_point_per_mille == 0U ||
      configuration.sample_point_per_mille >= 1'000U) {
    return selected;
  }

  for (std::uint32_t segment_one = 1U; segment_one <= 16U;
       ++segment_one) {
    for (std::uint32_t segment_two = 1U; segment_two <= 8U;
         ++segment_two) {
      const std::uint32_t time_quanta = segment_one + segment_two + 1U;
      const std::uint64_t denominator =
          configuration.bitrate.value * time_quanta;
      std::uint32_t nearest = static_cast<std::uint32_t>(
          (static_cast<std::uint64_t>(source_hz) + denominator / 2U) /
          denominator);
      if ((nearest & 1U) != 0U) {
        ++nearest;
      }
      const std::uint32_t candidates[2U]{nearest,
                                         nearest >= 4U ? nearest - 2U : 0U};
      for (const std::uint32_t prescaler : candidates) {
        if (prescaler < 2U || prescaler > 16'384U) {
          continue;
        }
        const std::uint32_t actual = source_hz / (prescaler * time_quanta);
        if (actual == 0U) {
          continue;
        }
        const std::uint64_t bitrate_error =
            actual > configuration.bitrate.value
                ? static_cast<std::uint64_t>(actual - configuration.bitrate.value)
                : static_cast<std::uint64_t>(configuration.bitrate.value - actual);
        const std::uint32_t sample_point =
            ((segment_one + 1U) * 1'000U) / time_quanta;
        const std::uint32_t sample_error =
            sample_point > configuration.sample_point_per_mille
                ? sample_point - configuration.sample_point_per_mille
                : configuration.sample_point_per_mille - sample_point;
        if (bitrate_error < best_bitrate_error ||
            (bitrate_error == best_bitrate_error &&
             sample_error < best_sample_error)) {
          best_bitrate_error = bitrate_error;
          best_sample_error = sample_error;
          selected = {static_cast<std::uint16_t>(prescaler),
                      static_cast<std::uint8_t>(segment_one),
                      static_cast<std::uint8_t>(segment_two), 1U, actual};
        }
      }
    }
  }
  if (selected.bitrate == 0U) {
    return selected;
  }
  const std::uint64_t error_ppm =
      (best_bitrate_error * 1'000'000ULL) / configuration.bitrate.value;
  if (error_ppm > configuration.max_error_ppm) {
    return {};
  }
  return selected;
}

template <std::uint32_t SourceHz = 80'000'000U>
class Controller {
 public:
  using error_type = error;

  explicit Controller(twai_dev_t& peripheral, config configuration = {}) noexcept
      : peripheral_{&peripheral}, configuration_{configuration} {}

  Controller(const Controller&) = delete;
  Controller& operator=(const Controller&) = delete;

  [[nodiscard]] result<hertz, error_type> configure(config configuration) noexcept {
    const timing selected = find_timing(configuration, SourceHz);
    if (selected.bitrate == 0U) {
      return result<hertz, error_type>::failure(error_type::configuration());
    }

    peripheral_->mode_reg.rm = 1U;
    peripheral_->mode_reg.lom = configuration.listen_only ? 1U : 0U;
    peripheral_->mode_reg.stm = configuration.self_test ? 1U : 0U;
    peripheral_->bus_timing_0_reg.val =
        (peripheral_->bus_timing_0_reg.val & ~((0x1FFFU << 0U) |
                                               (0x3U << 14U))) |
        ((static_cast<std::uint32_t>(selected.prescaler / 2U - 1U) &
          0x1FFFU) << 0U) |
        ((static_cast<std::uint32_t>(selected.synchronization_jump_width - 1U) &
          0x3U) << 14U);
    peripheral_->bus_timing_1_reg.val =
        (peripheral_->bus_timing_1_reg.val & ~0x7FU) |
        ((static_cast<std::uint32_t>(selected.time_segment_one - 1U)) &
         0x0FU) |
        ((static_cast<std::uint32_t>(selected.time_segment_two - 1U) & 0x07U)
         << 4U);
    peripheral_->error_warning_limit_reg.ewl = 96U;
    peripheral_->tx_error_counter_reg.txerr = 0U;
    peripheral_->rx_error_counter_reg.rxerr = 0U;
    peripheral_->interrupt_enable_reg.val =
        (1U << 0U) | (1U << 1U) | (1U << 2U) | (1U << 5U) | (1U << 6U) |
        (1U << 7U);
    set_accept_all_filter();
    (void)peripheral_->interrupt_reg.val;
    configuration_ = configuration;
    configured_ = true;
    peripheral_->mode_reg.rm = 0U;
    return result<hertz, error_type>::success(hertz{selected.bitrate});
  }

  [[nodiscard]] result<bool, error_type> try_send(
      const hal::can::classic_frame& frame) noexcept {
    if (!configured_) {
      return result<bool, error_type>::failure(error_type::configuration());
    }
    if ((peripheral_->status_reg.val & (1U << 7U)) != 0U) {
      return result<bool, error_type>::failure(error_type::bus_off());
    }
    if ((peripheral_->status_reg.val & (1U << 2U)) == 0U) {
      return result<bool, error_type>::success(false);
    }

    const hal::can::id frame_id = frame.frame_id();
    const std::uint8_t size = frame.size();
    peripheral_->tx_rx_buffer[0].byte = size;
    if (frame_id.extended) {
      peripheral_->tx_rx_buffer[0].byte = static_cast<std::uint8_t>(size | 0x80U);
      const std::uint32_t encoded = frame_id.value << 3U;
      const std::uint32_t swapped = byte_swap32(encoded);
      for (std::uint32_t index = 0U; index < 4U; ++index) {
        peripheral_->tx_rx_buffer[index + 1U].byte =
            static_cast<std::uint8_t>(swapped >> (index * 8U));
      }
      for (std::uint32_t index = 0U; index < size; ++index) {
        peripheral_->tx_rx_buffer[index + 5U].byte =
            std::to_integer<std::uint8_t>(frame.data()[index]);
      }
    } else {
      const std::uint16_t encoded = static_cast<std::uint16_t>(frame_id.value << 5U);
      peripheral_->tx_rx_buffer[1].byte = static_cast<std::uint8_t>(encoded >> 8U);
      peripheral_->tx_rx_buffer[2].byte = static_cast<std::uint8_t>(encoded);
      for (std::uint32_t index = 0U; index < size; ++index) {
        peripheral_->tx_rx_buffer[index + 3U].byte =
            std::to_integer<std::uint8_t>(frame.data()[index]);
      }
    }
    peripheral_->command_reg.tr = 1U;
    return result<bool, error_type>::success(true);
  }

  [[nodiscard]] result<hal::can::polled_frame<hal::can::classic_frame>,
                       error_type>
  try_receive() noexcept {
    if (!configured_) {
      return result<hal::can::polled_frame<hal::can::classic_frame>, error_type>::failure(
          error_type::configuration());
    }
    if ((peripheral_->status_reg.val & (1U << 7U)) != 0U) {
      return result<hal::can::polled_frame<hal::can::classic_frame>, error_type>::failure(
          error_type::bus_off());
    }
    if ((peripheral_->status_reg.val & (1U << 1U)) != 0U) {
      peripheral_->command_reg.cdo = 1U;
      peripheral_->command_reg.rrb = 1U;
      return result<hal::can::polled_frame<hal::can::classic_frame>, error_type>::failure(
          error_type::rx_overflow());
    }
    if ((peripheral_->status_reg.val & 1U) == 0U) {
      return result<hal::can::polled_frame<hal::can::classic_frame>, error_type>::success(
          hal::can::polled_frame<hal::can::classic_frame>::unavailable());
    }

    std::array<std::uint8_t, 13U> frame{};
    for (std::uint32_t index = 0U; index < frame.size(); ++index) {
      frame[index] = peripheral_->tx_rx_buffer[index].byte;
    }
    peripheral_->command_reg.rrb = 1U;
    const bool extended = (frame[0] & 0x80U) != 0U;
    const bool remote = (frame[0] & 0x40U) != 0U;
    if (remote) {
      return result<hal::can::polled_frame<hal::can::classic_frame>, error_type>::failure(
          error_type::protocol());
    }
    hal::can::id frame_id{};
    std::size_t data_offset = 0U;
    if (extended) {
      frame_id = {((static_cast<std::uint32_t>(frame[1]) << 21U) |
                   (static_cast<std::uint32_t>(frame[2]) << 13U) |
                   (static_cast<std::uint32_t>(frame[3]) << 5U) |
                   (static_cast<std::uint32_t>(frame[4]) >> 3U)),
                  true};
      data_offset = 5U;
    } else {
      frame_id = {static_cast<std::uint32_t>(frame[1] << 3U) |
                      static_cast<std::uint32_t>(frame[2] >> 5U),
                  false};
      data_offset = 3U;
    }
    const std::size_t length = frame[0] & 0x0FU;
    std::array<std::byte, 8U> payload{};
    if (length > payload.size()) {
      return result<hal::can::polled_frame<hal::can::classic_frame>, error_type>::failure(
          error_type::protocol());
    }
    for (std::size_t index = 0U; index < length; ++index) {
      payload[index] = static_cast<std::byte>(frame[data_offset + index]);
    }
    auto made = hal::can::classic_frame::make(
        frame_id, span<const std::byte>{payload.data(), length});
    if (!made) {
      return result<hal::can::polled_frame<hal::can::classic_frame>, error_type>::failure(
          error_type::protocol());
    }
    if (filter_configured_ && !filter_.matches(frame_id)) {
      return result<hal::can::polled_frame<hal::can::classic_frame>, error_type>::success(
          hal::can::polled_frame<hal::can::classic_frame>::unavailable());
    }
    return result<hal::can::polled_frame<hal::can::classic_frame>, error_type>::success(
        hal::can::polled_frame<hal::can::classic_frame>::available(made.value()));
  }

  [[nodiscard]] result<void, error_type> set_filters(
      span<const hal::can::filter> filters) noexcept {
    if (!configured_ || !filters.valid() || filters.size() > 1U) {
      return result<void, error_type>::failure(error_type::configuration());
    }
    peripheral_->mode_reg.rm = 1U;
    if (filters.empty()) {
      peripheral_->acceptance_filter.acr[0].byte = 0U;
      peripheral_->acceptance_filter.acr[1].byte = 0U;
      peripheral_->acceptance_filter.acr[2].byte = 0U;
      peripheral_->acceptance_filter.acr[3].byte = 0U;
      peripheral_->acceptance_filter.amr[0].byte = 0xFFU;
      peripheral_->acceptance_filter.amr[1].byte = 0xFFU;
      peripheral_->acceptance_filter.amr[2].byte = 0xFFU;
      peripheral_->acceptance_filter.amr[3].byte = 0xFFU;
      filter_configured_ = false;
    } else {
      const hal::can::filter selected = filters[0];
      if (!selected.valid()) {
        peripheral_->mode_reg.rm = 0U;
        return result<void, error_type>::failure(error_type::configuration());
      }
      const std::uint32_t shift = selected.frame_id.extended ? 3U : 21U;
      const std::uint32_t code = selected.frame_id.value << shift;
      const std::uint32_t mask = ~(selected.mask << shift);
      const std::uint32_t code_swapped = byte_swap32(code);
      const std::uint32_t mask_swapped = byte_swap32(mask);
      for (std::uint32_t index = 0U; index < 4U; ++index) {
        peripheral_->acceptance_filter.acr[index].byte =
            static_cast<std::uint8_t>(code_swapped >> (index * 8U));
        peripheral_->acceptance_filter.amr[index].byte =
            static_cast<std::uint8_t>(mask_swapped >> (index * 8U));
      }
      filter_ = selected;
      filter_configured_ = true;
    }
    peripheral_->mode_reg.afm = 1U;
    peripheral_->mode_reg.rm = 0U;
    return result<void, error_type>::success();
  }

  [[nodiscard]] bool bus_off() const noexcept {
    return (peripheral_->status_reg.val & (1U << 7U)) != 0U;
  }

  // A bus-off condition automatically places the ESP32-S3 TWAI controller in
  // reset mode. Exiting reset mode starts the hardware-defined recovery wait
  // for 128 bus-free occurrences. Completion remains observable through
  // bus_off(); queued frames are caller-owned, so there is no hidden queue to
  // discard here.
  [[nodiscard]] result<void, error_type> initiate_bus_off_recovery() noexcept {
    if (!configured_ || !bus_off()) {
      return result<void, error_type>::failure(error_type::configuration());
    }
    peripheral_->mode_reg.rm = 0U;
    return result<void, error_type>::success();
  }

 private:
  void set_accept_all_filter() noexcept {
    peripheral_->mode_reg.rm = 1U;
    for (std::uint32_t index = 0U; index < 4U; ++index) {
      peripheral_->acceptance_filter.acr[index].byte = 0U;
      peripheral_->acceptance_filter.amr[index].byte = 0xFFU;
    }
    peripheral_->mode_reg.afm = 1U;
    peripheral_->mode_reg.rm = 0U;
  }

  twai_dev_t* peripheral_;
  config configuration_{};
  hal::can::filter filter_{};
  bool configured_{};
  bool filter_configured_{};
};

}  // namespace hal::esp32s3_wroom_1_n16r8::can
