#pragma once

#include <cstddef>
#include <cstdint>
#include <hal/foundation/result.hpp>
#include <hal/foundation/span.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/serial.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif
#include <soc/uart_struct.h>

namespace hal::esp32s3_wroom_1_n16r8::serial {

class error {
 public:
  [[nodiscard]] static constexpr error configuration() noexcept {
    return error{hal::serial::error_kind::configuration};
  }
  [[nodiscard]] static constexpr error timeout() noexcept {
    return error{hal::serial::error_kind::timeout};
  }
  [[nodiscard]] static constexpr error overrun() noexcept {
    return error{hal::serial::error_kind::overrun};
  }
  [[nodiscard]] constexpr hal::serial::error_kind kind() const noexcept {
    return kind_;
  }

 private:
  constexpr explicit error(hal::serial::error_kind kind) noexcept : kind_{kind} {}
  hal::serial::error_kind kind_;
};

template <std::uint32_t SourceHz = 80'000'000U>
class Port {
 public:
  using error_type = error;

  Port(uart_dev_t& uart, std::uint32_t poll_limit = 1'000'000U) noexcept
      : uart_{&uart}, poll_limit_{poll_limit} {}

  [[nodiscard]] result<hertz, error_type> configure(
      hal::serial::config configuration) noexcept {
    if (configuration.baud_rate.value == 0U ||
        configuration.bits == hal::serial::data_bits::bits9 ||
        poll_limit_ == 0U) {
      return result<hertz, error_type>::failure(error_type::configuration());
    }

    const std::uint64_t scaled =
        (static_cast<std::uint64_t>(SourceHz) << 4U) /
        configuration.baud_rate.value;
    if (scaled == 0U || scaled > 0xFFFFFU) {
      return result<hertz, error_type>::failure(error_type::configuration());
    }
    uart_->clkdiv.val =
        (uart_->clkdiv.val & ~((0x0FFFU << 0U) | (0x0FU << 20U))) |
        ((static_cast<std::uint32_t>(scaled >> 4U) & 0x0FFFU) << 0U) |
        ((static_cast<std::uint32_t>(scaled & 0xFU)) << 20U);

    uart_->conf0.parity_en =
        configuration.parity_mode == hal::serial::parity::none ? 0U : 1U;
    uart_->conf0.parity =
        configuration.parity_mode == hal::serial::parity::odd ? 1U : 0U;
    uart_->conf0.bit_num =
        configuration.bits == hal::serial::data_bits::bits7 ? 2U : 3U;
    uart_->conf0.stop_bit_num =
        configuration.stops == hal::serial::stop_bits::two ? 3U : 1U;
    uart_->conf0.tx_flow_en =
        configuration.flow == hal::serial::flow_control::rts ||
                configuration.flow == hal::serial::flow_control::rts_cts
            ? 1U
            : 0U;
    uart_->conf1.rx_flow_en =
        configuration.flow == hal::serial::flow_control::cts ||
                configuration.flow == hal::serial::flow_control::rts_cts
            ? 1U
            : 0U;
    uart_->conf0.rxfifo_rst = 1U;
    uart_->conf0.rxfifo_rst = 0U;
    uart_->conf0.txfifo_rst = 1U;
    uart_->conf0.txfifo_rst = 0U;
    configured_ = true;
    return result<hertz, error_type>::success(actual_baud(scaled));
  }

  [[nodiscard]] result<std::size_t, error_type> try_write(
      span<const std::byte> data) noexcept {
    if (!configured_ || !data.valid()) {
      return result<std::size_t, error_type>::failure(error_type::configuration());
    }
    const std::size_t available = 128U - uart_->status.txfifo_cnt;
    const std::size_t count = data.size() < available ? data.size() : available;
    for (std::size_t index = 0U; index < count; ++index) {
      uart_->fifo.rxfifo_rd_byte =
          static_cast<std::uint32_t>(std::to_integer<unsigned char>(data[index]));
    }
    return result<std::size_t, error_type>::success(count);
  }

  [[nodiscard]] result<std::size_t, error_type> try_read(
      span<std::byte> data) noexcept {
    if (!configured_ || !data.valid()) {
      return result<std::size_t, error_type>::failure(error_type::configuration());
    }
    const std::size_t available = uart_->status.rxfifo_cnt;
    const std::size_t count = data.size() < available ? data.size() : available;
    const std::uint32_t interrupt_status = uart_->int_raw.val;
    if ((interrupt_status & (1U << 4U)) != 0U) {
      uart_->int_clr.rxfifo_ovf_int_clr = 1U;
      return result<std::size_t, error_type>::failure(error_type::overrun());
    }
    for (std::size_t index = 0U; index < count; ++index) {
      data[index] = static_cast<std::byte>(uart_->fifo.rxfifo_rd_byte & 0xFFU);
    }
    return result<std::size_t, error_type>::success(count);
  }

  [[nodiscard]] result<void, error_type> flush() noexcept {
    std::uint32_t remaining = poll_limit_;
    while (uart_->status.txfifo_cnt != 0U && remaining != 0U) {
      --remaining;
    }
    return remaining == 0U ? result<void, error_type>::failure(error_type::timeout())
                           : result<void, error_type>::success();
  }

  [[nodiscard]] static constexpr hertz actual_baud(
      std::uint64_t divider_4x) noexcept {
    return hertz{(static_cast<std::uint64_t>(SourceHz) << 4U) /
                 divider_4x};
  }

 private:
  uart_dev_t* uart_;
  std::uint32_t poll_limit_;
  bool configured_{};
};

}  // namespace hal::esp32s3_wroom_1_n16r8::serial
