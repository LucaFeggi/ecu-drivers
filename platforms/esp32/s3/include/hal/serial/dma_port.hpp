#pragma once

#include <cstddef>
#include <cstdint>
#include <hal/dma/channel.hpp>
#include <hal/foundation/result.hpp>
#include <hal/foundation/span.hpp>
#include <hal/serial/port.hpp>
#include <soc/uhci_struct.h>

namespace hal::esp32s3_wroom_1_n16r8::serial {

// UART RX remains a byte-stream FIFO operation. The ESP32-S3 does not connect
// the UART FIFO directly to GDMA; its UHCI block adds packet/escape semantics.
// This class uses UHCI only for raw one-shot TX DMA and intentionally keeps RX
// on the UART FIFO so a caller never gets implicit packet framing.
template <std::size_t DmaChannel = 1U, std::size_t ScratchCapacity = 4092U>
class DmaPort {
  static_assert(ScratchCapacity > 0U && ScratchCapacity <= 4092U);

 public:
  using error_type = error;

  DmaPort(uart_dev_t& uart, uhci_dev_t& uhci, gdma_dev_t& gdma,
          std::uint8_t uart_number,
          span<hal::esp32s3::dma::descriptor> descriptors,
          span<std::byte> tx_scratch,
          std::uint32_t poll_limit = 1'000'000U) noexcept
      : polling_{uart, poll_limit}, uhci_{&uhci}, uart_number_{uart_number},
        dma_{gdma}, descriptors_{descriptors}, tx_scratch_{tx_scratch},
        poll_limit_{poll_limit} {}

  DmaPort(const DmaPort&) = delete;
  DmaPort& operator=(const DmaPort&) = delete;

  [[nodiscard]] result<hertz, error_type> configure(
      hal::serial::config configuration) noexcept {
    if (uart_number_ > 2U || descriptors_.empty() || !descriptors_.valid() ||
        tx_scratch_.empty() || !tx_scratch_.valid() ||
        (reinterpret_cast<std::uintptr_t>(descriptors_.data()) % 4U) != 0U ||
        (reinterpret_cast<std::uintptr_t>(tx_scratch_.data()) % 4U) != 0U) {
      return result<hertz, error_type>::failure(error_type::configuration());
    }
    auto result = polling_.configure(configuration);
    if (!result) {
      return result;
    }

    uhci_->conf0.val = 0U;
    uhci_->conf0.clk_en = 1U;
    uhci_->conf0.uart0_ce = uart_number_ == 0U ? 1U : 0U;
    uhci_->conf0.uart1_ce = uart_number_ == 1U ? 1U : 0U;
    uhci_->conf0.uart2_ce = uart_number_ == 2U ? 1U : 0U;
    uhci_->conf1.val = 0U;
    uhci_->conf0.tx_rst = 1U;
    uhci_->conf0.tx_rst = 0U;
    uhci_->conf0.rx_rst = 1U;
    uhci_->conf0.rx_rst = 0U;
    uhci_->int_clr.val = 0x1FFU;
    dma_.reset_tx();
    configured_ = true;
    return result;
  }

  [[nodiscard]] result<std::size_t, error_type> try_write(
      span<const std::byte> data) noexcept {
    if (!configured_ || !data.valid() || descriptors_.empty() ||
        tx_scratch_.empty()) {
      return result<std::size_t, error_type>::failure(error_type::configuration());
    }
    if (!finish_tx(false)) {
      return result<std::size_t, error_type>::success(0U);
    }
    const std::size_t capacity = tx_scratch_.size() < ScratchCapacity
                                     ? tx_scratch_.size()
                                     : ScratchCapacity;
    const std::size_t count = data.size() < capacity ? data.size() : capacity;
    if (count == 0U) {
      return result<std::size_t, error_type>::success(0U);
    }
    for (std::size_t index = 0U; index < count; ++index) {
      tx_scratch_[index] = data[index];
    }
    dma::make_tx_descriptor(descriptors_[0], tx_scratch_.data(), count);
    dma_.prepare_tx(dma::peripheral::uhci0,
                    dma::address(descriptors_.data()));
    dma_.start_tx();
    active_ = true;
    return result<std::size_t, error_type>::success(count);
  }

  [[nodiscard]] result<std::size_t, error_type> try_read(
      span<std::byte> data) noexcept {
    return polling_.try_read(data);
  }

  [[nodiscard]] result<void, error_type> flush() noexcept {
    if (!configured_) {
      return result<void, error_type>::failure(error_type::configuration());
    }
    if (!finish_tx(true)) {
      return result<void, error_type>::failure(error_type::timeout());
    }
    return polling_.flush();
  }

 private:
  [[nodiscard]] bool finish_tx(bool wait) noexcept {
    if (!active_) {
      return true;
    }
    std::uint32_t remaining = wait ? poll_limit_ : 1U;
    while (remaining != 0U) {
      const std::uint32_t status = dma_.tx_interrupts();
      if ((status & 0x0BU) != 0U) {
        dma_.clear_tx_interrupts();
        dma_.stop_tx();
        active_ = false;
        return true;
      }
      --remaining;
    }
    return false;
  }

  Port<> polling_;
  uhci_dev_t* uhci_;
  std::uint8_t uart_number_;
  dma::Channel<DmaChannel> dma_;
  span<hal::esp32s3::dma::descriptor> descriptors_;
  span<std::byte> tx_scratch_;
  std::uint32_t poll_limit_;
  bool configured_{};
  bool active_{};
};

}  // namespace hal::esp32s3_wroom_1_n16r8::serial
