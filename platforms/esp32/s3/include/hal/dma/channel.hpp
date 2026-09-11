#pragma once

#include <cstddef>
#include <cstdint>
#include <hal/dma/descriptor.hpp>
#include <hal/register_access.hpp>
#include <soc/gdma_struct.h>

namespace hal::esp32s3_wroom_1_n16r8::dma {

enum class peripheral : std::uint8_t {
  spi2 = 0U,
  spi3 = 1U,
  uhci0 = 2U, // UART DMA is routed through the UHCI0 endpoint.
  uart0 = uhci0,
  i2s0 = 3U,
  i2s1 = 4U,
  lcd_cam = 5U,
  aes = 6U,
  sha = 7U,
  adc_dac = 8U,
  rmt = 9U
};

class error {
 public:
  enum class kind : std::uint8_t { invalid_channel, busy, hardware };

  [[nodiscard]] constexpr kind value() const noexcept { return value_; }

 private:
  explicit constexpr error(kind value) noexcept : value_{value} {}
  kind value_;
};

template <std::size_t ChannelIndex>
class Channel {
  static_assert(ChannelIndex < 5U);

 public:
  explicit Channel(gdma_dev_t& device) noexcept : device_{&device} {}

  Channel(const Channel&) = delete;
  Channel& operator=(const Channel&) = delete;

  void reset_rx() noexcept {
    auto& channel = device_->channel[ChannelIndex];
    channel.in.link.stop = 1U;
    channel.in.conf0.in_rst = 1U;
    channel.in.conf0.in_rst = 0U;
    channel.in.int_clr.val = 0x3FFU;
  }

  void reset_tx() noexcept {
    auto& channel = device_->channel[ChannelIndex];
    channel.out.link.stop = 1U;
    channel.out.conf0.out_rst = 1U;
    channel.out.conf0.out_rst = 0U;
    channel.out.int_clr.val = 0xFFU;
  }

  void prepare_rx(peripheral selected, std::uint32_t descriptor_address,
                  std::uint8_t priority = 1U) noexcept {
    auto& channel = device_->channel[ChannelIndex];
    reset_rx();
    constexpr std::uint32_t burst_configuration =
        (1U << 2U) | (1U << 3U) | (1U << 4U) | (1U << 5U);
    channel.in.conf0.val = (channel.in.conf0.val & ~burst_configuration) |
                           (1U << 2U) | (1U << 3U);
    channel.in.conf1.val |= 1U << 12U;
    channel.in.pri.val = (channel.in.pri.val & ~0x0FU) | (priority & 0x0FU);
    channel.in.peri_sel.val =
        (channel.in.peri_sel.val & ~0x3FU) |
        (static_cast<std::uint32_t>(selected) & 0x3FU);
    channel.in.link.val =
        (channel.in.link.val & ~0x000F'FFFFU) |
        (descriptor_address & 0x000F'FFFFU);
    channel.in.int_ena.val = (1U << 1U) | (1U << 3U) | (1U << 4U);
    register_access::device_write_barrier();
  }

  void prepare_tx(peripheral selected, std::uint32_t descriptor_address,
                  std::uint8_t priority = 1U) noexcept {
    auto& channel = device_->channel[ChannelIndex];
    reset_tx();
    // Auto-writeback and EOF mode are required for a one-shot descriptor;
    // burst mode keeps the FIFO fed with the lowest descriptor overhead.
    constexpr std::uint32_t tx_configuration =
        (1U << 2U) | (1U << 3U) | (1U << 4U) | (1U << 5U);
    channel.out.conf0.val = (channel.out.conf0.val & ~tx_configuration) |
                            tx_configuration;
    channel.out.conf1.val |= 1U << 12U;
    channel.out.pri.val = (channel.out.pri.val & ~0x0FU) | (priority & 0x0FU);
    channel.out.peri_sel.val =
        (channel.out.peri_sel.val & ~0x3FU) |
        (static_cast<std::uint32_t>(selected) & 0x3FU);
    channel.out.link.val =
        (channel.out.link.val & ~0x000F'FFFFU) |
        (descriptor_address & 0x000F'FFFFU);
    channel.out.int_ena.val = (1U << 0U) | (1U << 2U) | (1U << 3U);
    register_access::device_write_barrier();
  }

  void start_rx() noexcept {
    register_access::device_write_barrier();
    device_->channel[ChannelIndex].in.link.start = 1U;
  }
  void start_tx() noexcept {
    register_access::device_write_barrier();
    device_->channel[ChannelIndex].out.link.start = 1U;
  }
  void stop_rx() noexcept { device_->channel[ChannelIndex].in.link.stop = 1U; }
  void stop_tx() noexcept { device_->channel[ChannelIndex].out.link.stop = 1U; }

  [[nodiscard]] std::uint32_t rx_interrupts() const noexcept {
    return device_->channel[ChannelIndex].in.int_raw.val;
  }

  [[nodiscard]] std::uint32_t tx_interrupts() const noexcept {
    return device_->channel[ChannelIndex].out.int_raw.val;
  }

  void clear_rx_interrupts(std::uint32_t mask = 0x3FFU) noexcept {
    device_->channel[ChannelIndex].in.int_clr.val = mask;
  }

  void clear_tx_interrupts(std::uint32_t mask = 0xFFU) noexcept {
    device_->channel[ChannelIndex].out.int_clr.val = mask;
  }

  [[nodiscard]] bool rx_idle() const noexcept {
    return device_->channel[ChannelIndex].in.link.park != 0U;
  }

  [[nodiscard]] std::uint32_t rx_success_eof_descriptor() const noexcept {
    return device_->channel[ChannelIndex].in.suc_eof_des_addr;
  }

  [[nodiscard]] bool tx_idle() const noexcept {
    return device_->channel[ChannelIndex].out.link.park != 0U;
  }

 private:
  gdma_dev_t* device_;
};

[[nodiscard]] inline std::uint32_t address(const void* object) noexcept {
  return static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(object));
}

inline void make_tx_descriptor(hal::esp32s3::dma::descriptor& descriptor,
                               const void* buffer, std::size_t size,
                               bool successful_eof = true) noexcept {
  descriptor.control = hal::esp32s3::dma::descriptor_control(
      size, size, true, successful_eof);
  descriptor.buffer_address = address(buffer);
  descriptor.next_descriptor = 0U;
}

inline void make_rx_descriptor(hal::esp32s3::dma::descriptor& descriptor,
                               void* buffer,
                               std::size_t capacity,
                               bool successful_eof = true) noexcept {
  const std::size_t dma_size = (capacity + 3U) & ~std::size_t{3U};
  descriptor.control = hal::esp32s3::dma::descriptor_control(
      dma_size, 0U, true, successful_eof);
  descriptor.buffer_address = address(buffer);
  descriptor.next_descriptor = 0U;
}

}  // namespace hal::esp32s3_wroom_1_n16r8::dma
