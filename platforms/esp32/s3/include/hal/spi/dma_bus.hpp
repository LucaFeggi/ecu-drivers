#pragma once

#include <cstddef>
#include <cstdint>
#include <hal/dma/descriptor.hpp>
#include <hal/dma/channel.hpp>
#include <hal/foundation/result.hpp>
#include <hal/foundation/span.hpp>
#include <hal/contracts/spi_bus.hpp>
#include <soc/spi_struct.h>

namespace hal::esp32s3_wroom_1_n16r8::spi {

class error {
 public:
  [[nodiscard]] static constexpr error configuration() noexcept {
    return error{hal::spi::error_kind::configuration};
  }
  [[nodiscard]] static constexpr error busy() noexcept {
    return error{hal::spi::error_kind::busy};
  }
  [[nodiscard]] static constexpr error timeout() noexcept {
    return error{hal::spi::error_kind::timeout};
  }
  [[nodiscard]] static constexpr error io() noexcept {
    return error{hal::spi::error_kind::io};
  }
  [[nodiscard]] constexpr hal::spi::error_kind kind() const noexcept {
    return kind_;
  }

 private:
  constexpr explicit error(hal::spi::error_kind kind) noexcept : kind_{kind} {}
  hal::spi::error_kind kind_;
};

template <dma::peripheral Peripheral = dma::peripheral::spi2,
          std::size_t DmaChannel = 0U, std::size_t ScratchCapacity = 4092U>
class DmaBus8 {
  static_assert(ScratchCapacity > 0U && ScratchCapacity <= 4092U);

 public:
  using error_type = error;
  static constexpr bool supports_lsb_first{true};

  DmaBus8(spi_dev_t& spi, gdma_dev_t& gdma,
          span<hal::esp32s3::dma::descriptor> tx_descriptors,
          span<hal::esp32s3::dma::descriptor> rx_descriptors,
          span<std::byte> tx_scratch, span<std::byte> rx_scratch,
          std::uint32_t poll_limit = 1'000'000U) noexcept
      : spi_{&spi}, dma_{gdma}, tx_descriptors_{tx_descriptors},
        rx_descriptors_{rx_descriptors}, tx_scratch_{tx_scratch},
        rx_scratch_{rx_scratch}, poll_limit_{poll_limit} {}

  DmaBus8(const DmaBus8&) = delete;
  DmaBus8& operator=(const DmaBus8&) = delete;

  [[nodiscard]] result<hertz, error_type> configure(
      hal::spi::config8 configuration) noexcept {
    if (configuration.max_frequency.value == 0U || poll_limit_ == 0U ||
        tx_descriptors_.empty() || rx_descriptors_.empty() ||
        !tx_descriptors_.valid() || !rx_descriptors_.valid() ||
        !tx_scratch_.valid() || !rx_scratch_.valid() ||
        tx_scratch_.empty() || rx_scratch_.empty() ||
        tx_scratch_.size() < 4U || rx_scratch_.size() < 4U ||
        (reinterpret_cast<std::uintptr_t>(tx_descriptors_.data()) % 4U) != 0U ||
        (reinterpret_cast<std::uintptr_t>(rx_descriptors_.data()) % 4U) != 0U ||
        (reinterpret_cast<std::uintptr_t>(tx_scratch_.data()) % 4U) != 0U ||
        (reinterpret_cast<std::uintptr_t>(rx_scratch_.data()) % 4U) != 0U) {
      return result<hertz, error_type>::failure(error_type::configuration());
    }

    std::uint32_t selected_pre = 0U;
    std::uint32_t selected_n = 0U;
    std::uint64_t selected_frequency = 0U;
    const bool use_system_clock = configuration.max_frequency.value >=
                                  80'000'000U;
    if (use_system_clock) {
      selected_frequency = 80'000'000U;
    } else {
      for (std::uint32_t pre = 0U; pre < 16U; ++pre) {
        // N=1 cannot produce a valid high/low clock phase. The ESP32-S3
        // register implementation uses N in the range 2..64 for divided mode.
        for (std::uint32_t n = 2U; n <= 64U; ++n) {
          const std::uint64_t frequency =
              80'000'000ULL / (static_cast<std::uint64_t>(pre) + 1ULL) /
              static_cast<std::uint64_t>(n);
          if (frequency <= configuration.max_frequency.value &&
              frequency > selected_frequency) {
            selected_frequency = frequency;
            selected_pre = pre;
            selected_n = n;
          }
        }
      }
    }
    if (selected_frequency == 0U || (!use_system_clock && selected_n == 0U)) {
      return result<hertz, error_type>::failure(error_type::configuration());
    }

    dma_.reset_rx();
    dma_.reset_tx();
    spi_->cmd.usr = 0U;
    spi_->dma_conf.dma_rx_ena = 0U;
    spi_->dma_conf.dma_tx_ena = 0U;
    spi_->clk_gate.clk_en = 1U;
    spi_->clk_gate.mst_clk_active = 1U;
    spi_->clk_gate.mst_clk_sel = 1U;  // PLL_CLK_80M.
    if (use_system_clock) {
      spi_->clock.val = 1U << 31U;
    } else {
      spi_->clock.val = ((selected_n - 1U) << 0U) |
                        ((selected_n - 1U) << 6U) |
                        ((selected_n / 2U - 1U) << 12U) |
                        (selected_pre << 18U);
    }

    spi_->slave.slave_mode = 0U;
    spi_->misc.cs0_dis = 1U;  // CS is owned by hal::spi::StaticDevice8.
    spi_->misc.cs_keep_active = 0U;
    spi_->user.cs_setup = 0U;
    spi_->user.cs_hold = 0U;
    spi_->user.qpi_mode = 0U;
    spi_->user.opi_mode = 0U;
    spi_->user.doutdin = 1U;
    spi_->user.usr_command = 0U;
    spi_->user.usr_addr = 0U;
    spi_->user.usr_dummy = 0U;
    spi_->ctrl.rd_bit_order =
        configuration.order == hal::spi::bit_order::lsb_first ? 1U : 0U;
    spi_->ctrl.wr_bit_order =
        configuration.order == hal::spi::bit_order::lsb_first ? 1U : 0U;
    spi_->misc.ck_idle_edge =
        configuration.clock_mode == hal::spi::mode::mode2 ||
                configuration.clock_mode == hal::spi::mode::mode3
            ? 1U
            : 0U;
    spi_->user.ck_out_edge =
        configuration.clock_mode == hal::spi::mode::mode1 ||
                configuration.clock_mode == hal::spi::mode::mode2
            ? 1U
            : 0U;
    read_fill_ = configuration.read_fill;
    configured_ = true;
    return result<hertz, error_type>::success(hertz{selected_frequency});
  }

  [[nodiscard]] result<void, error_type> write(span<const std::byte> data) noexcept {
    return exchange(data, {}, std::byte{0U});
  }

  [[nodiscard]] result<void, error_type> read(span<std::byte> data,
                                               std::byte fill) noexcept {
    return exchange({}, data, fill);
  }

  [[nodiscard]] result<void, error_type> transfer(
      span<const std::byte> tx, span<std::byte> rx) noexcept {
    return exchange(tx, rx, read_fill_);
  }

  [[nodiscard]] result<void, error_type> flush() noexcept {
    if (!configured_) {
      return result<void, error_type>::failure(error_type::configuration());
    }
    return result<void, error_type>::success();
  }

 private:
  [[nodiscard]] result<void, error_type> exchange(span<const std::byte> tx,
                                                   span<std::byte> rx,
                                                   std::byte fill) noexcept {
    if (!configured_ || !tx.valid() || !rx.valid() ||
        !tx_descriptors_.valid() || !rx_descriptors_.valid() ||
        tx_descriptors_.empty() || rx_descriptors_.empty()) {
      return result<void, error_type>::failure(error_type::configuration());
    }
    const std::size_t total = tx.size() > rx.size() ? tx.size() : rx.size();
    if (total == 0U) {
      return result<void, error_type>::success();
    }

    const std::size_t available_capacity =
        tx_scratch_.size() < rx_scratch_.size() ? tx_scratch_.size()
                                                 : rx_scratch_.size();
    const std::size_t staging_capacity = available_capacity < ScratchCapacity
                                             ? available_capacity
                                             : ScratchCapacity;
    if (staging_capacity < 4U) {
      return result<void, error_type>::failure(error_type::configuration());
    }
    // RX DMA writes complete words. Reserve the possible three-byte tail so
    // the rounded DMA length remains within the caller-owned staging buffer.
    const std::size_t chunk_capacity = staging_capacity - 3U;
    std::size_t offset = 0U;
    while (offset < total) {
      const std::size_t remaining = total - offset;
      const std::size_t count = remaining < chunk_capacity ? remaining
                                                             : chunk_capacity;
      for (std::size_t index = 0U; index < count; ++index) {
        tx_scratch_[index] = index + offset < tx.size() ? tx[index + offset]
                                                         : fill;
        rx_scratch_[index] = std::byte{0U};
      }
      const std::size_t rx_dma_size = (count + 3U) & ~std::size_t{3U};
      if (rx_dma_size > rx_scratch_.size()) {
        return result<void, error_type>::failure(error_type::configuration());
      }
      dma::make_tx_descriptor(tx_descriptors_[0], tx_scratch_.data(), count);
      dma::make_rx_descriptor(rx_descriptors_[0], rx_scratch_.data(), count);
      dma_.prepare_rx(Peripheral, dma::address(rx_descriptors_.data()));
      dma_.prepare_tx(Peripheral, dma::address(tx_descriptors_.data()));

      spi_->dma_int_clr.val = 0x003F'FFFFU;
      spi_->dma_conf.rx_afifo_rst = 1U;
      spi_->dma_conf.rx_afifo_rst = 0U;
      spi_->dma_conf.dma_afifo_rst = 1U;
      spi_->dma_conf.dma_afifo_rst = 0U;
      spi_->ms_dlen.val =
          (spi_->ms_dlen.val & ~0x0003'FFFFU) |
          (static_cast<std::uint32_t>(count * 8U - 1U) & 0x0003'FFFFU);
      spi_->user.usr_mosi = 1U;
      spi_->user.usr_miso = 1U;
      spi_->dma_conf.dma_rx_ena = 1U;
      spi_->dma_conf.dma_tx_ena = 1U;
      dma_.start_rx();
      dma_.start_tx();
      spi_->cmd.usr = 1U;

      std::uint32_t remaining_polls = poll_limit_;
      while (remaining_polls != 0U) {
        const bool controller_done =
            spi_->dma_int_raw.trans_done_int_raw != 0U ||
            spi_->cmd.usr == 0U;
        const std::uint32_t rx_status = dma_.rx_interrupts();
        const std::uint32_t tx_status = dma_.tx_interrupts();
        const bool dma_done = (rx_status & (1U << 1U)) != 0U &&
                              (tx_status & (1U << 0U)) != 0U;
        if (controller_done && dma_done) {
          break;
        }
        --remaining_polls;
      }
      const std::uint32_t rx_status = dma_.rx_interrupts();
      const std::uint32_t tx_status = dma_.tx_interrupts();
      const bool dma_error = (rx_status & ((1U << 3U) | (1U << 4U))) != 0U ||
                             (tx_status & (1U << 2U)) != 0U;
      const bool controller_done = spi_->dma_int_raw.trans_done_int_raw != 0U ||
                                   spi_->cmd.usr == 0U;
      const bool dma_done = (rx_status & (1U << 1U)) != 0U &&
                            (tx_status & (1U << 0U)) != 0U;
      const bool done = controller_done && dma_done;
      spi_->cmd.usr = 0U;
      spi_->dma_conf.dma_rx_ena = 0U;
      spi_->dma_conf.dma_tx_ena = 0U;
      dma_.stop_rx();
      dma_.stop_tx();
      spi_->dma_int_clr.val = 0x003F'FFFFU;
      if (dma_error) {
        return result<void, error_type>::failure(error_type::io());
      }
      if (!done || remaining_polls == 0U) {
        return result<void, error_type>::failure(error_type::timeout());
      }

      for (std::size_t index = 0U; index < count && index + offset < rx.size();
           ++index) {
        rx[index + offset] = rx_scratch_[index];
      }
      offset += count;
    }
    return result<void, error_type>::success();
  }

  spi_dev_t* spi_;
  dma::Channel<DmaChannel> dma_;
  span<hal::esp32s3::dma::descriptor> tx_descriptors_;
  span<hal::esp32s3::dma::descriptor> rx_descriptors_;
  span<std::byte> tx_scratch_;
  span<std::byte> rx_scratch_;
  std::uint32_t poll_limit_;
  std::byte read_fill_{std::byte{0xFFU}};
  bool configured_{};
};

}  // namespace hal::esp32s3_wroom_1_n16r8::spi
