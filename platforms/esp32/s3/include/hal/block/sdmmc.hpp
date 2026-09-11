#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/block.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif
#include <hal/foundation/result.hpp>
#include <hal/foundation/span.hpp>
#include <hal/register_access.hpp>
#include <soc/sdmmc_struct.h>

namespace hal::esp32s3_wroom_1_n16r8::block {

class error {
 public:
  explicit constexpr error(hal::block::error_kind kind) noexcept : kind_{kind} {}

  [[nodiscard]] constexpr hal::block::error_kind kind() const noexcept {
    return kind_;
  }

 private:
  hal::block::error_kind kind_{};
};

struct configuration {
  // The SDMMC clock source is the 160 MHz PLL.  20 MHz is the conservative
  // default before card-specific high-speed negotiation is added.
  std::uint32_t clock_hz{20'000'000U};
  std::uint8_t bus_width{4U};
  std::uint32_t poll_limit{1'000'000U};
  bool card_detect{false};
  bool write_protected{false};
};

// ESP32-S3 SDMMC owns a Synopsys-style internal IDMAC.  The descriptors and
// the staging buffer are deliberately supplied by the BSP, so this layer has
// no heap dependency and can operate with a known DMA-capable DRAM region.
template <std::size_t DescriptorCount = 8U>
class Sdmmc {
  static_assert(DescriptorCount > 0U);

 public:
  using error_type = error;

  Sdmmc(sdmmc_dev_t& peripheral, span<sdmmc_desc_t> descriptors,
        span<std::byte> dma_buffer, configuration config = {}) noexcept
      : peripheral_{&peripheral}, descriptors_{descriptors},
        dma_buffer_{dma_buffer}, config_{config} {}

  Sdmmc(const Sdmmc&) = delete;
  Sdmmc& operator=(const Sdmmc&) = delete;

  [[nodiscard]] result<void, error_type> initialize() noexcept {
    if (!descriptors_.valid() || descriptors_.size() < 1U ||
        descriptors_.size() > DescriptorCount || !dma_buffer_.valid() ||
        dma_buffer_.size() < block_size ||
        (reinterpret_cast<std::uintptr_t>(descriptors_.data()) % 4U) != 0U ||
        (reinterpret_cast<std::uintptr_t>(dma_buffer_.data()) % 4U) != 0U ||
        (config_.bus_width != 1U && config_.bus_width != 4U) ||
        config_.clock_hz == 0U || config_.clock_hz > 50'000'000U ||
        config_.poll_limit == 0U) {
      return failure(hal::block::error_kind::not_aligned);
    }

    if (config_.card_detect && (peripheral_->cdetect.cards & 1U) != 0U) {
      return failure(hal::block::error_kind::no_media);
    }

    if (!reset()) {
      return failure(last_error_);
    }
    peripheral_->pwren = 1U;
    peripheral_->ctype.val = 0U;
    peripheral_->blksiz.block_size = block_size;
    peripheral_->tmout.response = 0xFFU;
    peripheral_->tmout.data = 0x00FF'FFFFU;
    peripheral_->fifoth.tx_watermark = 8U;
    peripheral_->fifoth.rx_watermark = 8U;
    peripheral_->clkena.cclk_low_power = 0U;
    peripheral_->ctrl.int_enable = 0U;
    peripheral_->intmask.val = 0U;
    peripheral_->idinten.val = idmac_normal | idmac_abnormal;

    if (!set_clock(400'000U)) {
      return failure(last_error_);
    }
    if (!command(0U, 0U, response_type::none, false, false)) {
      return failure(last_error_);
    }

    // CMD8 is not implemented by legacy SDSC cards.  Its result only selects
    // the later addressing mode; ACMD41 remains the authoritative probe.
    (void)command(8U, 0x1AAU, response_type::short_response, false, true);

    bool ready = false;
    for (std::uint32_t remaining = config_.poll_limit; remaining != 0U;
         --remaining) {
      if (!command(55U, 0U, response_type::short_response, false, true)) {
        if (last_error_ == hal::block::error_kind::timeout) {
          continue;
        }
        return failure(last_error_);
      }
      if (!command(41U, 0x40FF'8000U, response_type::short_response, false,
                   false)) {
        if (last_error_ == hal::block::error_kind::timeout) {
          continue;
        }
        return failure(last_error_);
      }
      if ((response_[0] & card_ready) != 0U) {
        high_capacity_ = (response_[0] & card_high_capacity) != 0U;
        ready = true;
        break;
      }
    }
    if (!ready) {
      return failure(hal::block::error_kind::timeout);
    }

    if (!command(2U, 0U, response_type::long_response, false, true) ||
        !command(3U, 0U, response_type::short_response, false, true)) {
      return failure(last_error_);
    }
    relative_address_ = response_[0] & 0xFFFF'0000U;

    if (!command(9U, relative_address_, response_type::long_response, false,
                 true)) {
      return failure(last_error_);
    }
    decode_csd();
    if (block_count_ == 0U) {
      return failure(hal::block::error_kind::other);
    }

    if (!command(7U, relative_address_, response_type::short_response, false,
                 true)) {
      return failure(last_error_);
    }
    if (config_.bus_width == 4U) {
      if (!command(55U, relative_address_, response_type::short_response,
                   false, true) ||
          !command(6U, 2U, response_type::short_response, false, true)) {
        return failure(last_error_);
      }
      peripheral_->ctype.card_width = 1U;
    }
    if (!high_capacity_ &&
        !command(16U, block_size, response_type::short_response, false,
                 true)) {
      return failure(last_error_);
    }

    if (!set_clock(config_.clock_hz)) {
      return failure(last_error_);
    }
    initialized_ = true;
    return result<void, error_type>::success();
  }

  [[nodiscard]] result<hal::block::geometry, error_type> geometry() noexcept {
    if (!initialized_) {
      return result<hal::block::geometry, error_type>::failure(
          error{hal::block::error_kind::other});
    }
    return result<hal::block::geometry, error_type>::success(
        {block_size, block_count_, config_.write_protected});
  }

  [[nodiscard]] result<void, error_type> read_blocks(
      std::uint64_t lba, span<std::byte> output) noexcept {
    return transfer(lba, output, false);
  }

  [[nodiscard]] result<void, error_type> write_blocks(
      std::uint64_t lba, span<const std::byte> input) noexcept {
    if (config_.write_protected) {
      return failure(hal::block::error_kind::write_protected);
    }
    return transfer(lba, input, true);
  }

  [[nodiscard]] result<void, error_type> sync() noexcept {
    if (!initialized_) {
      return failure(hal::block::error_kind::other);
    }
    if (!wait_not_busy()) {
      return failure(last_error_);
    }
    return command(13U, relative_address_, response_type::short_response,
                   false, true)
               ? result<void, error_type>::success()
               : failure(last_error_);
  }

  [[nodiscard]] result<void, error_type> trim(std::uint64_t first_lba,
                                               std::uint64_t count) noexcept {
    if (!initialized_ || count == 0U || first_lba >= block_count_ ||
        count > block_count_ - first_lba ||
        !address_fits(first_lba + count - 1U)) {
      return failure(hal::block::error_kind::out_of_range);
    }
    if (config_.write_protected) {
      return failure(hal::block::error_kind::write_protected);
    }
    if (!command(32U, address(first_lba), response_type::short_response, false,
                 true) ||
        !command(33U, address(first_lba + count - 1U),
                 response_type::short_response, false, true) ||
        !command(38U, 0U, response_type::short_response, false, true) ||
        !wait_not_busy()) {
      return failure(last_error_);
    }
    return result<void, error_type>::success();
  }

  [[nodiscard]] static constexpr std::uint32_t block_bytes() noexcept {
    return block_size;
  }

 private:
  enum class response_type : std::uint8_t { none, short_response, long_response };

  static constexpr std::uint32_t block_size{512U};
  static constexpr std::uint32_t card_ready{1U << 31U};
  static constexpr std::uint32_t card_high_capacity{1U << 30U};
  static constexpr std::uint32_t idmac_normal{1U << 8U};
  static constexpr std::uint32_t idmac_abnormal{1U << 9U};
  static constexpr std::uint32_t idmac_done{(1U << 0U) | (1U << 1U)};
  static constexpr std::uint32_t idmac_error{(1U << 2U) | (1U << 4U) |
                                             (1U << 5U)};
  static constexpr std::uint32_t command_error{(1U << 1U) | (1U << 6U) |
                                               (1U << 8U)};
  static constexpr std::uint32_t data_error{(1U << 7U) | (1U << 9U) |
                                            (1U << 10U) | (1U << 11U) |
                                            (1U << 13U) | (1U << 15U)};

  [[nodiscard]] result<void, error_type> failure(
      hal::block::error_kind kind) noexcept {
    last_error_ = kind;
    return result<void, error_type>::failure(error{kind});
  }

  [[nodiscard]] bool reset() noexcept {
    peripheral_->ctrl.controller_reset = 1U;
    peripheral_->ctrl.fifo_reset = 1U;
    peripheral_->ctrl.dma_reset = 1U;
    for (std::uint32_t remaining = config_.poll_limit; remaining != 0U;
         --remaining) {
      if (peripheral_->ctrl.controller_reset == 0U &&
          peripheral_->ctrl.fifo_reset == 0U &&
          peripheral_->ctrl.dma_reset == 0U) {
        return true;
      }
    }
    last_error_ = hal::block::error_kind::timeout;
    return false;
  }

  [[nodiscard]] bool update_clock() noexcept {
    for (std::uint32_t remaining = config_.poll_limit; remaining != 0U;
         --remaining) {
      if (peripheral_->cmd.start_command == 0U) {
        break;
      }
      if (remaining == 1U) {
        last_error_ = hal::block::error_kind::timeout;
        return false;
      }
    }

    clear_status();
    peripheral_->cmdarg = 0U;
    write_command(0U, response_type::none, false, false, false, false, true);

    for (std::uint32_t remaining = config_.poll_limit; remaining != 0U;
         --remaining) {
      if (peripheral_->cmd.start_command == 0U) {
        return true;
      }
    }
    last_error_ = hal::block::error_kind::timeout;
    return false;
  }

  [[nodiscard]] bool set_clock(std::uint32_t target_hz) noexcept {
    const std::uint32_t source_hz = 160'000'000U;
    if (target_hz == 0U) {
      last_error_ = hal::block::error_kind::out_of_range;
      return false;
    }

    // The CIU requires a clock-update command whenever its clock registers
    // change.  This is also the mechanism that safely disables/enables the
    // card clock around the update; writing clock fields alone is not enough.
    peripheral_->clkena.cclk_enable = 0U;
    peripheral_->clkena.cclk_low_power = 0U;
    if (!update_clock()) {
      return false;
    }

    // The first divider generates 160 MHz / host_div.  The card divider is
    // bypassed for host_div <= 16 and otherwise divides by 2 * card_div.
    // Select the smallest host divider that can represent the requested
    // frequency, matching the ESP32-S3 SDMMC clock tree's integer-divider
    // policy and retaining the highest useful intermediate clock.
    const std::uint32_t total_div =
        (source_hz + target_hz - 1U) / target_hz;
    std::uint32_t host_div = 2U;
    std::uint32_t card_div = 255U;
    if (total_div <= 16U) {
      host_div = total_div < 2U ? 2U : total_div;
      card_div = 0U;
    } else {
      for (std::uint32_t candidate = 2U; candidate <= 16U; ++candidate) {
        const std::uint32_t intermediate = source_hz / candidate;
        std::uint32_t candidate_card =
            (intermediate + 2U * target_hz - 1U) /
            (2U * target_hz);
        if (candidate_card == 0U) {
          candidate_card = 1U;
        }
        if (candidate_card <= 255U) {
          host_div = candidate;
          card_div = candidate_card;
          break;
        }
      }
    }
    // The SoC header exposes these fields as C bitfields.  Encode the whole
    // register explicitly so the raw target path remains warning-clean and
    // the hardware layout is visible here.
    peripheral_->clock.val =
        (1U << 0U) |                         // phase_dout = 90 degrees
        ((host_div / 2U - 1U) << 9U) |       // div_factor_h
        ((host_div - 1U) << 13U) |           // div_factor_l
        ((host_div - 1U) << 17U) |           // div_factor_n
        (1U << 23U);                         // PLL160M
    peripheral_->clkdiv.val =
        (peripheral_->clkdiv.val & ~0xFFU) | (card_div & 0xFFU);
    peripheral_->clksrc.val &= ~0x3U;
    if (!update_clock()) {
      return false;
    }
    peripheral_->clkena.cclk_enable = 1U;
    peripheral_->clkena.cclk_low_power = 1U;
    return update_clock();
  }

  [[nodiscard]] bool command(std::uint8_t index, std::uint32_t argument,
                              response_type response, bool data,
                              bool crc) noexcept {
    for (std::uint32_t remaining = config_.poll_limit; remaining != 0U;
         --remaining) {
      if (peripheral_->cmd.start_command == 0U) {
        break;
      }
      if (remaining == 1U) {
        last_error_ = hal::block::error_kind::timeout;
        return false;
      }
    }

    peripheral_->rintsts.val = 0xFFFF'FFFFU;
    peripheral_->idsts.val = 0xFFFF'FFFFU;
    peripheral_->cmdarg = argument;
    peripheral_->intmask.val = command_error | (1U << 2U) |
                               (data ? ((1U << 3U) | data_error) : 0U);
    write_command(index, response, crc, data, false, false);
    peripheral_->ctrl.int_enable = 1U;

    for (std::uint32_t remaining = config_.poll_limit; remaining != 0U;
         --remaining) {
      const std::uint32_t status = peripheral_->mintsts.val;
      if ((status & command_error) != 0U) {
        last_error_ = (status & (1U << 8U)) != 0U
                          ? hal::block::error_kind::timeout
                          : hal::block::error_kind::io;
        clear_status();
        return false;
      }
      if ((status & (1U << 2U)) != 0U) {
        if (response != response_type::none) {
          response_[0] = peripheral_->resp[0];
          response_[1] = peripheral_->resp[1];
          response_[2] = peripheral_->resp[2];
          response_[3] = peripheral_->resp[3];
        }
        clear_status();
        return true;
      }
    }
    last_error_ = hal::block::error_kind::timeout;
    clear_status();
    return false;
  }

  [[nodiscard]] bool data_transfer(std::uint64_t lba, std::size_t bytes,
                                   bool write) noexcept {
    const std::size_t blocks = bytes / block_size;
    const std::size_t max_blocks = descriptors_.size();
    if (blocks == 0U || blocks > max_blocks || blocks > 0xFFFFU ||
        bytes > 0xFFFFU * block_size) {
      last_error_ = hal::block::error_kind::out_of_range;
      return false;
    }

    for (std::size_t index = 0U; index < blocks; ++index) {
      sdmmc_desc_t& descriptor = descriptors_[index];
      descriptor = {};
      descriptor.first_descriptor = index == 0U;
      descriptor.last_descriptor = index + 1U == blocks;
      descriptor.second_address_chained = 1U;
      descriptor.owned_by_idmac = 1U;
      descriptor.buffer1_size = block_size;
      descriptor.buffer1_ptr = dma_buffer_.data() + index * block_size;
      descriptor.next_desc_ptr =
          descriptor.last_descriptor ? nullptr : &descriptors_[index + 1U];
    }

    register_access::device_write_barrier();

    peripheral_->rintsts.val = 0xFFFF'FFFFU;
    peripheral_->idsts.val = 0xFFFF'FFFFU;
    if (!reset_transfer_engines()) {
      return false;
    }
    peripheral_->ctrl.use_internal_dma = 1U;
    peripheral_->ctrl.dma_enable = 1U;
    peripheral_->bmod.val = 0U;
    peripheral_->bmod.fb = 1U;
    peripheral_->bmod.enable = 1U;
    peripheral_->bmod.pbl = 1U;
    peripheral_->dbaddr = descriptors_.data();
    peripheral_->blksiz.block_size = block_size;
    peripheral_->bytcnt = static_cast<std::uint32_t>(bytes);

    peripheral_->intmask.val = (1U << 3U) | data_error;
    peripheral_->idinten.val = idmac_normal | idmac_abnormal;
    peripheral_->pldmnd = 1U;

    peripheral_->cmdarg = address(lba);
    write_command(
        static_cast<std::uint8_t>(write ? (blocks == 1U ? 24U : 25U)
                                         : (blocks == 1U ? 17U : 18U)),
        response_type::short_response, true, true, write, blocks > 1U);

    bool complete = false;
    for (std::uint32_t remaining = config_.poll_limit; remaining != 0U;
         --remaining) {
      const std::uint32_t status = peripheral_->mintsts.val;
      const std::uint32_t dma_status = peripheral_->idsts.val;
      if ((status & data_error) != 0U || (dma_status & idmac_error) != 0U) {
        last_error_ = (status & ((1U << 10U) | (1U << 9U))) != 0U
                          ? hal::block::error_kind::timeout
                          : hal::block::error_kind::io;
        clear_status();
        stop_dma();
        return false;
      }
      if ((status & (1U << 3U)) != 0U &&
          (dma_status & idmac_done) != 0U) {
        complete = true;
        break;
      }
    }
    clear_status();
    stop_dma();
    if (!complete) {
      last_error_ = hal::block::error_kind::timeout;
      return false;
    }
    return true;
  }

  [[nodiscard]] bool reset_transfer_engines() noexcept {
    peripheral_->ctrl.dma_reset = 1U;
    peripheral_->ctrl.fifo_reset = 1U;
    for (std::uint32_t remaining = config_.poll_limit; remaining != 0U;
         --remaining) {
      if (peripheral_->ctrl.dma_reset == 0U &&
          peripheral_->ctrl.fifo_reset == 0U) {
        return true;
      }
    }
    last_error_ = hal::block::error_kind::timeout;
    return false;
  }

  template <class Buffer>
  [[nodiscard]] result<void, error_type> transfer(std::uint64_t lba,
                                                   Buffer buffer,
                                                   bool write) noexcept {
    if (!initialized_) {
      return failure(hal::block::error_kind::other);
    }
    if (!buffer.valid() || buffer.size() == 0U ||
        buffer.size() % block_size != 0U) {
      return failure(hal::block::error_kind::not_aligned);
    }
    const std::size_t total_blocks = buffer.size() / block_size;
    const std::size_t buffer_blocks = dma_buffer_.size() / block_size;
    const std::size_t max_transfer_blocks =
        descriptors_.size() < buffer_blocks ? descriptors_.size()
                                             : buffer_blocks;
    if (max_transfer_blocks == 0U) {
      return failure(hal::block::error_kind::not_aligned);
    }
    if (lba >= block_count_ || total_blocks > block_count_ - lba) {
      return failure(hal::block::error_kind::out_of_range);
    }

    std::size_t block_offset = 0U;
    while (block_offset < total_blocks) {
      const std::size_t blocks =
          (total_blocks - block_offset) < max_transfer_blocks
              ? total_blocks - block_offset
              : max_transfer_blocks;
      const std::size_t bytes = blocks * block_size;
      if (write) {
        for (std::size_t index = 0U; index < bytes; ++index) {
          dma_buffer_[index] = buffer[block_offset * block_size + index];
        }
      }
      if (!data_transfer(lba + block_offset, bytes, write)) {
        return failure(last_error_);
      }
      if (!write) {
        if constexpr (std::is_const_v<typename Buffer::element_type>) {
          return failure(hal::block::error_kind::other);
        } else {
          for (std::size_t index = 0U; index < bytes; ++index) {
            buffer[block_offset * block_size + index] = dma_buffer_[index];
          }
        }
      }
      block_offset += blocks;
    }
    return result<void, error_type>::success();
  }

  void clear_status() noexcept {
    peripheral_->rintsts.val = 0xFFFF'FFFFU;
    peripheral_->idsts.val = 0xFFFF'FFFFU;
  }

  void clear_command() noexcept {
    *reinterpret_cast<volatile std::uint32_t*>(&peripheral_->cmd) = 0U;
  }

  void write_command(std::uint8_t index, response_type response, bool crc,
                     bool data, bool write, bool auto_stop,
                     bool update_clock_reg = false) noexcept {
    constexpr std::uint32_t response_expect = 1U << 6U;
    constexpr std::uint32_t response_long = 1U << 7U;
    constexpr std::uint32_t check_response_crc = 1U << 8U;
    constexpr std::uint32_t data_expected = 1U << 9U;
    constexpr std::uint32_t rw = 1U << 10U;
    constexpr std::uint32_t send_auto_stop = 1U << 12U;
    constexpr std::uint32_t wait_complete = 1U << 13U;
    constexpr std::uint32_t update_clock = 1U << 21U;
    constexpr std::uint32_t use_hold_reg = 1U << 29U;
    constexpr std::uint32_t start_command = 1U << 31U;

    std::uint32_t value = static_cast<std::uint32_t>(index) & 0x3FU;
    if (response != response_type::none) {
      value |= response_expect;
    }
    if (response == response_type::long_response) {
      value |= response_long;
    }
    if (crc) {
      value |= check_response_crc;
    }
    if (data) {
      value |= data_expected;
    }
    if (write) {
      value |= rw;
    }
    if (auto_stop) {
      value |= send_auto_stop;
    }
    if (update_clock_reg) {
      value |= update_clock;
    }
    value |= wait_complete | use_hold_reg | start_command;
    *reinterpret_cast<volatile std::uint32_t*>(&peripheral_->cmd) = value;
  }

  void stop_dma() noexcept {
    peripheral_->ctrl.use_internal_dma = 0U;
    peripheral_->ctrl.dma_enable = 0U;
    peripheral_->bmod.enable = 0U;
    peripheral_->bmod.fb = 0U;
  }

  [[nodiscard]] bool wait_not_busy() noexcept {
    for (std::uint32_t remaining = config_.poll_limit; remaining != 0U;
         --remaining) {
      if (peripheral_->status.data_busy == 0U) {
        return true;
      }
    }
    last_error_ = hal::block::error_kind::timeout;
    return false;
  }

  [[nodiscard]] bool address_fits(std::uint64_t lba) const noexcept {
    return high_capacity_ ? lba <= 0xFFFF'FFFFULL
                          : lba <= 0x007F'FFFFULL;
  }

  [[nodiscard]] std::uint32_t address(std::uint64_t lba) const noexcept {
    return high_capacity_ ? static_cast<std::uint32_t>(lba)
                          : static_cast<std::uint32_t>(lba * block_size);
  }

  [[nodiscard]] static std::uint32_t bits(const std::uint32_t (&words)[4U],
                                           unsigned start,
                                           unsigned length) noexcept {
    const unsigned word = start / 32U;
    const unsigned shift = start % 32U;
    const std::uint32_t right = words[word] >> shift;
    const std::uint32_t left =
        (length + shift <= 32U) ? 0U : words[word + 1U] << (32U - shift);
    const std::uint32_t mask =
        length == 32U ? 0xFFFF'FFFFU : ((1U << length) - 1U);
    return (right | left) & mask;
  }

  void decode_csd() noexcept {
    const std::uint32_t version = bits(response_, 126U, 2U);
    if (version == 1U) {
      const std::uint32_t c_size = bits(response_, 48U, 22U);
      block_count_ = (static_cast<std::uint64_t>(c_size) + 1U) * 1024U;
      return;
    }
    if (version == 0U) {
      const std::uint32_t c_size = bits(response_, 62U, 12U);
      const std::uint32_t multiplier = bits(response_, 47U, 3U);
      const std::uint32_t read_length = bits(response_, 80U, 4U);
      const std::uint64_t bytes = (static_cast<std::uint64_t>(c_size) + 1U) *
                                  (std::uint64_t{1U} << (multiplier + 2U)) *
                                  (std::uint64_t{1U} << read_length);
      block_count_ = bytes / block_size;
      return;
    }
    block_count_ = 0U;
  }

  sdmmc_dev_t* peripheral_;
  span<sdmmc_desc_t> descriptors_;
  span<std::byte> dma_buffer_;
  configuration config_;
  std::uint32_t response_[4U]{};
  std::uint64_t block_count_{};
  std::uint32_t relative_address_{};
  hal::block::error_kind last_error_{hal::block::error_kind::other};
  bool high_capacity_{};
  bool initialized_{};
};

}  // namespace hal::esp32s3_wroom_1_n16r8::block
