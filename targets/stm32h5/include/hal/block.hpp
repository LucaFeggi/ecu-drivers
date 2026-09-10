#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <hal/block.hpp>
#include <hal/stm32h5/support.hpp>
#include <type_traits>

namespace hal::stm32h5::block {

inline constexpr capability support{
    implementation_status::available,
    "direct SDMMC block protocol with STM32H5 internal IDMA"};

class error {
public:
  explicit constexpr error(hal::block::error_kind kind) noexcept
      : kind_{kind} {}

  [[nodiscard]] constexpr hal::block::error_kind kind() const noexcept {
    return kind_;
  }

private:
  hal::block::error_kind kind_{};
};

struct config {
  std::uint32_t clock_control{};
  std::uint8_t dma_request{};
  std::uint8_t bus_width{1U};
  poll_budget timeout{};
  bool use_dma{};
  bool media_present{true};
  bool write_protected{};
};

struct no_dma {};

template <class Registers, unsigned Instance = 1U, class DmaStream = no_dma,
          class DmamuxChannel = no_dma>
class SdmmcDevice {
  using stream_type =
      std::conditional_t<std::is_same_v<DmaStream, no_dma>, no_dma, DmaStream>;
  using mux_type = std::conditional_t<std::is_same_v<DmamuxChannel, no_dma>,
                                      no_dma, DmamuxChannel>;

public:
  using error_type = error;

  explicit SdmmcDevice(Registers &registers, config configuration) noexcept
      : registers_{registers}, configuration_{configuration} {}

  SdmmcDevice(Registers &registers, DmaStream &stream, DmamuxChannel &dmamux,
              config configuration) noexcept
    requires(!std::is_same_v<DmaStream, no_dma> &&
             !std::is_same_v<DmamuxChannel, no_dma>)
      : registers_{registers}, configuration_{configuration}, stream_{&stream},
        dmamux_{&dmamux} {}

  SdmmcDevice(const SdmmcDevice &) = delete;
  SdmmcDevice &operator=(const SdmmcDevice &) = delete;

  [[nodiscard]] auto initialize() -> result<void, error_type> {
    if (!configuration_.timeout.valid() || !configuration_.media_present ||
        (configuration_.bus_width != 1U && configuration_.bus_width != 4U)) {
      return failure(configuration_.media_present
                         ? hal::block::error_kind::other
                         : hal::block::error_kind::no_media);
    }
    registers_.POWER = power_on;
    registers_.CLKCR =
        (configuration_.clock_control & ~clock_width_mask) |
        (configuration_.bus_width == 4U ? clock_width_4_bit : 0U);
    if (!command(0U, 0U, response::none)) {
      return failure(hal::block::error_kind::timeout);
    }
    const auto interface_condition =
        command(8U, 0x1AAU, response::short_response);
    // A legacy SDSC card legitimately times out on CMD8; the ACMD41 path
    // below determines whether it is still usable.
    (void)interface_condition;
    bool ready = false;
    for (std::uint32_t attempts = configuration_.timeout.iterations;
         attempts > 0U; --attempts) {
      if (!command(55U, 0U, response::short_response) &&
          last_error_ != hal::block::error_kind::no_media) {
        return failure(last_error_);
      }
      if (!command(41U, 0x40FF8000U, response::short_response)) {
        continue;
      }
      if ((registers_.RESP1 & card_power_up) != 0U) {
        high_capacity_ = (registers_.RESP1 & card_capacity) != 0U;
        ready = true;
        break;
      }
    }
    if (!ready) {
      return failure(hal::block::error_kind::timeout);
    }
    if (!command(2U, 0U, response::long_response) ||
        !command(3U, 0U, response::short_response)) {
      return failure(last_error_);
    }
    relative_address_ = registers_.RESP1 & 0xFFFF0000U;
    if (!command(9U, relative_address_, response::long_response)) {
      return failure(last_error_);
    }
    decode_csd();
    if (!command(7U, relative_address_, response::short_response)) {
      return failure(last_error_);
    }
    if (configuration_.bus_width == 4U) {
      if (!command(55U, relative_address_, response::short_response) ||
          !command(6U, 2U, response::short_response)) {
        return failure(last_error_);
      }
    }
    if (!high_capacity_ && !command(16U, 512U, response::short_response)) {
      return failure(last_error_);
    }
    initialized_ = true;
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto geometry() -> result<hal::block::geometry, error_type> {
    if (!initialized_) {
      return result<hal::block::geometry, error_type>::failure(
          error{hal::block::error_kind::other});
    }
    return result<hal::block::geometry, error_type>::success(
        {512U, block_count_, configuration_.write_protected});
  }

  [[nodiscard]] auto read_blocks(std::uint64_t lba, hal::span<std::byte> output)
      -> result<void, error_type> {
    return data_transfer(lba, output, false);
  }

  [[nodiscard]] auto write_blocks(std::uint64_t lba,
                                  hal::span<const std::byte> input)
      -> result<void, error_type> {
    HAL_CORE_ASSERT(input.valid());
    if (configuration_.write_protected) {
      return failure(hal::block::error_kind::write_protected);
    }
    return data_transfer(lba, input, true);
  }

  [[nodiscard]] auto sync() -> result<void, error_type> {
    if (!initialized_) {
      return failure(hal::block::error_kind::other);
    }
    return command(13U, relative_address_, response::short_response)
               ? result<void, error_type>::success()
               : failure(last_error_);
  }

  [[nodiscard]] auto trim(std::uint64_t first_lba, std::uint64_t count)
      -> result<void, error_type> {
    if (!initialized_ || count == 0U || first_lba >= block_count_ ||
        count > block_count_ - first_lba || !address_fits(first_lba, count)) {
      return failure(hal::block::error_kind::out_of_range);
    }
    const std::uint32_t first = address(first_lba);
    const std::uint32_t last = address(first_lba + count - 1U);
    if (!command(32U, first, response::short_response) ||
        !command(33U, last, response::short_response) ||
        !command(38U, 0U, response::short_response)) {
      return failure(last_error_);
    }
    return wait_card_ready();
  }

  [[nodiscard]] static constexpr unsigned instance() noexcept {
    return Instance;
  }

private:
  enum class response : std::uint8_t { none, short_response, long_response };
  static constexpr std::uint32_t power_on{3U};
  static constexpr std::uint32_t card_power_up{1U << 31U};
  static constexpr std::uint32_t card_capacity{1U << 30U};
  static constexpr std::uint32_t command_index_mask{0x3FU};
  static constexpr std::uint32_t command_data_transfer{1U << 6U};
  static constexpr std::uint32_t command_response_short{1U << 8U};
  static constexpr std::uint32_t command_response_long{3U << 8U};
  static constexpr std::uint32_t command_cpsm_enable{1U << 12U};
  static constexpr std::uint32_t status_command_crc_fail{1U << 0U};
  static constexpr std::uint32_t status_data_crc_fail{1U << 1U};
  static constexpr std::uint32_t status_command_timeout{1U << 2U};
  static constexpr std::uint32_t status_data_timeout{1U << 3U};
  static constexpr std::uint32_t status_tx_underrun{1U << 4U};
  static constexpr std::uint32_t status_rx_overrun{1U << 5U};
  static constexpr std::uint32_t status_command_response{1U << 6U};
  static constexpr std::uint32_t status_command_sent{1U << 7U};
  static constexpr std::uint32_t status_data_end{1U << 8U};
  static constexpr std::uint32_t status_tx_fifo_half_empty{1U << 14U};
  static constexpr std::uint32_t status_rx_fifo_half_full{1U << 15U};
  static constexpr std::uint32_t data_enable{1U << 0U};
  static constexpr std::uint32_t data_direction_to_host{1U << 1U};
  static constexpr std::uint32_t data_block_size_512{9U << 4U};
  static constexpr std::uint32_t idma_enable{1U << 0U};
  static constexpr std::uint32_t status_idma_error{1U << 27U};
  static constexpr std::uint32_t status_idma_complete{1U << 28U};
  static constexpr std::uint32_t dma_enable{1U << 0U};
  static constexpr std::uint32_t dma_direction_memory_to_peripheral{1U << 6U};
  static constexpr std::uint32_t dma_memory_increment{1U << 10U};
  static constexpr std::uint32_t dma_peripheral_word{2U << 11U};
  static constexpr std::uint32_t dma_memory_word{2U << 13U};
  static constexpr std::uint32_t clock_width_mask{3U << 14U};
  static constexpr std::uint32_t clock_width_4_bit{1U << 14U};

  [[nodiscard]] auto failure(hal::block::error_kind kind)
      -> result<void, error_type> {
    last_error_ = kind;
    return result<void, error_type>::failure(error{kind});
  }

  [[nodiscard]] bool command(std::uint32_t index, std::uint32_t argument,
                             response response_type,
                             bool data_transfer = false) noexcept {
    clear_status();
    registers_.ARG = argument;
    const std::uint32_t response_bits =
        response_type == response::long_response    ? command_response_long
        : response_type == response::short_response ? command_response_short
                                                    : 0U;
    registers_.CMD = (index & command_index_mask) | response_bits |
                     (data_transfer ? command_data_transfer : 0U) |
                     command_cpsm_enable;
    for (std::uint32_t remaining = configuration_.timeout.iterations;
         remaining > 0U; --remaining) {
      const std::uint32_t status = registers_.STA;
      if ((status & (status_command_timeout | status_command_crc_fail)) != 0U) {
        last_error_ = (status & status_command_timeout) != 0U
                          ? hal::block::error_kind::timeout
                          : hal::block::error_kind::io;
        clear_status();
        return false;
      }
      if ((status & (status_command_response | status_command_sent)) != 0U) {
        clear_status();
        return true;
      }
    }
    last_error_ = hal::block::error_kind::timeout;
    return false;
  }

  void clear_status() noexcept {
    registers_.ICR = status_command_crc_fail | status_data_crc_fail |
                     status_command_timeout | status_data_timeout |
                     status_tx_underrun | status_rx_overrun |
                     status_command_response | status_command_sent |
                     status_data_end | status_idma_error | status_idma_complete;
  }

  void decode_csd() noexcept {
    const std::uint32_t words[4U] = {registers_.RESP1, registers_.RESP2,
                                     registers_.RESP3, registers_.RESP4};
    const std::uint32_t version = bits(words, 127U, 126U);
    if (version == 1U) {
      const std::uint32_t c_size = bits(words, 73U, 62U);
      const std::uint32_t multiplier = bits(words, 49U, 47U);
      const std::uint32_t block_length = bits(words, 83U, 80U);
      const std::uint64_t bytes = (static_cast<std::uint64_t>(c_size) + 1U) *
                                  (std::uint64_t{1U} << (multiplier + 2U)) *
                                  (std::uint64_t{1U} << block_length);
      block_count_ = bytes / 512U;
    } else {
      const std::uint32_t c_size = bits(words, 69U, 48U);
      block_count_ = (static_cast<std::uint64_t>(c_size) + 1U) * 1024U;
    }
  }

  [[nodiscard]] static std::uint32_t
  bits(const std::uint32_t (&words)[4U], unsigned high, unsigned low) noexcept {
    std::uint32_t value = 0U;
    for (unsigned bit = low; bit <= high; ++bit) {
      const unsigned index = 127U - bit;
      const unsigned word = index / 32U;
      const unsigned shift = 31U - index % 32U;
      value = (value << 1U) | ((words[word] >> shift) & 1U);
    }
    return value;
  }

  [[nodiscard]] std::uint32_t address(std::uint64_t lba) const noexcept {
    return high_capacity_ ? static_cast<std::uint32_t>(lba)
                          : static_cast<std::uint32_t>(lba * 512U);
  }

  template <class Buffer>
  [[nodiscard]] auto data_transfer(std::uint64_t lba, Buffer buffer, bool write)
      -> result<void, error_type> {
    const std::size_t size = buffer.size();
    if (!initialized_) {
      return failure(hal::block::error_kind::other);
    }
    if (!buffer.valid() || size == 0U || size % 512U != 0U ||
        size % sizeof(std::uint32_t) != 0U ||
        (reinterpret_cast<std::uintptr_t>(buffer.data()) %
         alignof(std::uint32_t)) != 0U) {
      return failure(hal::block::error_kind::not_aligned);
    }
    const std::size_t block_count = size / 512U;
    if (block_count > 0xFFFF'FFFFULL || lba >= block_count_ ||
        block_count > block_count_ - lba ||
        !address_fits(lba, static_cast<std::uint64_t>(block_count)) ||
        size > 0x01FF'FFFFU ||
        (use_dma() && !has_internal_idma() &&
         size / sizeof(std::uint32_t) > 65'535U)) {
      return failure(hal::block::error_kind::out_of_range);
    }
    const std::uint32_t blocks = static_cast<std::uint32_t>(size / 512U);
    const std::uint32_t command_index =
        write ? (blocks == 1U ? 24U : 25U) : (blocks == 1U ? 17U : 18U);
    registers_.DTIMER = 0xFFFF'FFFFU;
    registers_.DLEN = static_cast<std::uint32_t>(size);
    registers_.DCTRL = data_block_size_512 |
                       (write ? 0U : data_direction_to_host) | data_enable;
    if (!prepare_data_move(buffer, write)) {
      return failure(hal::block::error_kind::other);
    }
    barrier();
    if (!command(command_index, address(lba), response::short_response, true)) {
      finish_data_move();
      return failure(last_error_);
    }
    const auto moved = move_data(buffer, write);
    barrier();
    finish_data_move();
    if (!moved) {
      return moved;
    }
    if (blocks > 1U && !command(12U, 0U, response::short_response)) {
      return failure(last_error_);
    }
    return wait_card_ready();
  }

  template <class Buffer>
  [[nodiscard]] auto move_data(Buffer buffer, bool write)
      -> result<void, error_type> {
    if (use_dma()) {
      if constexpr (has_internal_idma()) {
        (void)buffer;
        (void)write;
        for (std::uint32_t remaining = configuration_.timeout.iterations;
             remaining > 0U; --remaining) {
          const std::uint32_t status = registers_.STA;
          if ((status &
               (status_idma_error | status_data_crc_fail | status_data_timeout |
                status_rx_overrun | status_tx_underrun)) != 0U) {
            last_error_ = hal::block::error_kind::io;
            clear_status();
            return failure(last_error_);
          }
          if ((status & (status_data_end | status_idma_complete)) != 0U) {
            clear_status();
            return result<void, error_type>::success();
          }
        }
        return failure(hal::block::error_kind::timeout);
      } else if constexpr (std::is_same_v<DmaStream, no_dma> ||
                           std::is_same_v<DmamuxChannel, no_dma>) {
        return failure(hal::block::error_kind::other);
      } else {
        for (std::uint32_t remaining = configuration_.timeout.iterations;
             remaining > 0U; --remaining) {
          const std::uint32_t status = registers_.STA;
          if ((status & (status_data_crc_fail | status_data_timeout |
                         status_rx_overrun | status_tx_underrun)) != 0U) {
            stream_->CR = 0U;
            clear_status();
            return failure(hal::block::error_kind::io);
          }
          if (stream_->NDTR == 0U && (status & status_data_end) != 0U) {
            return result<void, error_type>::success();
          }
        }
        return failure(hal::block::error_kind::timeout);
      }
    }
    if constexpr (std::is_same_v<typename Buffer::element_type,
                                 const std::byte>) {
      if (!write) {
        return failure(hal::block::error_kind::other);
      }
      for (std::size_t index = 0U; index < buffer.size() / 4U; ++index) {
        if (!wait_fifo(status_tx_fifo_half_empty, status_tx_underrun)) {
          return failure(hal::block::error_kind::io);
        }
        registers_.FIFO = load_word(buffer.data() + index * 4U);
      }
    } else {
      if (write) {
        for (std::size_t index = 0U; index < buffer.size() / 4U; ++index) {
          if (!wait_fifo(status_tx_fifo_half_empty, status_tx_underrun)) {
            return failure(hal::block::error_kind::io);
          }
          registers_.FIFO = load_word(buffer.data() + index * 4U);
        }
      } else {
        for (std::size_t index = 0U; index < buffer.size() / 4U; ++index) {
          if (!wait_fifo(status_rx_fifo_half_full, status_rx_overrun)) {
            return failure(hal::block::error_kind::io);
          }
          store_word(buffer.data() + index * 4U, registers_.FIFO);
        }
      }
    }
    return (registers_.STA & status_data_end) != 0U
               ? result<void, error_type>::success()
               : failure(hal::block::error_kind::timeout);
  }

  [[nodiscard]] auto wait_card_ready() -> result<void, error_type> {
    for (std::uint32_t remaining = configuration_.timeout.iterations;
         remaining > 0U; --remaining) {
      if (command(13U, relative_address_, response::short_response) &&
          (registers_.RESP1 & 0x100U) != 0U) {
        return result<void, error_type>::success();
      }
    }
    return failure(hal::block::error_kind::timeout);
  }

  [[nodiscard]] bool use_dma() const noexcept {
    return configuration_.use_dma &&
           (has_internal_idma() || (stream_ != nullptr && dmamux_ != nullptr));
  }

  [[nodiscard]] bool address_fits(std::uint64_t lba,
                                  std::uint64_t blocks) const noexcept {
    if (blocks == 0U) {
      return false;
    }
    const std::uint64_t last_lba = lba + blocks - 1U;
    if (last_lba < lba) {
      return false;
    }
    return high_capacity_ ? last_lba <= 0xFFFF'FFFFULL
                          : last_lba <= 0xFFFF'FFFFULL / 512U;
  }

  [[nodiscard]] static constexpr bool has_internal_idma() noexcept {
    return requires(Registers &registers) {
      registers.IDMACTRL;
      registers.IDMABASER;
    };
  }

  template <class Buffer>
  [[nodiscard]] bool prepare_data_move(Buffer buffer, bool write) noexcept {
    if (!use_dma()) {
      return true;
    }
    if constexpr (has_internal_idma()) {
      (void)write;
      registers_.IDMACTRL = 0U;
      registers_.IDMABASER = pointer_word(buffer.data());
      if constexpr (requires(Registers &registers) { registers.IDMALAR; }) {
        registers_.IDMALAR = 0U;
      }
      registers_.IDMACTRL = idma_enable;
      return true;
    } else if constexpr (std::is_same_v<DmaStream, no_dma> ||
                         std::is_same_v<DmamuxChannel, no_dma>) {
      (void)buffer;
      (void)write;
      return false;
    } else {
      stream_->CR = stream_->CR & ~dma_enable;
      for (std::uint32_t remaining = configuration_.timeout.iterations;
           remaining > 0U; --remaining) {
        if ((stream_->CR & dma_enable) == 0U) {
          break;
        }
      }
      if ((stream_->CR & dma_enable) != 0U) {
        return false;
      }
      dmamux_->CCR = configuration_.dma_request;
      stream_->CR = 0U;
      stream_->PAR = pointer_word(&registers_.FIFO);
      stream_->M0AR = pointer_word(buffer.data());
      stream_->NDTR = static_cast<std::uint32_t>(buffer.size() / 4U);
      stream_->FCR = 0U;
      stream_->CR = dma_enable | dma_memory_increment | dma_peripheral_word |
                    dma_memory_word |
                    (write ? dma_direction_memory_to_peripheral : 0U);
      return true;
    }
  }

  void finish_data_move() noexcept {
    if constexpr (has_internal_idma()) {
      if (configuration_.use_dma) {
        registers_.IDMACTRL = 0U;
      }
    } else if constexpr (!std::is_same_v<DmaStream, no_dma> &&
                         !std::is_same_v<DmamuxChannel, no_dma>) {
      stream_->CR = stream_->CR & ~dma_enable;
      for (std::uint32_t remaining = configuration_.timeout.iterations;
           remaining > 0U; --remaining) {
        if ((stream_->CR & dma_enable) == 0U) {
          break;
        }
      }
      stream_->CR = 0U;
    }
    registers_.DCTRL = registers_.DCTRL & ~data_enable;
  }

  [[nodiscard]] static std::uint32_t load_word(const std::byte *data) noexcept {
    return static_cast<std::uint32_t>(std::to_integer<std::uint8_t>(data[0U])) |
           (static_cast<std::uint32_t>(std::to_integer<std::uint8_t>(data[1U]))
            << 8U) |
           (static_cast<std::uint32_t>(std::to_integer<std::uint8_t>(data[2U]))
            << 16U) |
           (static_cast<std::uint32_t>(std::to_integer<std::uint8_t>(data[3U]))
            << 24U);
  }

  [[nodiscard]] bool wait_fifo(std::uint32_t ready,
                               std::uint32_t fault) const noexcept {
    for (std::uint32_t remaining = configuration_.timeout.iterations;
         remaining > 0U; --remaining) {
      if ((registers_.STA & ready) != 0U) {
        return true;
      }
      if ((registers_.STA & (fault | status_data_timeout)) != 0U) {
        return false;
      }
    }
    return false;
  }

  static void store_word(std::byte *data, std::uint32_t value) noexcept {
    data[0U] = static_cast<std::byte>(value & 0xFFU);
    data[1U] = static_cast<std::byte>((value >> 8U) & 0xFFU);
    data[2U] = static_cast<std::byte>((value >> 16U) & 0xFFU);
    data[3U] = static_cast<std::byte>((value >> 24U) & 0xFFU);
  }

  [[nodiscard]] static std::uint32_t
  pointer_word(const volatile void *pointer) noexcept {
    return static_cast<std::uint32_t>(
        reinterpret_cast<std::uintptr_t>(pointer));
  }

  static void barrier() noexcept {
#if defined(__arm__) || defined(__thumb__)
    __asm volatile("dsb 0xF" ::: "memory");
#else
    std::atomic_thread_fence(std::memory_order_seq_cst);
#endif
  }

  Registers &registers_;
  config configuration_{};
  stream_type *stream_{};
  mux_type *dmamux_{};
  bool initialized_{};
  bool high_capacity_{};
  std::uint32_t relative_address_{};
  std::uint64_t block_count_{};
  hal::block::error_kind last_error_{hal::block::error_kind::other};
};

} // namespace hal::stm32h5::block
