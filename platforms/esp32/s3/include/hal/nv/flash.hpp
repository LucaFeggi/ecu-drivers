#pragma once

#include <cstddef>
#include <cstdint>
#include <hal/mcu/esp32s3_wroom_1_n16r8/memory.hpp>
#include <hal/foundation/result.hpp>
#include <hal/foundation/span.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/nv.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif
#include <soc/spi_mem_struct.h>

namespace hal::esp32s3_wroom_1_n16r8::nv {

class error {
 public:
  [[nodiscard]] static constexpr error out_of_range() noexcept {
    return error{hal::nv::flash_error_kind::out_of_range};
  }
  [[nodiscard]] static constexpr error not_aligned() noexcept {
    return error{hal::nv::flash_error_kind::not_aligned};
  }
  [[nodiscard]] static constexpr error program() noexcept {
    return error{hal::nv::flash_error_kind::program};
  }
  [[nodiscard]] static constexpr error erase() noexcept {
    return error{hal::nv::flash_error_kind::erase};
  }
  [[nodiscard]] static constexpr error io() noexcept {
    return error{hal::nv::flash_error_kind::io};
  }
  [[nodiscard]] constexpr hal::nv::flash_error_kind kind() const noexcept {
    return kind_;
  }

 private:
  constexpr explicit error(hal::nv::flash_error_kind kind) noexcept : kind_{kind} {}
  hal::nv::flash_error_kind kind_;
};

// The bootloader establishes the MSPI flash mode and pin timing. This driver
// intentionally reuses that configuration and only issues the controller's
// raw WREN/RDSR/PP/SE operations. Code that can execute while MSPI/cache is
// occupied is placed in IRAM; program input must be in DMA/DRAM-visible memory
// supplied by the BSP.
template <std::uintptr_t MappedBase = 0x3C00'0000U,
          std::size_t Capacity = 16U * 1024U * 1024U,
          std::size_t ProgramSize = 4U, std::size_t EraseSize = 4U * 1024U>
class Flash {
  static_assert(Capacity > 0U && Capacity <= 0x0100'0000U);
  static_assert(ProgramSize == 4U && EraseSize == 4U * 1024U);
  static_assert(Capacity % EraseSize == 0U);

 public:
  using error_type = error;

  explicit Flash(spi_mem_dev_t& peripheral,
                 std::uint32_t poll_limit = 1'000'000U) noexcept
      : peripheral_{&peripheral}, poll_limit_{poll_limit} {}

  Flash(const Flash&) = delete;
  Flash& operator=(const Flash&) = delete;

  [[nodiscard]] static constexpr hal::nv::flash_geometry geometry() noexcept {
    return {Capacity, 1U, ProgramSize, EraseSize};
  }

  ECU_ESP32S3_IRAM [[nodiscard]] result<void, error_type> read(
      std::size_t offset, span<std::byte> output) noexcept {
    if (!output.valid() || !in_range(offset, output.size())) {
      return failure(error::out_of_range());
    }
    volatile const std::byte* source = memory() + offset;
    for (std::size_t index = 0U; index < output.size(); ++index) {
      output[index] = source[index];
    }
    return result<void, error_type>::success();
  }

  ECU_ESP32S3_IRAM [[nodiscard]] result<void, error_type> program(
      std::size_t offset, span<const std::byte> input) noexcept {
    if (!input.valid() || offset % ProgramSize != 0U ||
        input.size() % ProgramSize != 0U) {
      return failure(error::not_aligned());
    }
    if (!in_range(offset, input.size())) {
      return failure(error::out_of_range());
    }
    if (poll_limit_ == 0U) {
      return failure(error::io());
    }

    std::size_t position = 0U;
    while (position < input.size()) {
      const std::size_t absolute = offset + position;
      const std::size_t page_remaining = 64U - (absolute % 64U);
      const std::size_t count =
          (input.size() - position) < page_remaining ? input.size() - position
                                                      : page_remaining;
      if (!write_enable() || !wait_idle()) {
        return failure(error::program());
      }
      peripheral_->addr = static_cast<std::uint32_t>(absolute & 0x00FF'FFFFU) |
                          (static_cast<std::uint32_t>(count) << 24U);
      const std::size_t words = (count + 3U) / 4U;
      for (std::size_t word = 0U; word < words; ++word) {
        const std::size_t base = position + word * 4U;
        std::uint32_t packed = 0xFFFF'FFFFU;
        for (std::size_t byte = 0U; byte < 4U && word * 4U + byte < count;
             ++byte) {
          packed = (packed & ~(0xFFU << (byte * 8U))) |
                   (static_cast<std::uint32_t>(std::to_integer<std::uint8_t>(
                        input[base + byte]))
                    << (byte * 8U));
        }
        peripheral_->data_buf[word] = packed;
      }
      peripheral_->user.usr_dummy = 0U;
      peripheral_->cmd.flash_pp = 1U;
      if (!wait_command() || !wait_idle()) {
        return failure(error::program());
      }
      position += count;
    }
    return result<void, error_type>::success();
  }

  ECU_ESP32S3_IRAM [[nodiscard]] result<void, error_type> erase(
      std::size_t offset, std::size_t length) noexcept {
    if (offset % EraseSize != 0U || length % EraseSize != 0U) {
      return failure(error::not_aligned());
    }
    if (!in_range(offset, length)) {
      return failure(error::out_of_range());
    }
    if (poll_limit_ == 0U) {
      return failure(error::io());
    }
    for (std::size_t position = 0U; position < length; position += EraseSize) {
      if (!write_enable() || !wait_idle()) {
        return failure(error::erase());
      }
      peripheral_->addr = static_cast<std::uint32_t>((offset + position) &
                                                      0x00FF'FFFFU);
      peripheral_->ctrl.val = 0U;
      peripheral_->cmd.flash_se = 1U;
      if (!wait_command() || !wait_idle()) {
        return failure(error::erase());
      }
    }
    return result<void, error_type>::success();
  }

  ECU_ESP32S3_IRAM [[nodiscard]] result<void, error_type> sync() noexcept {
    return wait_idle() ? result<void, error_type>::success()
                       : failure(error::io());
  }

 private:
  [[nodiscard]] static constexpr bool in_range(std::size_t offset,
                                               std::size_t length) noexcept {
    return offset <= Capacity && length <= Capacity - offset;
  }

  [[nodiscard]] static volatile const std::byte* memory() noexcept {
    return reinterpret_cast<volatile const std::byte*>(MappedBase);
  }

  ECU_ESP32S3_IRAM [[nodiscard]] bool write_enable() noexcept {
    peripheral_->cmd.flash_wren = 1U;
    return wait_command();
  }

  ECU_ESP32S3_IRAM [[nodiscard]] bool wait_idle() noexcept {
    std::uint32_t remaining = poll_limit_;
    while (remaining != 0U) {
      peripheral_->rd_status.val = 0U;
      peripheral_->cmd.flash_rdsr = 1U;
      if (!wait_command()) {
        return false;
      }
      if ((peripheral_->rd_status.status & 1U) == 0U) {
        return true;
      }
      --remaining;
    }
    return false;
  }

  ECU_ESP32S3_IRAM [[nodiscard]] bool wait_command() noexcept {
    std::uint32_t remaining = poll_limit_;
    while (remaining != 0U) {
      // The ESP32-S3 MSPI command register contains one-shot command bits;
      // the complete register returns to zero when the operation is done.
      if (peripheral_->cmd.val == 0U) {
        return true;
      }
      --remaining;
    }
    return false;
  }

  [[nodiscard]] static result<void, error_type> failure(error value) noexcept {
    return result<void, error_type>::failure(value);
  }

  spi_mem_dev_t* peripheral_;
  std::uint32_t poll_limit_;
};

}  // namespace hal::esp32s3_wroom_1_n16r8::nv
