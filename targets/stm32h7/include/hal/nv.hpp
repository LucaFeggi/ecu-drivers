#pragma once

#include <cstddef>
#include <cstdint>
#include <hal/nv.hpp>
#include <hal/stm32h7/support.hpp>

namespace hal::stm32h7::nv {

inline constexpr capability support{
    implementation_status::available,
    "direct STM32H7 internal NOR flash program/erase access"};

class error {
public:
  explicit constexpr error(hal::nv::flash_error_kind kind) noexcept : kind_{kind} {}

  [[nodiscard]] constexpr hal::nv::flash_error_kind kind() const noexcept {
    return kind_;
  }

private:
  hal::nv::flash_error_kind kind_{};
};

struct region {
  std::uintptr_t base{};
  std::size_t capacity{};
  std::size_t program_size{32U};
  std::size_t erase_size{128U * 1024U};
};

// STM32H7 flash is programmed as a 256-bit flash word. Erase sector numbering
// and bank selection are exposed as compile-time policy because they differ
// between H7 subfamilies. The BSP supplies the mapped base address and chooses
// the bank/sector policy without duplicating the peripheral algorithm.
template <class Registers, std::uintptr_t Base, std::size_t Capacity,
          std::size_t ProgramSize = 32U,
          std::size_t EraseSize = 128U * 1024U, bool BankTwo = false>
class NorFlash {
  static_assert(Capacity > 0U);
  static_assert((Base % ProgramSize) == 0U);
  static_assert(ProgramSize == 32U);
  static_assert(EraseSize > 0U);
  static_assert(Capacity % EraseSize == 0U);
  // STM32H723 exposes a single bank control register. A future dual-bank
  // profile must provide a different register policy instead of silently
  // treating BankTwo as bank one.
  static_assert(!BankTwo);

public:
  using error_type = error;

  explicit NorFlash(Registers &registers,
                    poll_budget timeout = {1'000'000U}) noexcept
      : registers_{registers}, timeout_{timeout} {}

  NorFlash(const NorFlash &) = delete;
  NorFlash &operator=(const NorFlash &) = delete;

  [[nodiscard]] static constexpr hal::nv::flash_geometry geometry() noexcept {
    return {Capacity, 1U, ProgramSize, EraseSize};
  }

  [[nodiscard]] auto read(std::size_t offset, hal::span<std::byte> output)
      -> result<void, error_type> {
    if (!output.valid() || !in_range(offset, output.size())) {
      return failure(hal::nv::flash_error_kind::out_of_range);
    }
    volatile const std::byte *source = memory() + offset;
    for (std::size_t index = 0U; index < output.size(); ++index) {
      output[index] = source[index];
    }
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto program(std::size_t offset,
                             hal::span<const std::byte> input)
      -> result<void, error_type> {
    if (!input.valid() || offset % ProgramSize != 0U ||
        input.size() % ProgramSize != 0U) {
      return failure(hal::nv::flash_error_kind::not_aligned);
    }
    if (!in_range(offset, input.size())) {
      return failure(hal::nv::flash_error_kind::out_of_range);
    }
    if (!timeout_.valid()) {
      return failure(hal::nv::flash_error_kind::io);
    }
    unlock();
    for (std::size_t block = 0U; block < input.size(); block += ProgramSize) {
      if (!wait_not_busy()) {
        lock();
        return failure(hal::nv::flash_error_kind::program);
      }
      clear_errors();
      control() = program_enable;
      volatile std::uint32_t *destination =
          reinterpret_cast<volatile std::uint32_t *>(memory() + offset + block);
      for (std::size_t word = 0U; word < ProgramSize / sizeof(std::uint32_t);
           ++word) {
        const std::size_t byte_offset = block + word * sizeof(std::uint32_t);
        const std::uint32_t value =
            static_cast<std::uint32_t>(std::to_integer<std::uint8_t>(input[byte_offset])) |
            (static_cast<std::uint32_t>(std::to_integer<std::uint8_t>(input[byte_offset + 1U])) << 8U) |
            (static_cast<std::uint32_t>(std::to_integer<std::uint8_t>(input[byte_offset + 2U])) << 16U) |
            (static_cast<std::uint32_t>(std::to_integer<std::uint8_t>(input[byte_offset + 3U])) << 24U);
        destination[word] = value;
      }
      if (!wait_not_busy() || (status() & error_flags) != 0U) {
        lock();
        return failure(hal::nv::flash_error_kind::program);
      }
    }
    control() = 0U;
    lock();
    barrier();
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto erase(std::size_t offset, std::size_t length)
      -> result<void, error_type> {
    if (offset % EraseSize != 0U || length % EraseSize != 0U) {
      return failure(hal::nv::flash_error_kind::not_aligned);
    }
    if (!in_range(offset, length)) {
      return failure(hal::nv::flash_error_kind::out_of_range);
    }
    if (!timeout_.valid()) {
      return failure(hal::nv::flash_error_kind::io);
    }
    unlock();
    for (std::size_t position = offset; position < offset + length;
         position += EraseSize) {
      if (!wait_not_busy()) {
        lock();
        return failure(hal::nv::flash_error_kind::erase);
      }
      clear_errors();
      const std::uint32_t sector =
          static_cast<std::uint32_t>(position / EraseSize);
      control() = erase_sector | ((sector & 0x7U) << sector_position);
      control() = control() | start_operation;
      if (!wait_not_busy() || (status() & error_flags) != 0U) {
        lock();
        return failure(hal::nv::flash_error_kind::erase);
      }
    }
    control() = 0U;
    lock();
    barrier();
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto sync() -> result<void, error_type> {
    barrier();
    return result<void, error_type>::success();
  }

private:
  static constexpr std::uint32_t program_enable{1U << 1U};
  static constexpr std::uint32_t erase_sector{1U << 2U};
  static constexpr std::uint32_t start_operation{1U << 7U};
  static constexpr std::uint32_t sector_position{8U};
  static constexpr std::uint32_t busy{1U << 0U};
  static constexpr std::uint32_t error_flags{(1U << 17U) | (1U << 18U) |
                                             (1U << 19U) | (1U << 21U) |
                                             (1U << 22U) | (1U << 23U) |
                                             (1U << 24U) | (1U << 25U) |
                                             (1U << 26U)};
  static constexpr std::uint32_t key_one{0x4567'0123U};
  static constexpr std::uint32_t key_two{0xCDEF'89ABU};

  [[nodiscard]] static constexpr bool in_range(std::size_t offset,
                                               std::size_t length) noexcept {
    return offset <= Capacity && length <= Capacity - offset;
  }

  [[nodiscard]] static volatile std::byte *memory() noexcept {
    return reinterpret_cast<volatile std::byte *>(Base);
  }

  [[nodiscard]] bool wait_not_busy() const noexcept {
    for (std::uint32_t remaining = timeout_.iterations; remaining > 0U;
         --remaining) {
      if ((status() & busy) == 0U) {
        return true;
      }
    }
    return false;
  }

  void unlock() noexcept {
    key_register() = key_one;
    key_register() = key_two;
  }

  void lock() noexcept { control() = control() | (1U << 0U); }

  void clear_errors() noexcept { clear_register() = error_flags; }

  [[nodiscard]] static volatile std::uint32_t &key_register(Registers &registers) noexcept {
    if constexpr (requires { registers.KEYR1; }) {
      return registers.KEYR1;
    } else {
      return registers.KEYR;
    }
  }

  [[nodiscard]] volatile std::uint32_t &key_register() noexcept {
    return key_register(registers_);
  }

  [[nodiscard]] static volatile std::uint32_t &control(Registers &registers) noexcept {
    if constexpr (requires { registers.CR1; }) {
      return registers.CR1;
    } else {
      return registers.CR;
    }
  }

  [[nodiscard]] volatile std::uint32_t &control() noexcept {
    return control(registers_);
  }

  [[nodiscard]] static volatile std::uint32_t &status(Registers &registers) noexcept {
    if constexpr (requires { registers.SR1; }) {
      return registers.SR1;
    } else {
      return registers.SR;
    }
  }

  [[nodiscard]] volatile std::uint32_t &status() const noexcept {
    return status(const_cast<Registers &>(registers_));
  }

  [[nodiscard]] static volatile std::uint32_t &clear_register(Registers &registers) noexcept {
    if constexpr (requires { registers.CCR1; }) {
      return registers.CCR1;
    } else {
      return status(registers);
    }
  }

  [[nodiscard]] volatile std::uint32_t &clear_register() noexcept {
    return clear_register(registers_);
  }

  static void barrier() noexcept {
#if defined(__arm__) || defined(__thumb__)
    __asm volatile("dsb 0xF" ::: "memory");
    __asm volatile("isb 0xF" ::: "memory");
#endif
  }

  [[nodiscard]] static auto failure(hal::nv::flash_error_kind kind)
      -> result<void, error_type> {
    return result<void, error_type>::failure(error{kind});
  }

  Registers &registers_;
  poll_budget timeout_{};
};

} // namespace hal::stm32h7::nv
