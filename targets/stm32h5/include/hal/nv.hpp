#pragma once

#include <cstddef>
#include <cstdint>
#include <hal/nv.hpp>
#include <hal/stm32h5/support.hpp>

namespace hal::stm32h5::nv {

inline constexpr capability support{
    implementation_status::available,
    "STM32H5 256-bit internal NOR flash access"};
class error {
public:
  explicit constexpr error(hal::nv::flash_error_kind kind) noexcept
      : kind_{kind} {}
  [[nodiscard]] constexpr hal::nv::flash_error_kind kind() const noexcept {
    return kind_;
  }

private:
  hal::nv::flash_error_kind kind_{};
};

template <class Registers, std::uintptr_t Base, std::size_t Capacity,
          std::size_t ProgramSize = 32U, std::size_t EraseSize = 8U * 1024U,
          bool BankTwo = false>
class NorFlash {
  static_assert(Capacity > 0U && ProgramSize == 32U &&
                Capacity % EraseSize == 0U);

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
  [[nodiscard]] auto read(std::size_t offset, span<std::byte> output)
      -> result<void, error_type> {
    if (!output.valid() || !in_range(offset, output.size()))
      return failure(hal::nv::flash_error_kind::out_of_range);
    volatile const std::byte *source =
        reinterpret_cast<volatile const std::byte *>(Base + offset);
    for (std::size_t i = 0U; i < output.size(); ++i)
      output[i] = source[i];
    return result<void, error_type>::success();
  }
  [[nodiscard]] auto program(std::size_t offset, span<const std::byte> input)
      -> result<void, error_type> {
    if (!input.valid() || offset % ProgramSize != 0U ||
        input.size() % ProgramSize != 0U)
      return failure(hal::nv::flash_error_kind::not_aligned);
    if (!in_range(offset, input.size()) || !timeout_.valid())
      return failure(hal::nv::flash_error_kind::out_of_range);
    unlock();
    for (std::size_t block = 0U; block < input.size(); block += ProgramSize) {
      if (!wait_ready()) {
        lock();
        return failure(hal::nv::flash_error_kind::program);
      }
      clear_errors();
      control() = bank_selection | program_enable;
      auto *destination =
          reinterpret_cast<volatile std::uint32_t *>(Base + offset + block);
      for (std::size_t word = 0U; word < ProgramSize / sizeof(std::uint32_t);
           ++word)
        destination[word] =
            load_word(input.data() + block + word * sizeof(std::uint32_t));
      if (!wait_ready() || (status() & error_flags) != 0U) {
        lock();
        return failure(hal::nv::flash_error_kind::program);
      }
    }
    control() = 0U;
    lock();
    return result<void, error_type>::success();
  }
  [[nodiscard]] auto erase(std::size_t offset, std::size_t length)
      -> result<void, error_type> {
    if (offset % EraseSize != 0U || length % EraseSize != 0U ||
        !in_range(offset, length) || !timeout_.valid())
      return failure(hal::nv::flash_error_kind::not_aligned);
    unlock();
    for (std::size_t p = offset; p < offset + length; p += EraseSize) {
      if (!wait_ready()) {
        lock();
        return failure(hal::nv::flash_error_kind::erase);
      }
      clear_errors();
      control() =
          bank_selection | erase_sector |
          (static_cast<std::uint32_t>(p / EraseSize) << sector_position) |
          start_operation;
      if (!wait_ready() || (status() & error_flags) != 0U) {
        lock();
        return failure(hal::nv::flash_error_kind::erase);
      }
    }
    control() = bank_selection;
    lock();
    return result<void, error_type>::success();
  }
  [[nodiscard]] auto sync() -> result<void, error_type> {
    return wait_ready() ? result<void, error_type>::success()
                        : failure(hal::nv::flash_error_kind::io);
  }

private:
  static constexpr std::uint32_t lock_bit{1U};
  static constexpr std::uint32_t program_enable{1U << 1U};
  static constexpr std::uint32_t erase_sector{1U << 2U};
  static constexpr std::uint32_t start_operation{1U << 5U};
  static constexpr std::uint32_t sector_position{6U};
  static constexpr std::uint32_t bank_select{1U << 31U};
  static constexpr std::uint32_t bank_selection = BankTwo ? bank_select : 0U;
  static constexpr std::uint32_t busy{1U};
  static constexpr std::uint32_t error_flags{
      (1U << 17U) | (1U << 18U) | (1U << 19U) | (1U << 20U) | (1U << 21U) |
      (1U << 22U) | (1U << 23U)};
  [[nodiscard]] auto failure(hal::nv::flash_error_kind kind)
      -> result<void, error_type> {
    lock();
    return result<void, error_type>::failure(error{kind});
  }
  [[nodiscard]] bool in_range(std::size_t offset,
                              std::size_t length) const noexcept {
    return offset <= Capacity && length <= Capacity - offset;
  }
  [[nodiscard]] volatile std::uint32_t &control() noexcept {
    return registers_.NSCR;
  }
  [[nodiscard]] std::uint32_t status() const noexcept {
    return registers_.NSSR;
  }
  void unlock() noexcept {
    control() = (control() & lock_bit) | bank_selection;
    registers_.NSKEYR = 0x45670123U;
    registers_.NSKEYR = 0xCDEF89ABU;
  }
  void lock() noexcept { control() = (control() & bank_select) | lock_bit; }
  void clear_errors() noexcept { registers_.NSCCR = error_flags; }
  [[nodiscard]] bool wait_ready() const noexcept {
    for (std::uint32_t n = timeout_.iterations; n > 0U; --n)
      if ((status() & busy) == 0U)
        return true;
    return false;
  }
  static std::uint32_t load_word(const std::byte *data) noexcept {
    return std::to_integer<std::uint8_t>(data[0U]) |
           (static_cast<std::uint32_t>(std::to_integer<std::uint8_t>(data[1U]))
            << 8U) |
           (static_cast<std::uint32_t>(std::to_integer<std::uint8_t>(data[2U]))
            << 16U) |
           (static_cast<std::uint32_t>(std::to_integer<std::uint8_t>(data[3U]))
            << 24U);
  }
  Registers &registers_;
  poll_budget timeout_{};
};
} // namespace hal::stm32h5::nv
