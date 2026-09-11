#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/nv.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include <cstddef>
#include <cstdint>
#include <atomic>
#include "support.hpp"

namespace hal::stm32g4::nv {

inline constexpr capability support{
    implementation_status::available,
    "STM32G4 internal dual-bank flash double-word program and page erase"};

class error {
public:
  explicit constexpr error(hal::nv::flash_error_kind kind) noexcept : kind_{kind} {}
  [[nodiscard]] constexpr hal::nv::flash_error_kind kind() const noexcept {
    return kind_;
  }

private:
  hal::nv::flash_error_kind kind_{};
};

// The STM32G4 512-KiB configuration is represented as two 256-KiB banks of
// 2-KiB pages. These parameters stay template arguments so another G4
// ordering code can supply its exact geometry without changing the driver.
template <class Registers, std::uintptr_t Base = 0x08000000U,
          std::size_t Capacity = 512U * 1024U, std::size_t ProgramSize = 8U,
          std::size_t PageSize = 2U * 1024U,
          std::size_t BankSize = 256U * 1024U>
class NorFlash {
  static_assert(Capacity > 0U && ProgramSize == 8U && PageSize > 0U &&
                BankSize > 0U && Capacity % PageSize == 0U &&
                Capacity % BankSize == 0U && BankSize % PageSize == 0U);

public:
  using error_type = error;

  explicit NorFlash(Registers &registers,
                    poll_budget timeout = {1'000'000U}) noexcept
      : registers_{registers}, timeout_{timeout} {}

  NorFlash(const NorFlash &) = delete;
  NorFlash &operator=(const NorFlash &) = delete;

  [[nodiscard]] static constexpr hal::nv::flash_geometry geometry() noexcept {
    return {Capacity, 1U, ProgramSize, PageSize};
  }

  [[nodiscard]] auto read(std::size_t offset, hal::span<std::byte> output)
      -> result<void, error_type> {
    if (!output.valid() || !in_range(offset, output.size())) {
      return failure(hal::nv::flash_error_kind::out_of_range);
    }
    volatile const std::byte *source =
        reinterpret_cast<volatile const std::byte *>(Base + offset);
    for (std::size_t i = 0U; i < output.size(); ++i) output[i] = source[i];
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
    unlock();
    for (std::size_t i = 0U; i < input.size(); i += ProgramSize) {
      if (!wait_ready()) return operation_failure(hal::nv::flash_error_kind::program);
      const std::uint64_t desired = load_double_word(input.data() + i);
      const auto current = *reinterpret_cast<volatile const std::uint64_t *>(
          Base + offset + i);
      if ((current & desired) != desired) {
        return operation_failure(hal::nv::flash_error_kind::program);
      }
      clear_status();
      registers_.CR = registers_.CR | control_program;
      auto *destination = reinterpret_cast<volatile std::uint32_t *>(Base + offset + i);
      destination[0U] = static_cast<std::uint32_t>(desired);
      instruction_barrier();
      destination[1U] = static_cast<std::uint32_t>(desired >> 32U);
      if (!wait_ready() || (registers_.SR & error_flags) != 0U) {
        return operation_failure(hal::nv::flash_error_kind::program);
      }
      registers_.CR = registers_.CR & ~control_program;
    }
    lock();
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto erase(std::size_t offset, std::size_t length)
      -> result<void, error_type> {
    if (!valid_erase_range(offset, length)) {
      return failure(hal::nv::flash_error_kind::not_aligned);
    }
    if (!in_range(offset, length)) {
      return failure(hal::nv::flash_error_kind::out_of_range);
    }
    unlock();
    for (std::size_t page_offset = offset; page_offset < offset + length;
         page_offset += PageSize) {
      if (!wait_ready()) return operation_failure(hal::nv::flash_error_kind::erase);
      clear_status();
      const std::size_t bank = page_offset / BankSize;
      const std::size_t page = (page_offset % BankSize) / PageSize;
      registers_.CR = control_page_erase |
                      (static_cast<std::uint32_t>(page) << page_number_shift) |
                      (bank == 1U ? control_bank_2 : 0U);
      registers_.CR = registers_.CR | control_start;
      if (!wait_ready() || (registers_.SR & error_flags) != 0U) {
        return operation_failure(hal::nv::flash_error_kind::erase);
      }
      registers_.CR = registers_.CR & ~(control_page_erase | control_bank_2);
    }
    lock();
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto sync() -> result<void, error_type> {
    return wait_ready() && (registers_.SR & error_flags) == 0U
               ? result<void, error_type>::success()
               : failure(hal::nv::flash_error_kind::io);
  }

private:
  static constexpr std::uint32_t status_eop{1U << 0U};
  static constexpr std::uint32_t status_error{(1U << 1U) | (1U << 3U) |
                                              (1U << 4U) | (1U << 5U) |
                                              (1U << 6U) | (1U << 7U) |
                                              (1U << 8U) | (1U << 9U) |
                                              (1U << 14U) | (1U << 15U)};
  static constexpr std::uint32_t status_busy{1U << 16U};
  static constexpr std::uint32_t control_program{1U << 0U};
  static constexpr std::uint32_t control_page_erase{1U << 1U};
  static constexpr std::uint32_t control_bank_2{1U << 11U};
  static constexpr std::uint32_t control_start{1U << 16U};
  static constexpr std::uint32_t control_lock{1U << 31U};
  static constexpr std::uint32_t page_number_shift{3U};
  static constexpr std::uint32_t key_1{0x45670123U};
  static constexpr std::uint32_t key_2{0xCDEF89ABU};
  static constexpr std::uint32_t error_flags{status_error};

  [[nodiscard]] bool in_range(std::size_t offset, std::size_t length) const noexcept {
    return offset <= Capacity && length <= Capacity - offset;
  }

  [[nodiscard]] bool valid_erase_range(std::size_t offset,
                                       std::size_t length) const noexcept {
    return offset % PageSize == 0U && length % PageSize == 0U;
  }

  [[nodiscard]] bool wait_ready() const noexcept {
    if (!timeout_.valid()) return false;
    for (std::uint32_t left = timeout_.iterations; left > 0U; --left) {
      if ((registers_.SR & status_busy) == 0U) return true;
    }
    return false;
  }

  void clear_status() noexcept { registers_.SR = status_eop | error_flags; }
  void unlock() noexcept {
    registers_.KEYR = key_1;
    registers_.KEYR = key_2;
  }
  void lock() noexcept { registers_.CR = registers_.CR | control_lock; }

  [[nodiscard]] auto failure(hal::nv::flash_error_kind kind)
      -> result<void, error_type> {
    registers_.CR = registers_.CR & ~(control_program | control_page_erase);
    lock();
    return result<void, error_type>::failure(error{kind});
  }

  [[nodiscard]] auto operation_failure(hal::nv::flash_error_kind kind)
      -> result<void, error_type> {
    return failure(kind);
  }

  [[nodiscard]] static std::uint64_t load_double_word(const std::byte *data) noexcept {
    std::uint64_t value{};
    for (unsigned i = 0U; i < 8U; ++i) {
      value |= static_cast<std::uint64_t>(std::to_integer<std::uint8_t>(data[i]))
               << (i * 8U);
    }
    return value;
  }

  static void instruction_barrier() noexcept {
#if defined(__arm__) || defined(__thumb__)
    __asm volatile("isb 0xF" ::: "memory");
#else
    std::atomic_thread_fence(std::memory_order_seq_cst);
#endif
  }

  Registers &registers_;
  poll_budget timeout_{};
};

} // namespace hal::stm32g4::nv
