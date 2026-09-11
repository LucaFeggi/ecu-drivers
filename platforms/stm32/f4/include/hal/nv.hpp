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
#include "support.hpp"

namespace hal::stm32f4::nv {
inline constexpr capability support{
    implementation_status::available,
    "STM32F4 internal flash program and sector erase"};
class error {
public:
  explicit constexpr error(hal::nv::flash_error_kind k) noexcept : kind_{k} {}
  [[nodiscard]] constexpr hal::nv::flash_error_kind kind() const noexcept {
    return kind_;
  }

private:
  hal::nv::flash_error_kind kind_{};
};

template <class Registers, std::uintptr_t Base, std::size_t Capacity,
          std::size_t ProgramSize = 4U, std::size_t EraseSize = 16U * 1024U>
class NorFlash {
  static_assert(Capacity > 0U && ProgramSize == 4U && EraseSize == 16U * 1024U);

public:
  using error_type = error;
  explicit NorFlash(Registers &r, poll_budget timeout = {1'000'000U}) noexcept
      : registers_{r}, timeout_{timeout} {}
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
    if (!in_range(offset, input.size()))
      return failure(hal::nv::flash_error_kind::out_of_range);
    unlock();
    for (std::size_t i = 0U; i < input.size(); i += 4U) {
      if (!wait_ready())
        return failure(hal::nv::flash_error_kind::program);
      registers_.SR = error_flags;
      registers_.CR = program_word | program_parallelism;
      *reinterpret_cast<volatile std::uint32_t *>(Base + offset + i) =
          load_word(input.data() + i);
      if (!wait_ready())
        return failure(hal::nv::flash_error_kind::program);
      registers_.CR &= ~((1U << 0U) | (1U << 1U));
    }
    lock();
    return result<void, error_type>::success();
  }
  [[nodiscard]] auto erase(std::size_t offset, std::size_t length)
      -> result<void, error_type> {
    if (!in_range(offset, length))
      return failure(hal::nv::flash_error_kind::not_aligned);
    if (!valid_erase_range(offset, length))
      return failure(hal::nv::flash_error_kind::not_aligned);
    unlock();
    const auto end = offset + length;
    for (std::size_t p = offset; p < end;) {
      const auto sector = sector_for(p);
      const auto size = sector_size(sector);
      if (!wait_ready())
        return failure(hal::nv::flash_error_kind::erase);
      registers_.SR = error_flags;
      registers_.CR = sector_erase | (sector << sector_number_shift);
      registers_.CR |= start;
      if (!wait_ready())
        return failure(hal::nv::flash_error_kind::erase);
      p += size;
    }
    registers_.CR = 0U;
    lock();
    return result<void, error_type>::success();
  }
  [[nodiscard]] auto sync() -> result<void, error_type> {
    return wait_ready() ? result<void, error_type>::success()
                        : failure(hal::nv::flash_error_kind::io);
  }

private:
  static constexpr std::uint32_t busy{1U << 16U};
  static constexpr std::uint32_t error_flags{
      (1U << 0U) | (1U << 4U) | (1U << 5U) | (1U << 6U) | (1U << 7U)};
  static constexpr std::uint32_t program_word{1U << 0U};
  static constexpr std::uint32_t program_parallelism{2U << 8U};
  static constexpr std::uint32_t sector_erase{1U << 1U};
  static constexpr std::uint32_t sector_number_shift{3U};
  static constexpr std::uint32_t start{1U << 16U};
  template <class T = void>
  [[nodiscard]] auto failure(hal::nv::flash_error_kind k)
      -> result<T, error_type> {
    registers_.CR &= ~(program_word | sector_erase | program_parallelism);
    lock();
    return result<T, error_type>::failure(error{k});
  }
  [[nodiscard]] bool in_range(std::size_t offset,
                              std::size_t length) const noexcept {
    return offset <= Capacity && length <= Capacity - offset;
  }
  [[nodiscard]] bool wait_ready() const noexcept {
    for (std::uint32_t n = timeout_.iterations; n > 0U; --n)
      if ((registers_.SR & busy) == 0U)
        return true;
    return false;
  }
  void unlock() noexcept {
    registers_.KEYR = 0x45670123U;
    registers_.KEYR = 0xCDEF89ABU;
  }
  void lock() noexcept { registers_.CR |= 1U << 31U; }
  [[nodiscard]] static constexpr std::uint32_t
  sector_for(std::size_t offset) noexcept {
    return offset < 16U * 1024U   ? 0U
           : offset < 32U * 1024U ? 1U
           : offset < 48U * 1024U ? 2U
           : offset < 64U * 1024U ? 3U
           : offset < 128U * 1024U
               ? 4U
               : static_cast<std::uint32_t>(
                     5U + ((offset - 128U * 1024U) / (128U * 1024U)));
  }
  [[nodiscard]] static constexpr std::size_t
  sector_size(std::uint32_t sector) noexcept {
    return sector < 4U    ? 16U * 1024U
           : sector == 4U ? 64U * 1024U
                          : 128U * 1024U;
  }
  [[nodiscard]] bool valid_erase_range(std::size_t offset,
                                       std::size_t length) const noexcept {
    if (!in_range(offset, length))
      return false;
    const auto end = offset + length;
    for (std::size_t p = offset; p < end;) {
      const auto size = sector_size(sector_for(p));
      if (p % size != 0U || size > end - p)
        return false;
      p += size;
    }
    return true;
  }
  static std::uint32_t load_word(const std::byte *p) noexcept {
    return std::to_integer<std::uint8_t>(p[0U]) |
           (static_cast<std::uint32_t>(std::to_integer<std::uint8_t>(p[1U]))
            << 8U) |
           (static_cast<std::uint32_t>(std::to_integer<std::uint8_t>(p[2U]))
            << 16U) |
           (static_cast<std::uint32_t>(std::to_integer<std::uint8_t>(p[3U]))
            << 24U);
  }
  Registers &registers_;
  poll_budget timeout_{};
};
} // namespace hal::stm32f4::nv
