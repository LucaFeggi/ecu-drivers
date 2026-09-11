#pragma once

#include <cstddef>
#include <cstdint>

namespace hal::esp32s3::dma {

// ESP32-S3 GDMA layout is a silicon-family fact. Exact module profiles decide
// which memory is DMA-visible and how much storage exists.
struct alignas(4) descriptor {
  std::uint32_t control{};
  std::uint32_t buffer_address{};
  std::uint32_t next_descriptor{};
};

static_assert(sizeof(descriptor) == 12U);

inline constexpr std::uint32_t owner_dma = 1U << 31U;
inline constexpr std::uint32_t successful_eof = 1U << 30U;
inline constexpr std::uint32_t size_mask = 0x00000FFFU;
inline constexpr std::uint32_t length_mask = 0x00FFF000U;
inline constexpr std::uint32_t length_shift = 12U;

[[nodiscard]] constexpr std::uint32_t descriptor_control(
    std::size_t size, std::size_t length, bool owned_by_dma,
    bool mark_successful_eof) noexcept {
  return (static_cast<std::uint32_t>(size) & size_mask) |
         ((static_cast<std::uint32_t>(length) << length_shift) & length_mask) |
         (owned_by_dma ? owner_dma : 0U) |
         (mark_successful_eof ? successful_eof : 0U);
}

} // namespace hal::esp32s3::dma
