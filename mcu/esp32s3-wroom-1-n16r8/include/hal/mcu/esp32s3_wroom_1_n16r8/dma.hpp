#pragma once

#include <cstddef>
#include <cstdint>

namespace hal::esp32s3_wroom_1_n16r8::device {

// GDMA descriptors are intentionally kept in the target package. Their
// address fields are 32-bit on ESP32-S3 even when the host-side test process
// uses wider pointers; target code must populate them with DMA-visible
// addresses supplied by the BSP.
struct alignas(4) gdma_descriptor {
  // DW0: size[11:0], length[23:12], err_eof[28], suc_eof[30], owner[31].
  std::uint32_t control{};
  std::uint32_t buffer_address{};
  std::uint32_t next_descriptor{};
};

static_assert(sizeof(gdma_descriptor) == 12U);

inline constexpr std::uint32_t gdma_owner_dma = 1U << 31U;
inline constexpr std::uint32_t gdma_suc_eof = 1U << 30U;
inline constexpr std::uint32_t gdma_size_mask = 0x00000FFFU;
inline constexpr std::uint32_t gdma_length_mask = 0x00FFF000U;
inline constexpr std::uint32_t gdma_length_shift = 12U;

[[nodiscard]] constexpr std::uint32_t descriptor_control(
    std::size_t size, std::size_t length, bool owner_dma,
    bool successful_eof) noexcept {
  return (static_cast<std::uint32_t>(size) & gdma_size_mask) |
         ((static_cast<std::uint32_t>(length) << gdma_length_shift) &
          gdma_length_mask) |
         (owner_dma ? gdma_owner_dma : 0U) |
         (successful_eof ? gdma_suc_eof : 0U);
}

}  // namespace hal::esp32s3_wroom_1_n16r8::device
