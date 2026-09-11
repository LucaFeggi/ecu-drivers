#pragma once

#include <cstddef>
#include <cstdint>
#include <hal/dma/descriptor.hpp>

namespace hal::esp32s3_wroom_1_n16r8::device {

using gdma_descriptor = hal::esp32s3::dma::descriptor;
inline constexpr std::uint32_t gdma_owner_dma = hal::esp32s3::dma::owner_dma;
inline constexpr std::uint32_t gdma_suc_eof =
    hal::esp32s3::dma::successful_eof;
inline constexpr std::uint32_t gdma_size_mask = hal::esp32s3::dma::size_mask;
inline constexpr std::uint32_t gdma_length_mask =
    hal::esp32s3::dma::length_mask;
inline constexpr std::uint32_t gdma_length_shift =
    hal::esp32s3::dma::length_shift;

[[nodiscard]] constexpr std::uint32_t descriptor_control(
    std::size_t size, std::size_t length, bool owner_dma,
    bool successful_eof) noexcept {
  return hal::esp32s3::dma::descriptor_control(size, length, owner_dma,
                                                successful_eof);
}

}  // namespace hal::esp32s3_wroom_1_n16r8::device
