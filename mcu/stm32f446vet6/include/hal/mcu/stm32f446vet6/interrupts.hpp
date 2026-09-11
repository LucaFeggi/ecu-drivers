#pragma once

#include <cstdint>

namespace hal::stm32f4::device::stm32f446vet6 {

// Only interrupt numbers consumed by reusable drivers are published here.
// Vector installation and priority remain BSP policy.
namespace interrupt_number {
inline constexpr std::int32_t dma2_stream0{56};
inline constexpr std::int32_t adc{18};
inline constexpr std::int32_t tim6_dac{54};
inline constexpr std::int32_t can1_rx0{20};
inline constexpr std::int32_t can2_rx0{64};
inline constexpr std::int32_t sdio{49};
}  // namespace interrupt_number
}  // namespace hal::stm32f4::device::stm32f446vet6
