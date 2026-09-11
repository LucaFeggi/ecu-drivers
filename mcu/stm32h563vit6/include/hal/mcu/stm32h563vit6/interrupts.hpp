#pragma once

#include <cstdint>

namespace hal::stm32h5::device::stm32h563vit6 {

// Only interrupt numbers consumed by reusable drivers are published here.
// Vector installation and priority remain BSP policy.
namespace interrupt_number {
inline constexpr std::int32_t gpdma1_channel0{27};
inline constexpr std::int32_t adc1_2{17};
inline constexpr std::int32_t tim6{54};
inline constexpr std::int32_t fdcan1_it0{39};
inline constexpr std::int32_t fdcan2_it0{109};
inline constexpr std::int32_t eth{106};
}  // namespace interrupt_number
}  // namespace hal::stm32h5::device::stm32h563vit6
