#pragma once

#include <cstdint>

namespace hal::stm32g4::device::stm32g484ce {

// Only interrupt numbers consumed by reusable drivers are published here.
// Vector installation and priority remain BSP policy.
namespace interrupt_number {
inline constexpr std::int32_t dma1_channel1{11};
inline constexpr std::int32_t dma1_channel2{12};
inline constexpr std::int32_t dma1_channel3{13};
inline constexpr std::int32_t dma1_channel4{14};
inline constexpr std::int32_t dma1_channel5{15};
inline constexpr std::int32_t dma1_channel6{16};
inline constexpr std::int32_t dma1_channel7{17};
inline constexpr std::int32_t adc1_adc2{18};
inline constexpr std::int32_t fdcan1_it0{21};
inline constexpr std::int32_t fdcan1_it1{22};
inline constexpr std::int32_t i2c1_event{31};
inline constexpr std::int32_t i2c1_error{32};
inline constexpr std::int32_t spi1{35};
inline constexpr std::int32_t spi2{36};
inline constexpr std::int32_t usart1{37};
inline constexpr std::int32_t usart2{38};
inline constexpr std::int32_t usart3{39};
inline constexpr std::int32_t i2c2_event{40};
inline constexpr std::int32_t i2c2_error{41};
inline constexpr std::int32_t spi3{51};
inline constexpr std::int32_t uart4{52};
inline constexpr std::int32_t uart5{53};
inline constexpr std::int32_t tim6_dac{54};
inline constexpr std::int32_t tim7_dac{55};
inline constexpr std::int32_t dma2_channel1{56};
inline constexpr std::int32_t dma2_channel2{57};
inline constexpr std::int32_t dma2_channel3{58};
inline constexpr std::int32_t dma2_channel4{59};
inline constexpr std::int32_t dma2_channel5{60};
inline constexpr std::int32_t adc4{61};
inline constexpr std::int32_t adc5{62};
inline constexpr std::int32_t i2c4_event{82};
inline constexpr std::int32_t i2c4_error{83};
inline constexpr std::int32_t spi4{84};
inline constexpr std::int32_t fdcan2_it0{86};
inline constexpr std::int32_t fdcan2_it1{87};
inline constexpr std::int32_t fdcan3_it0{88};
inline constexpr std::int32_t fdcan3_it1{89};
inline constexpr std::int32_t lpuart1{91};
inline constexpr std::int32_t i2c3_event{92};
inline constexpr std::int32_t i2c3_error{93};
inline constexpr std::int32_t dmamux_overrun{94};
inline constexpr std::int32_t quadspi{95};
inline constexpr std::int32_t dma1_channel8{96};
inline constexpr std::int32_t dma2_channel6{97};
inline constexpr std::int32_t dma2_channel7{98};
inline constexpr std::int32_t dma2_channel8{99};
}  // namespace interrupt_number

}  // namespace hal::stm32g4::device::stm32g484ce
