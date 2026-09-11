#pragma once

#include <stm32h563xx.h>

// This file maps reusable driver register roles to CMSIS register types. It
// contains no board-level routing or initialization policy.
namespace hal::stm32h5::device::stm32h563vit6::binding {

using adc_registers = ADC_TypeDef;
using adc_common_registers = ADC_Common_TypeDef;
using dma_registers = DMA_TypeDef;
using dma_channel_registers = DMA_Channel_TypeDef;
using gpio_registers = GPIO_TypeDef;
using exti_registers = EXTI_TypeDef;
using timer_registers = TIM_TypeDef;
using serial_registers = USART_TypeDef;
using spi_registers = SPI_TypeDef;
using i2c_registers = I2C_TypeDef;
using fdcan_registers = FDCAN_GlobalTypeDef;
using sdmmc_registers = SDMMC_TypeDef;
using flash_registers = FLASH_TypeDef;
using rtc_registers = RTC_TypeDef;
using watchdog_registers = IWDG_TypeDef;
using ethernet_registers = ETH_TypeDef;

}  // namespace hal::stm32h5::device::stm32h563vit6::binding
