#pragma once

// This file is an exact-part register binding, not a board description. It
// intentionally contains no clock, pin, DMA-stream, interrupt, or memory
// placement choices. The BSP supplies those choices when constructing the
// reusable family templates.
#include <stm32h723xx.h>

namespace hal::stm32h7::device::stm32h723vgt6::binding {

using adc_registers = ADC_TypeDef;
using adc_common_registers = ADC_Common_TypeDef;
using dma_registers = DMA_TypeDef;
using dma_stream_registers = DMA_Stream_TypeDef;
using dmamux_channel_registers = DMAMUX_Channel_TypeDef;
using gpio_registers = GPIO_TypeDef;
using syscfg_registers = SYSCFG_TypeDef;
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

} // namespace hal::stm32h7::device::stm32h723vgt6::binding
