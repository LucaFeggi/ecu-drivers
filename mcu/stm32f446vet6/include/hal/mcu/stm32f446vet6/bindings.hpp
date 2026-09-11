#pragma once

#include <stm32f446xx.h>

namespace hal::stm32f4::device::stm32f446vet6::binding {

using adc_registers = ADC_TypeDef;
using adc_common_registers = ADC_Common_TypeDef;
using dma_registers = DMA_TypeDef;
using dma_stream_registers = DMA_Stream_TypeDef;
using gpio_registers = GPIO_TypeDef;
using syscfg_registers = SYSCFG_TypeDef;
using exti_registers = EXTI_TypeDef;
using timer_registers = TIM_TypeDef;
using serial_registers = USART_TypeDef;
using spi_registers = SPI_TypeDef;
using i2c_registers = I2C_TypeDef;
using can_registers = CAN_TypeDef;
using sdio_registers = SDIO_TypeDef;
using flash_registers = FLASH_TypeDef;
using rtc_registers = RTC_TypeDef;
using watchdog_registers = IWDG_TypeDef;

} // namespace hal::stm32f4::device::stm32f446vet6::binding
