#pragma once

#include <cstdint>
#include <soc/apb_saradc_struct.h>
#include <soc/gdma_struct.h>
#include <soc/gpio_struct.h>
#include <soc/i2c_struct.h>
#include <soc/ledc_struct.h>
#include <soc/reg_base.h>
#include <soc/rtc_cntl_struct.h>
#include <soc/sdmmc_struct.h>
#include <soc/sens_struct.h>
#include <soc/soc.h>
#include <soc/system_struct.h>
#include <soc/spi_mem_struct.h>
#include <soc/spi_struct.h>
#include <soc/systimer_struct.h>
#include <soc/twai_struct.h>
#include <soc/timer_group_struct.h>
#include <soc/uart_struct.h>
#include <soc/uhci_struct.h>

namespace hal::esp32s3_wroom_1_n16r8::device {

template <class T>
[[nodiscard]] inline T& at(std::uintptr_t address) noexcept {
  return *reinterpret_cast<T*>(address);
}

[[nodiscard]] inline gpio_dev_t& gpio() noexcept {
  return at<gpio_dev_t>(DR_REG_GPIO_BASE);
}
[[nodiscard]] inline i2c_dev_t& i2c0() noexcept {
  return at<i2c_dev_t>(DR_REG_I2C_EXT_BASE);
}
[[nodiscard]] inline i2c_dev_t& i2c1() noexcept {
  return at<i2c_dev_t>(DR_REG_I2C1_EXT_BASE);
}
[[nodiscard]] inline spi_dev_t& spi2() noexcept {
  return at<spi_dev_t>(DR_REG_SPI2_BASE);
}
[[nodiscard]] inline spi_dev_t& spi3() noexcept {
  return at<spi_dev_t>(DR_REG_SPI3_BASE);
}
[[nodiscard]] inline uart_dev_t& uart0() noexcept {
  return at<uart_dev_t>(REG_UART_BASE(0));
}
[[nodiscard]] inline uart_dev_t& uart1() noexcept {
  return at<uart_dev_t>(REG_UART_BASE(1));
}
[[nodiscard]] inline uart_dev_t& uart2() noexcept {
  return at<uart_dev_t>(REG_UART_BASE(2));
}
[[nodiscard]] inline uhci_dev_t& uhci0() noexcept {
  return at<uhci_dev_t>(DR_REG_UHCI0_BASE);
}
[[nodiscard]] inline twai_dev_t& twai() noexcept {
  return at<twai_dev_t>(DR_REG_TWAI_BASE);
}
[[nodiscard]] inline ledc_dev_t& ledc() noexcept {
  return at<ledc_dev_t>(DR_REG_LEDC_BASE);
}
[[nodiscard]] inline systimer_dev_t& systimer() noexcept {
  return at<systimer_dev_t>(DR_REG_SYSTIMER_BASE);
}
[[nodiscard]] inline rtc_cntl_dev_t& rtc() noexcept {
  return at<rtc_cntl_dev_t>(DR_REG_RTCCNTL_BASE);
}
[[nodiscard]] inline timg_dev_t& timer_group0() noexcept {
  return at<timg_dev_t>(DR_REG_TIMERGROUP0_BASE);
}
[[nodiscard]] inline timg_dev_t& timer_group1() noexcept {
  return at<timg_dev_t>(DR_REG_TIMERGROUP1_BASE);
}
[[nodiscard]] inline gdma_dev_t& gdma() noexcept {
  return at<gdma_dev_t>(DR_REG_GDMA_BASE);
}
[[nodiscard]] inline sdmmc_dev_t& sdmmc() noexcept {
  return at<sdmmc_dev_t>(DR_REG_SDMMC_BASE);
}
[[nodiscard]] inline apb_saradc_dev_t& adc() noexcept {
  return at<apb_saradc_dev_t>(DR_REG_APB_SARADC_BASE);
}
[[nodiscard]] inline sens_dev_t& sens() noexcept {
  return at<sens_dev_t>(DR_REG_SENS_BASE);
}
[[nodiscard]] inline spi_mem_dev_t& flash() noexcept {
  return at<spi_mem_dev_t>(DR_REG_SPI1_BASE);
}
[[nodiscard]] inline system_dev_t& system() noexcept {
  return at<system_dev_t>(DR_REG_SYSTEM_BASE);
}

}  // namespace hal::esp32s3_wroom_1_n16r8::device
