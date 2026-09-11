#pragma once

#include <soc/system_struct.h>

namespace hal::esp32s3_wroom_1_n16r8::clock {

// Peripheral clocks and resets are board/startup policy. A driver never
// silently changes another peripheral's clock state.
enum class resource : std::uint8_t {
  uart0,
  uart1,
  uart2,
  i2c0,
  i2c1,
  spi2,
  spi3,
  spi2_dma,
  spi3_dma,
  gdma,
  twai,
  ledc,
  systimer,
  adc,
  sdmmc,
  timer_group0,
  timer_group1,
  spi_mem
};

class Gate {
 public:
  explicit Gate(system_dev_t& system) noexcept : system_{&system} {}

  Gate(const Gate&) = delete;
  Gate& operator=(const Gate&) = delete;

  void enable(resource selected) noexcept {
    switch (selected) {
      case resource::uart0: system_->perip_clk_en0.uart_clk_en = 1U; break;
      case resource::uart1: system_->perip_clk_en0.uart1_clk_en = 1U; break;
      case resource::uart2: system_->perip_clk_en1.uart2_clk_en = 1U; break;
      case resource::i2c0: system_->perip_clk_en0.i2c_ext0_clk_en = 1U; break;
      case resource::i2c1: system_->perip_clk_en0.i2c_ext1_clk_en = 1U; break;
      case resource::spi2: system_->perip_clk_en0.spi2_clk_en = 1U; break;
      case resource::spi3: system_->perip_clk_en0.spi3_clk_en = 1U; break;
      case resource::spi2_dma: system_->perip_clk_en0.spi2_dma_clk_en = 1U; break;
      case resource::spi3_dma: system_->perip_clk_en0.spi3_dma_clk_en = 1U; break;
      case resource::gdma: system_->perip_clk_en1.dma_clk_en = 1U; break;
      case resource::twai: system_->perip_clk_en0.can_clk_en = 1U; break;
      case resource::ledc: system_->perip_clk_en0.ledc_clk_en = 1U; break;
      case resource::systimer: system_->perip_clk_en0.systimer_clk_en = 1U; break;
      case resource::adc:
        system_->perip_clk_en0.apb_saradc_clk_en = 1U;
        system_->perip_clk_en0.adc2_arb_clk_en = 1U;
        break;
      case resource::sdmmc: system_->perip_clk_en1.sdio_host_clk_en = 1U; break;
      case resource::timer_group0:
        system_->perip_clk_en0.timers_clk_en = 1U;
        system_->perip_clk_en0.timergroup_clk_en = 1U;
        system_->perip_clk_en0.wdg_clk_en = 1U;
        break;
      case resource::timer_group1:
        system_->perip_clk_en0.timers_clk_en = 1U;
        system_->perip_clk_en0.timergroup1_clk_en = 1U;
        system_->perip_clk_en0.wdg_clk_en = 1U;
        break;
      case resource::spi_mem: system_->perip_clk_en0.spi01_clk_en = 1U; break;
    }
  }

  void reset(resource selected) noexcept {
    switch (selected) {
      case resource::uart0:
        system_->perip_rst_en0.uart_rst = 1U;
        system_->perip_rst_en0.uart_rst = 0U;
        break;
      case resource::uart1:
        system_->perip_rst_en0.uart1_rst = 1U;
        system_->perip_rst_en0.uart1_rst = 0U;
        break;
      case resource::uart2:
        system_->perip_rst_en1.uart2_rst = 1U;
        system_->perip_rst_en1.uart2_rst = 0U;
        break;
      case resource::i2c0:
        system_->perip_rst_en0.i2c_ext0_rst = 1U;
        system_->perip_rst_en0.i2c_ext0_rst = 0U;
        break;
      case resource::i2c1:
        system_->perip_rst_en0.i2c_ext1_rst = 1U;
        system_->perip_rst_en0.i2c_ext1_rst = 0U;
        break;
      case resource::spi2:
        system_->perip_rst_en0.spi2_rst = 1U;
        system_->perip_rst_en0.spi2_rst = 0U;
        break;
      case resource::spi3:
        system_->perip_rst_en0.spi3_rst = 1U;
        system_->perip_rst_en0.spi3_rst = 0U;
        break;
      case resource::spi2_dma:
        system_->perip_rst_en0.spi2_dma_rst = 1U;
        system_->perip_rst_en0.spi2_dma_rst = 0U;
        break;
      case resource::spi3_dma:
        system_->perip_rst_en0.spi3_dma_rst = 1U;
        system_->perip_rst_en0.spi3_dma_rst = 0U;
        break;
      case resource::gdma:
        system_->perip_rst_en1.dma_rst = 1U;
        system_->perip_rst_en1.dma_rst = 0U;
        break;
      case resource::twai:
        system_->perip_rst_en0.can_rst = 1U;
        system_->perip_rst_en0.can_rst = 0U;
        break;
      case resource::ledc:
        system_->perip_rst_en0.ledc_rst = 1U;
        system_->perip_rst_en0.ledc_rst = 0U;
        break;
      case resource::systimer:
        system_->perip_rst_en0.systimer_rst = 1U;
        system_->perip_rst_en0.systimer_rst = 0U;
        break;
      case resource::adc:
        system_->perip_rst_en0.apb_saradc_rst = 1U;
        system_->perip_rst_en0.apb_saradc_rst = 0U;
        system_->perip_rst_en0.adc2_arb_rst = 1U;
        system_->perip_rst_en0.adc2_arb_rst = 0U;
        break;
      case resource::sdmmc:
        system_->perip_rst_en1.sdio_host_rst = 1U;
        system_->perip_rst_en1.sdio_host_rst = 0U;
        break;
      case resource::timer_group0:
        system_->perip_rst_en0.timers_rst = 1U;
        system_->perip_rst_en0.timers_rst = 0U;
        system_->perip_rst_en0.timergroup_rst = 1U;
        system_->perip_rst_en0.timergroup_rst = 0U;
        break;
      case resource::timer_group1:
        system_->perip_rst_en0.timers_rst = 1U;
        system_->perip_rst_en0.timers_rst = 0U;
        system_->perip_rst_en0.timergroup1_rst = 1U;
        system_->perip_rst_en0.timergroup1_rst = 0U;
        break;
      case resource::spi_mem:
        system_->perip_rst_en0.spi01_rst = 1U;
        system_->perip_rst_en0.spi01_rst = 0U;
        break;
    }
  }

  void enable_and_reset(resource selected) noexcept {
    enable(selected);
    reset(selected);
  }

 private:
  system_dev_t* system_;
};

}  // namespace hal::esp32s3_wroom_1_n16r8::clock
