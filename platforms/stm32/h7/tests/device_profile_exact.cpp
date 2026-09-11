#include <array>
#include <cstddef>
#include <cstdint>

#include <hal/adc.hpp>
#include <hal/block.hpp>
#include <hal/can.hpp>
#include <hal/ethernet.hpp>
#include <hal/gpio.hpp>
#include <hal/i2c.hpp>
#include <hal/nv.hpp>
#include <hal/pwm.hpp>
#include <hal/rtc.hpp>
#include <hal/serial.hpp>
#include <hal/spi.hpp>
#include <hal/time.hpp>
#include <hal/watchdog.hpp>
#include <hal/adc.hpp>
#include <hal/block.hpp>
#include <hal/can.hpp>
#include <hal/ethernet.hpp>
#include <hal/gpio.hpp>
#include <hal/i2c.hpp>
#include <hal/nv.hpp>
#include <hal/pwm.hpp>
#include <hal/rtc.hpp>
#include <hal/serial.hpp>
#include <hal/spi.hpp>
#include <hal/time.hpp>
#include <hal/watchdog.hpp>
#include <hal/adc.hpp>
#include <hal/block.hpp>
#include <hal/can.hpp>
#include <hal/ethernet.hpp>
#include <hal/gpio.hpp>
#include <hal/i2c.hpp>
#include <hal/nv.hpp>
#include <hal/pwm.hpp>
#include <hal/rtc.hpp>
#include <hal/serial.hpp>
#include <hal/spi.hpp>
#include <hal/time.hpp>
#include <hal/watchdog.hpp>
#include <hal/mcu/stm32h723vgt6/bindings.hpp>
#include <hal/adc.hpp>
#include <hal/block.hpp>
#include <hal/can.hpp>
#include <hal/ethernet.hpp>
#include <hal/gpio.hpp>
#include <hal/i2c.hpp>
#include <hal/nv.hpp>
#include <hal/pwm.hpp>
#include <hal/rtc.hpp>
#include <hal/serial.hpp>
#include <hal/spi.hpp>
#include <hal/time.hpp>
#include <hal/watchdog.hpp>
#include <hal/adc.hpp>
#include <hal/block.hpp>
#include <hal/can.hpp>
#include <hal/ethernet.hpp>
#include <hal/gpio.hpp>
#include <hal/i2c.hpp>
#include <hal/nv.hpp>
#include <hal/pwm.hpp>
#include <hal/rtc.hpp>
#include <hal/serial.hpp>
#include <hal/spi.hpp>
#include <hal/time.hpp>
#include <hal/watchdog.hpp>
#include <hal/adc.hpp>
#include <hal/block.hpp>
#include <hal/can.hpp>
#include <hal/ethernet.hpp>
#include <hal/gpio.hpp>
#include <hal/i2c.hpp>
#include <hal/nv.hpp>
#include <hal/pwm.hpp>
#include <hal/rtc.hpp>
#include <hal/serial.hpp>
#include <hal/spi.hpp>
#include <hal/time.hpp>
#include <hal/watchdog.hpp>
#include <hal/adc.hpp>
#include <hal/block.hpp>
#include <hal/can.hpp>
#include <hal/ethernet.hpp>
#include <hal/gpio.hpp>
#include <hal/i2c.hpp>
#include <hal/nv.hpp>
#include <hal/pwm.hpp>
#include <hal/rtc.hpp>
#include <hal/serial.hpp>
#include <hal/spi.hpp>
#include <hal/time.hpp>
#include <hal/watchdog.hpp>
#include <hal/adc.hpp>
#include <hal/block.hpp>
#include <hal/can.hpp>
#include <hal/ethernet.hpp>
#include <hal/gpio.hpp>
#include <hal/i2c.hpp>
#include <hal/nv.hpp>
#include <hal/pwm.hpp>
#include <hal/rtc.hpp>
#include <hal/serial.hpp>
#include <hal/spi.hpp>
#include <hal/time.hpp>
#include <hal/watchdog.hpp>
#include <hal/adc.hpp>
#include <hal/block.hpp>
#include <hal/can.hpp>
#include <hal/ethernet.hpp>
#include <hal/gpio.hpp>
#include <hal/i2c.hpp>
#include <hal/nv.hpp>
#include <hal/pwm.hpp>
#include <hal/rtc.hpp>
#include <hal/serial.hpp>
#include <hal/spi.hpp>
#include <hal/time.hpp>
#include <hal/watchdog.hpp>
#include <hal/adc.hpp>
#include <hal/block.hpp>
#include <hal/can.hpp>
#include <hal/ethernet.hpp>
#include <hal/gpio.hpp>
#include <hal/i2c.hpp>
#include <hal/nv.hpp>
#include <hal/pwm.hpp>
#include <hal/rtc.hpp>
#include <hal/serial.hpp>
#include <hal/spi.hpp>
#include <hal/time.hpp>
#include <hal/watchdog.hpp>
#include <hal/adc.hpp>
#include <hal/block.hpp>
#include <hal/can.hpp>
#include <hal/ethernet.hpp>
#include <hal/gpio.hpp>
#include <hal/i2c.hpp>
#include <hal/nv.hpp>
#include <hal/pwm.hpp>
#include <hal/rtc.hpp>
#include <hal/serial.hpp>
#include <hal/spi.hpp>
#include <hal/time.hpp>
#include <hal/watchdog.hpp>
#include <hal/adc.hpp>
#include <hal/block.hpp>
#include <hal/can.hpp>
#include <hal/ethernet.hpp>
#include <hal/gpio.hpp>
#include <hal/i2c.hpp>
#include <hal/nv.hpp>
#include <hal/pwm.hpp>
#include <hal/rtc.hpp>
#include <hal/serial.hpp>
#include <hal/spi.hpp>
#include <hal/time.hpp>
#include <hal/watchdog.hpp>
#include <hal/adc.hpp>
#include <hal/block.hpp>
#include <hal/can.hpp>
#include <hal/ethernet.hpp>
#include <hal/gpio.hpp>
#include <hal/i2c.hpp>
#include <hal/nv.hpp>
#include <hal/pwm.hpp>
#include <hal/rtc.hpp>
#include <hal/serial.hpp>
#include <hal/spi.hpp>
#include <hal/time.hpp>
#include <hal/watchdog.hpp>

namespace {

namespace binding = hal::stm32h7::device::stm32h723vgt6::binding;

constexpr hal::adc::characteristics adc_characteristics{
    18U, hal::microvolts{0}, hal::microvolts{3'300'000}};

struct delay_stub {
  void operator()(std::uint32_t) noexcept {}
};

[[maybe_unused]] void instantiate_exact_h723_templates() {
  alignas(32U) volatile std::uint32_t adc_buffer[8U]{};
  constexpr hal::stm32h7::adc::dma_config adc_config{
      64'000'000U,
      hal::stm32h7::adc::asynchronous_prescaler::divide_by_2,
      4U,
      3U,
      16U,
      2U,
      13U,
      9U,
      63U,
      199U,
      200'000U,
      hal::stm32h7::poll_budget{100U}};
  using adc_driver = hal::stm32h7::adc::Adc12DmaStream0<
      adc_characteristics, 16U, 8U, binding::adc_registers,
      binding::adc_common_registers, binding::dma_registers,
      binding::dma_stream_registers, binding::dmamux_channel_registers,
      binding::timer_registers>;
  adc_driver adc{*ADC1, *ADC12_COMMON, *DMA1, *DMA1_Stream0,
                 *DMAMUX1_Channel0, *TIM6, adc_buffer, adc_config};
  delay_stub delay{};
  (void)adc.initialize(delay);
  (void)adc.start(hal::instant{});
  (void)adc.stop();
  adc.on_dma_interrupt();

  using dma_serial = hal::stm32h7::serial::DmaPort<
      binding::serial_registers, binding::dma_stream_registers,
      binding::dma_stream_registers, binding::dmamux_channel_registers,
      binding::dmamux_channel_registers>;
  alignas(32U) std::array<std::byte, 1024U> serial_tx{};
  alignas(32U) std::array<std::byte, 2048U> serial_rx{};
  dma_serial serial{*USART1, *DMA1_Stream1, *DMA1_Stream2,
                    *DMAMUX1_Channel1, *DMAMUX1_Channel2, 64'000'000U, 42U,
                    41U, serial_tx, serial_rx};
  (void)serial.configure({hal::hertz{115'200U}});
  (void)serial.try_write({serial_tx.data(), 1U});
  (void)serial.try_read({serial_rx.data(), 1U});
  serial.on_tx_dma_interrupt();
  serial.on_rx_dma_interrupt();

  using dma_spi = hal::stm32h7::spi::DmaBus8<
      binding::spi_registers, binding::dma_stream_registers,
      binding::dma_stream_registers, binding::dmamux_channel_registers,
      binding::dmamux_channel_registers>;
  alignas(32U) std::array<std::byte, 4096U> spi_tx{};
  alignas(32U) std::array<std::byte, 4096U> spi_rx{};
  dma_spi spi{*SPI1, *DMA1_Stream3, *DMA1_Stream4, *DMAMUX1_Channel3,
              *DMAMUX1_Channel4, 64'000'000U,
              hal::stm32h7::poll_budget{100U}, 38U, 37U, spi_tx, spi_rx};
  (void)spi.configure({hal::hertz{1'000'000U}});
  (void)spi.transfer({spi_tx.data(), 1U}, {spi_rx.data(), 1U});

  using dma_i2c = hal::stm32h7::i2c::Controller<
      binding::i2c_registers, 1U, binding::dma_stream_registers,
      binding::dmamux_channel_registers>;
  dma_i2c i2c{*I2C1, *DMA1_Stream5, *DMAMUX1_Channel5,
              {1U, hal::stm32h7::poll_budget{100U}, 0U, true}};
  (void)i2c.initialize();

  alignas(32U) hal::stm32h7::ethernet::descriptor tx_descriptors[2U]{};
  alignas(32U) hal::stm32h7::ethernet::descriptor rx_descriptors[2U]{};
  alignas(32U) std::array<std::byte, 2U * 1536U> tx_buffers{};
  alignas(32U) std::array<std::byte, 2U * 1536U> rx_buffers{};
  using ethernet_mac = hal::stm32h7::ethernet::Mac<
      binding::ethernet_registers, 2U, 2U>;
  ethernet_mac mac{*ETH, tx_descriptors, rx_descriptors, tx_buffers,
                   rx_buffers, {1536U, 8U, 8U,
                                 hal::stm32h7::poll_budget{100U}}};
  (void)mac.initialize();
  (void)mac.try_transmit({tx_buffers.data(), 1U});
  (void)mac.try_receive({rx_buffers.data(), 1U});

  using can_driver = hal::stm32h7::can::Controller<binding::fdcan_registers>;
  alignas(4U) volatile std::uint32_t message_ram[512U]{};
  can_driver can{*FDCAN1, message_ram, 512U,
                 {1U, 1U, 1U, 0U, 1U, 1U, 8U, 8U,
                  hal::stm32h7::poll_budget{100U}, true, true}};
  (void)can.initialize();

  using block_driver = hal::stm32h7::block::SdmmcDevice<binding::sdmmc_registers>;
  block_driver block{*SDMMC1,
                     {0U, 0U, 1U, hal::stm32h7::poll_budget{100U}, true, true,
                      false}};
  (void)block.initialize();

  using flash_driver = hal::stm32h7::nv::NorFlash<
      binding::flash_registers, 0x08000000U, 1024U * 1024U>;
  flash_driver flash{*FLASH, hal::stm32h7::poll_budget{100U}};
  (void)flash.sync();

  hal::stm32h7::gpio::OutputPin gpio_output{*GPIOA, 0U};
  (void)gpio_output.write(hal::gpio::level::low);
  hal::stm32h7::gpio::EdgeInput edge_input{*GPIOA, *SYSCFG, *EXTI, 0U, 0U};
  (void)edge_input.configure_edge(hal::gpio::edge::rising);
  (void)edge_input.take_event();

  using pwm_driver = hal::stm32h7::pwm::Output<binding::timer_registers,
                                                64'000'000U, 1U>;
  pwm_driver pwm{*TIM2};
  (void)pwm.configure({hal::nanoseconds{1'000U}, 1'000U,
                       hal::pwm::polarity::active_high});
  hal::stm32h7::rtc::Clock rtc{*RTC};
  (void)rtc.alarm_pending();
  hal::stm32h7::watchdog::Feeder<binding::watchdog_registers, 32'000U> watchdog{
      *IWDG1};
  (void)watchdog.feed();
}

} // namespace
