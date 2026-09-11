#include <array>
#include <cstddef>
#include <cstdint>

#include <hal/mcu/stm32g484ce/bindings.hpp>
#include <hal/adc.hpp>
#include <hal/can.hpp>
#include <hal/gpio.hpp>
#include <hal/i2c.hpp>
#include <hal/nv.hpp>
#include <hal/pwm.hpp>
#include <hal/rtc.hpp>
#include <hal/serial.hpp>
#include <hal/spi.hpp>
#include <hal/time.hpp>
#include <hal/watchdog.hpp>

namespace binding = hal::stm32g4::device::stm32g484ce::binding;

constexpr hal::adc::characteristics adc_characteristics{
    12U, hal::microvolts{0}, hal::microvolts{3'300'000}};

struct delay_stub {
  void operator()(std::uint32_t) noexcept {}
};

[[maybe_unused]] void instantiate_exact_g484_templates() {
  alignas(4U) volatile std::uint16_t adc_buffer[16U]{};
  constexpr hal::stm32g4::adc::dma_config adc_config{
      170'000'000U,
      hal::stm32g4::adc::asynchronous_prescaler::divide_by_4,
      1U,
      3U,
      1U,
      0U,
      1U,
      5U,
      169U,
      999U,
      200'000U,
      hal::stm32g4::poll_budget{100U},
      12U};
  using adc_driver = hal::stm32g4::adc::AdcDma<
      adc_characteristics, 8U, 16U, binding::adc_registers,
      binding::adc_common_registers, binding::dma_registers,
      binding::dma_channel_registers, binding::dmamux_channel_registers,
      binding::timer_registers>;
  adc_driver adc{*ADC1, *ADC12_COMMON, *DMA1, *DMA1_Channel1,
                 *DMAMUX1_Channel0, *TIM6, adc_buffer, 1U, adc_config};
  delay_stub delay{};
  (void)adc.initialize(delay);
  (void)adc.start(hal::instant{});
  (void)adc.stop();
  adc.on_dma_interrupt();

  using dma_serial = hal::stm32g4::serial::DmaPort<
      binding::serial_registers, binding::dma_channel_registers,
      binding::dma_channel_registers, binding::dmamux_channel_registers,
      binding::dmamux_channel_registers>;
  std::array<std::byte, 1024U> serial_tx{};
  std::array<std::byte, 2048U> serial_rx{};
  dma_serial serial{*USART1, *DMA1_Channel2, *DMA1_Channel3,
                    *DMAMUX1_Channel1, *DMAMUX1_Channel2, 170'000'000U,
                    25U, 24U, serial_tx, serial_rx};
  (void)serial.configure({hal::hertz{115'200U}});
  (void)serial.try_write({serial_tx.data(), 1U});
  (void)serial.try_read({serial_rx.data(), 1U});
  serial.on_tx_dma_interrupt();
  serial.on_rx_dma_interrupt();

  using dma_spi = hal::stm32g4::spi::DmaBus8<
      binding::spi_registers, binding::dma_channel_registers,
      binding::dma_channel_registers, binding::dmamux_channel_registers,
      binding::dmamux_channel_registers>;
  std::array<std::byte, 4096U> spi_tx{};
  std::array<std::byte, 4096U> spi_rx{};
  dma_spi spi{*SPI1, *DMA1_Channel4, *DMA1_Channel5, *DMAMUX1_Channel3,
              *DMAMUX1_Channel4, 170'000'000U,
              hal::stm32g4::poll_budget{100U}, 11U, 10U, spi_tx, spi_rx};
  (void)spi.configure({hal::hertz{1'000'000U}});
  (void)spi.transfer({spi_tx.data(), 1U}, {spi_rx.data(), 1U});

  using dma_i2c = hal::stm32g4::i2c::Controller<
      binding::i2c_registers, 1U, binding::dma_channel_registers,
      binding::dma_channel_registers, binding::dmamux_channel_registers,
      binding::dmamux_channel_registers>;
  dma_i2c i2c{*I2C1, *DMA1_Channel6, *DMA1_Channel7, *DMAMUX1_Channel5,
              *DMAMUX1_Channel6,
              {1U, hal::stm32g4::poll_budget{100U}, 17U, true, 16U}};
  (void)i2c.initialize();
  (void)i2c.recover_bus();
  std::array<std::byte, 1U> i2c_data{};
  (void)hal::i2c::write(i2c, hal::i2c::address7{0x10U},
                        {i2c_data.data(), i2c_data.size()});

  using can_controller = hal::stm32g4::can::Controller<binding::fdcan_registers>;
  can_controller can{*FDCAN1, reinterpret_cast<volatile std::uint32_t *>(0x4000A400U),
                     212U, {1U, 1U, 0U, 0U, 1U, 1U, 3U, 3U,
                            hal::stm32g4::poll_budget{100U}, true, true}};
  (void)can.initialize();
  std::array<std::byte, 8U> can_data{};
  const auto classic = hal::can::classic_frame::make(
      {0x123U, false}, {can_data.data(), can_data.size()});
  if (classic) {
    (void)can.try_send(classic.value());
  }
  (void)can.try_receive();

  using flash = hal::stm32g4::nv::NorFlash<binding::flash_registers>;
  flash internal_flash{*FLASH};
  (void)internal_flash.sync();
  (void)internal_flash.read(0U, {can_data.data(), can_data.size()});
  (void)internal_flash.program(0U, {can_data.data(), can_data.size()});
  (void)internal_flash.erase(0U, 2U * 1024U);

  hal::stm32g4::gpio::OutputPin gpio_output{*GPIOA, 0U};
  (void)gpio_output.write(hal::gpio::level::low);
  hal::stm32g4::gpio::EdgeInput edge_input{*GPIOA, *SYSCFG, *EXTI, 0U, 0U};
  (void)edge_input.configure_edge(hal::gpio::edge::rising);
  (void)edge_input.take_event();

  using pwm_driver = hal::stm32g4::pwm::Output<binding::timer_registers,
                                                170'000'000U, 1U>;
  pwm_driver pwm{*TIM2};
  (void)pwm.configure({hal::nanoseconds{1'000U}, 1'000U,
                       hal::pwm::polarity::active_high});
  (void)pwm.set_pulse_width(hal::nanoseconds{500U});
  (void)pwm.set_dead_time(hal::nanoseconds{100U});

  using timer_clock = hal::stm32g4::time::TimerClock<170'000'000U, 1'000'000U,
                                                      binding::timer_registers>;
  timer_clock clock{*TIM2};
  clock.initialize();
  hal::stm32g4::rtc::Clock rtc{*RTC};
  (void)rtc.alarm_pending();
  (void)rtc.set_alarm(hal::utc_time{0, 0U});
  hal::stm32g4::watchdog::Feeder<binding::watchdog_registers, 32'000U> watchdog{
      *IWDG};
  (void)watchdog.start();
  (void)watchdog.feed();
}

int main() { return 0; }
