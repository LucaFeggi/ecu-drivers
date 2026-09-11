#include <array>
#include <hal/mcu/stm32h563vit6/bindings.hpp>
#include <hal/mcu/stm32h563vit6/capabilities.hpp>
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
namespace binding = hal::stm32h5::device::stm32h563vit6::binding;
constexpr hal::adc::characteristics characteristics{12U, hal::microvolts{0},
                                                    hal::microvolts{3'300'000}};
static_assert(hal::adc::ContinuousChannel<
              hal::stm32h5::adc::ContinuousChannel<characteristics, 8U>>);
static_assert(hal::serial::ConfigurablePort<
              hal::stm32h5::serial::PollingPort<binding::serial_registers>>);
static_assert(
    hal::spi::Bus8<hal::stm32h5::spi::PollingBus8<binding::spi_registers>>);
static_assert(hal::i2c::Controller7<
              hal::stm32h5::i2c::Controller<binding::i2c_registers>>);
static_assert(hal::can::FdController<
              hal::stm32h5::can::Controller<binding::fdcan_registers>>);
static_assert(hal::block::Device<
              hal::stm32h5::block::SdmmcDevice<binding::sdmmc_registers>>);
static_assert(hal::nv::NorFlash<hal::stm32h5::nv::NorFlash<
                  binding::flash_registers, 0x08000000U, 2U * 1024U * 1024U>>);
static_assert(hal::gpio::EdgeInput<hal::stm32h5::gpio::EdgeInput<
                  binding::gpio_registers, binding::exti_registers>>);

[[maybe_unused]] void instantiate() {
  volatile std::uint32_t adc_buffer[8U]{};
  binding::adc_registers adc{};
  binding::adc_common_registers common{};
  binding::dma_channel_registers dma{};
  hal::stm32h5::adc::Adc12Gpdma<characteristics, 8U, binding::adc_registers,
                                binding::adc_common_registers,
                                binding::dma_channel_registers>
      adc_driver{
          adc,
          common,
          dma,
          adc_buffer,
          {64'000'000U, 4U, 3U, 13U, 9U, hal::stm32h5::poll_budget{10U}}};
  (void)adc_driver.initialize();

  binding::serial_registers serial{};
  std::array<std::byte, 32U> tx{};
  std::array<std::byte, 64U> rx{};
  hal::stm32h5::serial::DmaPort<binding::serial_registers,
                                binding::dma_channel_registers,
                                binding::dma_channel_registers, 32U, 64U>
      serial_driver{serial, dma, dma, 64'000'000U, 1U, 2U, tx, rx};
  (void)serial_driver.configure({hal::hertz{115'200U}});

  binding::spi_registers spi{};
  std::array<std::byte, 32U> spi_tx{};
  std::array<std::byte, 32U> spi_rx{};
  hal::stm32h5::spi::DmaBus8<binding::spi_registers,
                             binding::dma_channel_registers,
                             binding::dma_channel_registers, 32U>
      spi_driver{spi, dma, dma,    64'000'000U, hal::stm32h5::poll_budget{10U},
                 3U,  4U,  spi_tx, spi_rx};
  (void)spi_driver.configure({hal::hertz{1'000'000U}});

  binding::i2c_registers i2c{};
  hal::stm32h5::i2c::Controller<binding::i2c_registers, 1U,
                                binding::dma_channel_registers>
      i2c_driver{i2c,
                 dma,
                 {0x0000'2052U, hal::stm32h5::poll_budget{10U}, 5U, true, 6U}};
  (void)i2c_driver.initialize();

  binding::fdcan_registers can{};
  volatile std::uint32_t message_ram[256U]{};
  hal::stm32h5::can::Controller<binding::fdcan_registers> can_driver{
      can,
      message_ram,
      256U,
      {1U, 1U, 1U, 0U, 1U, 1U, 8U, 8U, hal::stm32h5::poll_budget{10U}, true,
       true}};
  (void)can_driver.initialize();

  binding::sdmmc_registers sdmmc{};
  hal::stm32h5::block::SdmmcDevice<binding::sdmmc_registers> block{
      sdmmc, {0U, 0U, 1U, hal::stm32h5::poll_budget{10U}, true, true, false}};
  (void)block.initialize();

  binding::flash_registers flash{};
  hal::stm32h5::nv::NorFlash<binding::flash_registers, 0x08000000U,
                             2U * 1024U * 1024U>
      flash_driver{flash, hal::stm32h5::poll_budget{10U}};
  (void)flash_driver.sync();

  binding::gpio_registers gpio{};
  binding::exti_registers exti{};
  hal::stm32h5::gpio::OutputPin output{gpio, 1U};
  (void)output.write(hal::gpio::level::low);
  hal::stm32h5::gpio::EdgeInput edge{gpio, exti, 1U, 0U};
  (void)edge.configure_edge(hal::gpio::edge::rising);
}

static_assert(
    hal::stm32h5::device::stm32h563vit6::capabilities::gpdma_instances == 2U);
} // namespace

int main() {
  instantiate();
  return 0;
}
