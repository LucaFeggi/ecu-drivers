#include <array>
#include <hal/mcu/stm32f446vet6/bindings.hpp>
#include <hal/mcu/stm32f446vet6/device.hpp>
#include <hal/adc.hpp>
#include <hal/block.hpp>
#include <hal/can.hpp>
#include <hal/contracts/ethernet.hpp>
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
namespace binding = hal::stm32f4::device::stm32f446vet6::binding;
constexpr hal::adc::characteristics characteristics{12U, hal::microvolts{0},
                                                    hal::microvolts{3'300'000}};
static_assert(hal::adc::ContinuousChannel<
              hal::stm32f4::adc::ContinuousChannel<characteristics, 8U>>);
static_assert(hal::serial::ConfigurablePort<
              hal::stm32f4::serial::PollingPort<binding::serial_registers>>);
static_assert(
    hal::spi::Bus8<hal::stm32f4::spi::PollingBus8<binding::spi_registers>>);
static_assert(hal::i2c::Controller7<
              hal::stm32f4::i2c::Controller<binding::i2c_registers>>);
static_assert(hal::can::ClassicController<
              hal::stm32f4::can::Controller<binding::can_registers>>);
static_assert(hal::block::Device<
              hal::stm32f4::block::SdmmcDevice<binding::sdio_registers>>);
static_assert(hal::nv::NorFlash<hal::stm32f4::nv::NorFlash<
                  binding::flash_registers, 0x08000000U, 512U * 1024U>>);
static_assert(hal::gpio::EdgeInput<hal::stm32f4::gpio::EdgeInput<
                  binding::gpio_registers, binding::syscfg_registers,
                  binding::exti_registers>>);

[[maybe_unused]] void instantiate() {
  volatile std::uint16_t adc_buffer[8U]{};
  binding::adc_registers adc{};
  binding::adc_common_registers common{};
  binding::dma_stream_registers stream{};
  hal::stm32f4::adc::AdcDma<characteristics, 8U, binding::adc_registers,
                            binding::adc_common_registers,
                            binding::dma_stream_registers>
      adc_driver{adc,
                 common,
                 stream,
                 adc_buffer,
                 {36'000'000U, 4U, 3U, 0U, hal::stm32f4::poll_budget{10U}}};
  (void)adc_driver.initialize();

  binding::serial_registers serial{};
  std::array<std::byte, 32U> tx{};
  std::array<std::byte, 64U> rx{};
  hal::stm32f4::serial::DmaPort<binding::serial_registers,
                                binding::dma_stream_registers,
                                binding::dma_stream_registers, 32U, 64U>
      serial_driver{serial, stream, stream, 45'000'000U, 4U, 4U, tx, rx};
  (void)serial_driver.configure({hal::hertz{115'200U}});
  (void)serial_driver.try_write({tx.data(), 1U});
  (void)serial_driver.try_read({rx.data(), 1U});
  serial_driver.on_tx_dma_interrupt(true);
  serial_driver.on_rx_dma_interrupt(true);
  (void)serial_driver.fault();

  binding::spi_registers spi{};
  std::array<std::byte, 32U> spi_tx{};
  std::array<std::byte, 32U> spi_rx{};
  hal::stm32f4::spi::DmaBus8<binding::spi_registers,
                             binding::dma_stream_registers,
                             binding::dma_stream_registers, 32U>
      spi_driver{
          spi, stream, stream, 45'000'000U, hal::stm32f4::poll_budget{10U},
          0U,  0U,     spi_tx, spi_rx};
  (void)spi_driver.configure({hal::hertz{1'000'000U}});

  binding::i2c_registers i2c{};
  hal::stm32f4::i2c::Controller<binding::i2c_registers, 1U,
                                binding::dma_stream_registers,
                                binding::dma_stream_registers>
      i2c_driver{i2c,
                 stream,
                 stream,
                 {45'000'000U, 100'000U, hal::stm32f4::poll_budget{10U}, true}};
  (void)i2c_driver.initialize();

  binding::can_registers can{};
  hal::stm32f4::can::Controller<binding::can_registers> can_driver{
      can, {0x001C0003U, hal::stm32f4::poll_budget{10U}, false}};
  (void)can_driver.initialize();

  binding::sdio_registers sdio{};
  hal::stm32f4::block::SdmmcDevice<binding::sdio_registers> block{
      sdio, {0U, 1U, hal::stm32f4::poll_budget{10U}, true, true, false}};
  (void)block.initialize();

  binding::flash_registers flash{};
  hal::stm32f4::nv::NorFlash<binding::flash_registers, 0x08000000U,
                             512U * 1024U>
      flash_driver{flash, hal::stm32f4::poll_budget{10U}};
  (void)flash_driver.sync();

  binding::gpio_registers gpio{};
  binding::syscfg_registers syscfg{};
  binding::exti_registers exti{};
  hal::stm32f4::gpio::OutputPin output{gpio, 1U};
  (void)output.write(hal::gpio::level::low);
  hal::stm32f4::gpio::EdgeInput edge{gpio, syscfg, exti, 1U, 0U};
  (void)edge.configure_edge(hal::gpio::edge::rising);
}

static_assert(
    hal::stm32f4::device::stm32f446vet6::capabilities::can_instances == 2U);

[[nodiscard]] int test_receive_sequences() {
  auto receive = []<std::size_t Count>() {
    binding::i2c_registers registers{};
    registers.SR1 = (1U << 0U) | (1U << 1U) | (1U << 2U) | (1U << 6U) |
                    (1U << 7U);
    hal::stm32f4::i2c::Controller<binding::i2c_registers> controller{
        registers,
        {45'000'000U, 100'000U, hal::stm32f4::poll_budget{10U}, false}};
    std::array<std::byte, Count> data{};
    const auto initialized = controller.initialize();
    const auto received = hal::i2c::read(
        controller, hal::i2c::address7{0x10U},
        hal::span<std::byte>{data.data(), data.size()});
    constexpr std::uint32_t stop = 1U << 9U;
    constexpr std::uint32_t ack = 1U << 10U;
    constexpr std::uint32_t pos = 1U << 11U;
    return initialized && received && (registers.CR1 & stop) != 0U &&
           (registers.CR1 & ack) != 0U && (registers.CR1 & pos) == 0U;
  };

  if (!receive.template operator()<1U>()) return 1;
  if (!receive.template operator()<2U>()) return 2;
  if (!receive.template operator()<4U>()) return 3;
  return 0;
}

[[nodiscard]] int test_spi_enable_and_cleanup() {
  binding::spi_registers polling_registers{};
  polling_registers.SR = (1U << 0U) | (1U << 1U);
  hal::stm32f4::spi::PollingBus8 polling{
      polling_registers, 45'000'000U, hal::stm32f4::poll_budget{10U}};
  std::array<std::byte, 1U> byte{std::byte{0x5AU}};
  if (!polling.configure({hal::hertz{1'000'000U}}) ||
      !polling.transfer(byte, byte) ||
      (polling_registers.CR1 & (1U << 6U)) != 0U) {
    return 4;
  }

  binding::spi_registers dma_registers{};
  binding::dma_stream_registers tx_stream{};
  binding::dma_stream_registers rx_stream{};
  std::array<std::byte, 4U> tx{};
  std::array<std::byte, 4U> rx{};
  hal::stm32f4::spi::DmaBus8<binding::spi_registers,
                              binding::dma_stream_registers,
                              binding::dma_stream_registers, 4U>
      dma{dma_registers, tx_stream, rx_stream, 45'000'000U,
          hal::stm32f4::poll_budget{2U}, 0U, 0U, tx, rx};
  const auto configured = dma.configure({hal::hertz{1'000'000U}});
  const auto timed_out = dma.transfer(tx, rx);
  if (!configured || timed_out ||
      timed_out.error().kind() != hal::spi::error_kind::timeout ||
      (tx_stream.CR & 1U) != 0U || (rx_stream.CR & 1U) != 0U ||
      (dma_registers.CR1 & (1U << 6U)) != 0U ||
      (dma_registers.CR2 & 3U) != 0U) {
    return 5;
  }
  return 0;
}
} // namespace

int main() {
  instantiate();
  const int receive = test_receive_sequences();
  return receive == 0 ? test_spi_enable_and_cleanup() : receive;
}
