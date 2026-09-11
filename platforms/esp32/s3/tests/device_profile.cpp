#include <array>
#include <string_view>
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
#include <hal/serial/dma_port.hpp>
#include <hal/spi.hpp>
#include <hal/support.hpp>
#include <hal/time.hpp>
#include <hal/watchdog.hpp>
#include <hal/mcu/esp32s3_wroom_1_n16r8/capabilities.hpp>
#include <hal/mcu/esp32s3_wroom_1_n16r8/bindings.hpp>
#include <hal/mcu/esp32s3_wroom_1_n16r8/clock.hpp>
#include <hal/mcu/esp32s3_wroom_1_n16r8/device.hpp>
#include <hal/mcu/esp32s3_wroom_1_n16r8/dma.hpp>
#include <hal/mcu/esp32s3_wroom_1_n16r8/interrupts.hpp>
#include <hal/mcu/esp32s3_wroom_1_n16r8/memory.hpp>
#include <hal/mcu/esp32s3_wroom_1_n16r8/pins.hpp>

namespace profile = hal::esp32s3_wroom_1_n16r8;

static_assert(std::string_view{profile::device::name} ==
              "esp32s3_wroom_1_n16r8");
static_assert(profile::device::flash_bytes == 16U * 1024U * 1024U);
static_assert(profile::device::psram_bytes == 8U * 1024U * 1024U);
static_assert(profile::device::gpio_count == 49U);
static_assert(!profile::device::is_valid_gpio(35U));
static_assert(!profile::device::is_output_capable(46U));
static_assert(profile::device::spi_dma.status ==
              profile::implementation_status::available);
static_assert(profile::device::uart_dma.status ==
              profile::implementation_status::available);
static_assert(profile::device::adc_dma.status ==
              profile::implementation_status::available);
static_assert(profile::device::sdmmc_dma.status ==
              profile::implementation_status::available);
static_assert(profile::device::native_ethernet.status ==
              profile::implementation_status::unsupported);

static_assert(sizeof(profile::device::gdma_descriptor) == 12U);
static_assert(profile::device::gdma_length_shift == 12U);

static_assert(hal::gpio::OutputPin<profile::gpio::OutputPin>);
static_assert(hal::gpio::InputPin<profile::gpio::InputPin>);
static_assert(hal::gpio::EdgeInput<profile::gpio::EdgeInput>);
static_assert(hal::block::Device<profile::block::Sdmmc<>>);
static_assert(hal::block::TrimDevice<profile::block::Sdmmc<>>);
static_assert(hal::ethernet::FramePort<profile::ethernet::FramePort>);

constexpr hal::adc::characteristics adc_characteristics{
    12U, hal::microvolts{0U}, hal::microvolts{3'300'000U}};
using adc_reader = profile::adc::ChannelReader<
    profile::adc::unit::one, 0U, adc_characteristics>;
using adc_stream = profile::adc::Continuous<adc_characteristics, 8U>;
using serial_port = profile::serial::Port<>;
using serial_dma_port = profile::serial::DmaPort<>;
using spi_bus = profile::spi::DmaBus8<>;
using i2c_controller = profile::i2c::Controller<>;
using can_controller = profile::can::Controller<>;
using pwm_output = profile::pwm::Output<0U, 0U>;
using rtc_clock = profile::rtc::Clock<>;
using monotonic_clock = profile::time::MonotonicClock<>;
using monotonic_alarm = profile::time::OneShotAlarm<>;
using watchdog = profile::watchdog::Feeder<>;
using flash = profile::nv::Flash<>;

static_assert(hal::adc::Channel<adc_reader>);
static_assert(hal::adc::ContinuousChannel<adc_stream>);
static_assert(hal::serial::ConfigurablePort<serial_port>);
static_assert(hal::serial::ConfigurablePort<serial_dma_port>);
static_assert(hal::spi::Bus8<spi_bus>);
static_assert(hal::spi::LsbFirstBus8<spi_bus>);
static_assert(hal::i2c::Controller7<i2c_controller>);
static_assert(hal::i2c::Controller10<i2c_controller>);
static_assert(hal::can::ClassicController<can_controller>);
static_assert(hal::can::FilterableController<can_controller>);
static_assert(hal::pwm::Output<pwm_output>);
static_assert(hal::rtc::Clock<rtc_clock>);
static_assert(hal::rtc::Alarm<rtc_clock>);
static_assert(hal::time::MonotonicClock<monotonic_clock>);
static_assert(hal::time::Delay<profile::time::Delay<>>);
static_assert(hal::time::OneShotAlarm<monotonic_alarm>);
static_assert(hal::watchdog::Feeder<watchdog>);
static_assert(hal::nv::NorFlash<flash>);

// This function is intentionally not executed on the host. Its calls force
// GCC to instantiate the concrete register-accessing method bodies, covering
// template code that a concepts-only test would otherwise leave unchecked.
[[maybe_unused]] static void instantiate_driver_bodies() {
  gpio_dev_t gpio{};
  apb_saradc_dev_t adc{};
  sens_dev_t sens{};
  systimer_dev_t systimer{};
  gdma_dev_t gdma{};
  uhci_dev_t uhci{};
  spi_dev_t spi{};
  i2c_dev_t i2c{};
  twai_dev_t twai{};
  ledc_dev_t ledc{};
  rtc_cntl_dev_t rtc_registers{};
  timg_dev_t timer_group{};
  spi_mem_dev_t flash_registers{};
  sdmmc_dev_t sdmmc_registers{};
  uart_dev_t uart{};

  profile::gpio::OutputPin output_pin{gpio, 0U};
  profile::gpio::InputPin input_pin{gpio, 1U};
  profile::gpio::EdgeInput edge_input{gpio, 2U};
  (void)output_pin.configure();
  (void)input_pin.configure();
  (void)edge_input.configure_edge(hal::gpio::edge::both);
  (void)output_pin.write(hal::gpio::level::low);
  (void)input_pin.read();
  (void)edge_input.take_event();
  (void)profile::gpio::connect_input(
      gpio, 3U, profile::gpio::signal_index::uart0_rx);
  (void)profile::gpio::connect_output(
      gpio, 4U, profile::gpio::signal_index::uart0_tx);
  (void)profile::gpio::connect_bidirectional(
      gpio, 5U, profile::gpio::signal_index::i2c0_sda);
  (void)profile::gpio::connect_output(
      gpio, 6U, profile::gpio::signal_index::sdmmc0_clk);

  std::array<profile::device::gdma_descriptor, 2U> descriptors{};
  std::array<std::byte, 128U> scratch{};
  std::array<std::byte, 128U> receive{};
  profile::device::dma_array<std::uint32_t, 32U> adc_samples{};
  std::array<sdmmc_desc_t, 2U> sdmmc_descriptors{};
  profile::spi::DmaBus8<> bus{
      spi, gdma, {descriptors.data(), 1U}, {descriptors.data() + 1U, 1U},
      scratch, receive};
  (void)bus.configure({hal::hertz{1'000'000U}});
  (void)bus.transfer(scratch, receive);

  profile::i2c::Controller<> i2c_bus{i2c};
  (void)i2c_bus.configure({});
  std::array<std::byte, 1U> byte{};
  hal::i2c::operation operation = hal::i2c::operation::write(byte);
  (void)i2c_bus.transaction(hal::i2c::address7{0x20U}, {&operation, 1U});

  profile::serial::Port<> serial{uart};
  (void)serial.configure({hal::hertz{115'200U}});
  profile::serial::DmaPort<> dma_serial{
      uart, uhci, gdma, 0U,
      {descriptors.data(), 1U}, scratch};
  (void)dma_serial.configure({hal::hertz{115'200U}});

  profile::adc::ChannelReader<profile::adc::unit::one, 0U,
                              adc_characteristics>
      reader{adc, sens};
  (void)reader.configure();
  (void)reader.read_raw();
  profile::time::MonotonicClock<> clock{systimer};
  profile::adc::Continuous<adc_characteristics, 2U, 2U,
                           profile::time::MonotonicClock<>>
      stream{
      adc, sens, gdma, clock, {descriptors.data(), 2U},
      {adc_samples.values, 32U}};
  const profile::adc::pattern pattern{0U, 0U};
  (void)stream.configure({{&pattern, 1U}, hal::hertz{1'000U}});
  (void)stream.start();
  (void)stream.stop();

  profile::can::Controller<> can{twai};
  (void)can.configure({});
  (void)can.set_filters({});
  (void)can.bus_off();

  profile::pwm::Output<0U, 0U> pwm{ledc};
  (void)pwm.configure({hal::nanoseconds{1'000'000U}, 1'000U});
  (void)pwm.set_pulse_width(hal::nanoseconds{500'000U});
  (void)pwm.enable_output();
  (void)pwm.disable_output();

  profile::rtc::Clock<> real_time{rtc_registers};
  (void)real_time.read();
  (void)real_time.set({0, 0U});
  (void)real_time.clear_alarm();
  profile::time::Delay<> delay{clock};
  (void)delay;
  profile::time::OneShotAlarm<> alarm{systimer};
  (void)alarm.arm(clock.now());
  (void)alarm.cancel();

  profile::watchdog::Feeder<> feeder{timer_group};
  (void)feeder.start();
  (void)feeder.feed();
  (void)feeder.stop();

  profile::nv::Flash<> nonvolatile{flash_registers};
  std::array<std::byte, 4U> nv_input{};
  std::array<std::byte, 4U> nv_output{};
  (void)nonvolatile.read(0U, nv_output);
  (void)nonvolatile.program(0U, nv_input);
  (void)nonvolatile.erase(0U, 4U * 1024U);
  (void)nonvolatile.sync();

  profile::block::Sdmmc<> block{
      sdmmc_registers, sdmmc_descriptors,
      scratch};
  (void)block.initialize();
  std::array<std::byte, 512U> block_buffer{};
  (void)block.read_blocks(0U, block_buffer);
  (void)block.write_blocks(0U, block_buffer);
  (void)block.trim(0U, 1U);
  (void)block.sync();
}

int main() { return 0; }
