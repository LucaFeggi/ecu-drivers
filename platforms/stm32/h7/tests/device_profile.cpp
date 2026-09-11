#include <array>
#include <hal/mcu/stm32h723vgt6/device.hpp>
#include <hal/mcu/stm32h723vgt6/dma.hpp>
#include <hal/mcu/stm32h723vgt6/interrupts.hpp>
#include <hal/mcu/stm32h723vgt6/memory.hpp>
#include <hal/mcu/stm32h723vgt6/pins.hpp>
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

namespace profile = hal::stm32h7::device::stm32h723vgt6;

constexpr hal::adc::characteristics adc_characteristics{
    16U, hal::microvolts{0}, hal::microvolts{3'300'000}};
using adc_channel =
    hal::stm32h7::adc::ContinuousChannel<adc_characteristics, 8U>;

struct gpio_registers {
  volatile std::uint32_t MODER{};
  volatile std::uint32_t OTYPER{};
  volatile std::uint32_t OSPEEDR{};
  volatile std::uint32_t PUPDR{};
  volatile std::uint32_t IDR{};
  volatile std::uint32_t ODR{};
  volatile std::uint32_t BSRR{};
};

using input_pin = hal::stm32h7::gpio::InputPin<gpio_registers>;
using output_pin = hal::stm32h7::gpio::OutputPin<gpio_registers>;

struct adc_registers {
  volatile std::uint32_t ISR{};
  volatile std::uint32_t IER{};
  volatile std::uint32_t CR{};
  volatile std::uint32_t CFGR{};
  volatile std::uint32_t CFGR2{};
  volatile std::uint32_t SMPR1{};
  volatile std::uint32_t SMPR2{};
  volatile std::uint32_t PCSEL_RES0{};
  volatile std::uint32_t SQR1{};
  volatile std::uint32_t DIFSEL_RES12{};
  volatile std::uint32_t DR{};
};
struct adc_common_registers {
  volatile std::uint32_t CCR{};
};
struct dma_registers {
  volatile std::uint32_t LISR{};
  volatile std::uint32_t LIFCR{};
};
struct dma_stream_registers {
  volatile std::uint32_t CR{};
  volatile std::uint32_t NDTR{};
  volatile std::uint32_t PAR{};
  volatile std::uint32_t M0AR{};
  volatile std::uint32_t FCR{};
};
struct dmamux_registers {
  volatile std::uint32_t CCR{};
};
struct timer_registers {
  volatile std::uint32_t CR1{};
  volatile std::uint32_t CR2{};
  volatile std::uint32_t DIER{};
  volatile std::uint32_t SR{};
  volatile std::uint32_t EGR{};
  volatile std::uint32_t CNT{};
  volatile std::uint32_t PSC{};
  volatile std::uint32_t ARR{};
};
struct serial_registers {
  volatile std::uint32_t CR1{};
  volatile std::uint32_t CR2{};
  volatile std::uint32_t CR3{};
  volatile std::uint32_t BRR{};
  volatile std::uint32_t ISR{};
  volatile std::uint32_t ICR{};
  volatile std::uint32_t RDR{};
  volatile std::uint32_t TDR{};
};
struct spi_registers {
  volatile std::uint32_t CR1{};
  volatile std::uint32_t CR2{};
  volatile std::uint32_t CFG1{};
  volatile std::uint32_t CFG2{};
  volatile std::uint32_t SR{};
  volatile std::uint32_t IFCR{};
  volatile std::uint32_t TXDR{};
  volatile std::uint32_t RXDR{};
};

struct dma_stream_registers_ext {
  volatile std::uint32_t CR{};
  volatile std::uint32_t NDTR{};
  volatile std::uint32_t PAR{};
  volatile std::uint32_t M0AR{};
  volatile std::uint32_t FCR{};
};
struct dmamux_registers_ext { volatile std::uint32_t CCR{}; };
struct i2c_registers {
  volatile std::uint32_t CR1{};
  volatile std::uint32_t CR2{};
  volatile std::uint32_t TIMINGR{};
  volatile std::uint32_t ISR{};
  volatile std::uint32_t ICR{};
  volatile std::uint32_t TXDR{};
  volatile std::uint32_t RXDR{};
};
struct can_registers {
  volatile std::uint32_t CCCR{};
  volatile std::uint32_t NBTP{};
  volatile std::uint32_t DBTP{};
  volatile std::uint32_t TSCC{};
  volatile std::uint32_t GFC{};
  volatile std::uint32_t SIDFC{};
  volatile std::uint32_t XIDFC{};
  volatile std::uint32_t RXF0C{};
  volatile std::uint32_t RXF0S{};
  volatile std::uint32_t RXF0A{};
  volatile std::uint32_t RXESC{};
  volatile std::uint32_t TXBC{};
  volatile std::uint32_t TXESC{};
  volatile std::uint32_t TXFQS{};
  volatile std::uint32_t TXBAR{};
  volatile std::uint32_t IE{};
  volatile std::uint32_t ILE{};
  volatile std::uint32_t IR{};
};
struct sdmmc_registers {
  volatile std::uint32_t POWER{};
  volatile std::uint32_t CLKCR{};
  volatile std::uint32_t ARG{};
  volatile std::uint32_t CMD{};
  volatile std::uint32_t RESP1{};
  volatile std::uint32_t RESP2{};
  volatile std::uint32_t RESP3{};
  volatile std::uint32_t RESP4{};
  volatile std::uint32_t DTIMER{};
  volatile std::uint32_t DLEN{};
  volatile std::uint32_t DCTRL{};
  volatile std::uint32_t STA{};
  volatile std::uint32_t ICR{};
  volatile std::uint32_t FIFO{};
};
struct flash_registers {
  volatile std::uint32_t KEYR1{};
  volatile std::uint32_t CR1{};
  volatile std::uint32_t SR1{};
  volatile std::uint32_t CCR1{};
};
struct pwm_registers {
  volatile std::uint32_t CR1{};
  volatile std::uint32_t PSC{};
  volatile std::uint32_t ARR{};
  volatile std::uint32_t EGR{};
  volatile std::uint32_t CCER{};
  volatile std::uint32_t CCR1{};
  volatile std::uint32_t CCR2{};
  volatile std::uint32_t CCR3{};
  volatile std::uint32_t CCR4{};
  volatile std::uint32_t BDTR{};
};
struct rtc_registers {
  volatile std::uint32_t WPR{};
  volatile std::uint32_t ISR{};
  volatile std::uint32_t TR{};
  volatile std::uint32_t DR{};
  volatile std::uint32_t CR{};
  volatile std::uint32_t ALRMAR{};
};
struct watchdog_registers {
  volatile std::uint32_t KR{};
  volatile std::uint32_t PR{};
  volatile std::uint32_t RLR{};
  volatile std::uint32_t SR{};
};
struct ethernet_registers {
  volatile std::uint32_t MACMDIOAR{};
  volatile std::uint32_t MACMDIODR{};
  volatile std::uint32_t MACCR{};
  volatile std::uint32_t MACA0HR{};
  volatile std::uint32_t MACA0LR{};
  volatile std::uint32_t DMACCR{};
  volatile std::uint32_t DMACRDLAR{};
  volatile std::uint32_t DMACTDLAR{};
  volatile std::uint32_t DMACRDRLR{};
  volatile std::uint32_t DMACTDRLR{};
  volatile std::uint32_t DMAMR{};
  volatile std::uint32_t DMACTCR{};
  volatile std::uint32_t DMACRCR{};
  volatile std::uint32_t DMACTDTPR{};
  volatile std::uint32_t DMACRDTPR{};
};

constexpr hal::adc::characteristics oversampled_characteristics{
    18U, hal::microvolts{0}, hal::microvolts{3'300'000}};
constexpr hal::stm32h7::adc::dma_config adc_dma_configuration{
    64'000'000U, hal::stm32h7::adc::asynchronous_prescaler::divide_by_2,
    4U,          3U,
    16U,         2U,
    13U,         9U,
    63U,         199U,
    200'000U,    hal::stm32h7::poll_budget{100U}};
using dma_adc = hal::stm32h7::adc::Adc12DmaStream0<
    oversampled_characteristics, 16U, 8U, adc_registers, adc_common_registers,
    dma_registers, dma_stream_registers, dmamux_registers, timer_registers>;
using serial_port = hal::stm32h7::serial::PollingPort<serial_registers>;
using spi_bus = hal::stm32h7::spi::PollingBus8<spi_registers>;
using timer_clock =
    hal::stm32h7::time::TimerClock<64'000'000U, 1'000'000U, timer_registers>;
using i2c_controller = hal::stm32h7::i2c::Controller<i2c_registers>;
using i2c_dma_controller =
    hal::stm32h7::i2c::Controller<i2c_registers, 1U,
                                  dma_stream_registers_ext,
                                  dmamux_registers_ext>;
using can_controller = hal::stm32h7::can::Controller<can_registers>;
using sdmmc_device = hal::stm32h7::block::SdmmcDevice<sdmmc_registers>;
using flash_device = hal::stm32h7::nv::NorFlash<flash_registers, 0U,
                                                1024U * 1024U>;
using pwm_output = hal::stm32h7::pwm::Output<pwm_registers, 64'000'000U, 1U>;
using complementary_pwm_output =
    hal::stm32h7::pwm::Output<pwm_registers, 64'000'000U, 2U,
                              hal::stm32h7::pwm::default_compare_accessor, true>;
using rtc_clock = hal::stm32h7::rtc::Clock<rtc_registers>;
using watchdog_feeder = hal::stm32h7::watchdog::Feeder<watchdog_registers,
                                                        32'000U>;
using mdio_bus = hal::stm32h7::ethernet::MdioBus<ethernet_registers>;
using phy = hal::stm32h7::ethernet::Phy<mdio_bus, 1U>;
using ethernet_mac = hal::stm32h7::ethernet::Mac<ethernet_registers, 2U, 2U>;
using dma_serial = hal::stm32h7::serial::DmaPort<
    serial_registers, dma_stream_registers_ext, dma_stream_registers_ext,
    dmamux_registers_ext, dmamux_registers_ext>;
using dma_spi = hal::stm32h7::spi::DmaBus8<
    spi_registers, dma_stream_registers_ext, dma_stream_registers_ext,
    dmamux_registers_ext, dmamux_registers_ext>;

static_assert(hal::i2c::Controller7<i2c_controller>);
static_assert(hal::i2c::Controller10<i2c_controller>);
static_assert(hal::i2c::Controller7<i2c_dma_controller>);
static_assert(hal::can::ClassicController<can_controller>);
static_assert(hal::can::FdController<can_controller>);
static_assert(hal::can::FilterableController<can_controller>);
static_assert(hal::block::TrimDevice<sdmmc_device>);
static_assert(hal::nv::NorFlash<flash_device>);
static_assert(hal::pwm::ComplementaryOutput<pwm_output>);
static_assert(hal::pwm::ComplementaryOutput<complementary_pwm_output>);
static_assert(hal::rtc::Alarm<rtc_clock>);
static_assert(hal::watchdog::Feeder<watchdog_feeder>);
static_assert(hal::ethernet::MdioBus<mdio_bus>);
static_assert(hal::ethernet::Phy<phy>);
static_assert(hal::ethernet::Mac<ethernet_mac>);
using ethernet_port = hal::stm32h7::ethernet::FramePort<ethernet_mac, phy>;
static_assert(hal::ethernet::FramePort<ethernet_port>);
static_assert(hal::serial::ConfigurablePort<dma_serial>);
static_assert(hal::spi::Bus8<dma_spi>);

template class hal::stm32h7::i2c::Controller<i2c_registers>;
template class hal::stm32h7::i2c::Controller<
    i2c_registers, 1U, dma_stream_registers_ext, dmamux_registers_ext>;
template class hal::stm32h7::can::Controller<can_registers>;
template class hal::stm32h7::block::SdmmcDevice<sdmmc_registers>;
template class hal::stm32h7::nv::NorFlash<flash_registers, 0U,
                                           1024U * 1024U>;
template class hal::stm32h7::pwm::Output<pwm_registers, 64'000'000U, 1U>;
template class hal::stm32h7::pwm::Output<
    pwm_registers, 64'000'000U, 2U,
    hal::stm32h7::pwm::default_compare_accessor, true>;
template class hal::stm32h7::rtc::Clock<rtc_registers>;
template class hal::stm32h7::watchdog::Feeder<watchdog_registers, 32'000U>;
template class hal::stm32h7::ethernet::MdioBus<ethernet_registers>;
template class hal::stm32h7::ethernet::Mac<ethernet_registers, 2U, 2U>;


static_assert(profile::capabilities::adc_instances == 3U);
static_assert(profile::capabilities::fdcan_instances == 3U);
static_assert(profile::capabilities::adc1_adc2_timer_triggered_dma);
static_assert(!profile::capabilities::adc3_multi_channel_scan_reliable);
static_assert(profile::data_cache_line_bytes == 32U);
static_assert(profile::adc1_pin::inp4_channel == 4U);
static_assert(profile::dmamux1_request::adc1 == 9U);
static_assert(profile::interrupt_number::dma1_stream0 == 11);
static_assert(profile::sram1.dma1_accessible);
static_assert(!profile::dtcm.dma1_accessible);
static_assert(adc_dma_configuration.valid());
static_assert(adc_dma_configuration.analog_clock_hz() == 16'000'000U);
static_assert(hal::adc::ContinuousChannel<adc_channel>);
static_assert(hal::adc::ContinuousChannel<dma_adc>);
static_assert(hal::gpio::InputPin<input_pin>);
static_assert(hal::gpio::StatefulOutputPin<output_pin>);
static_assert(hal::serial::ConfigurablePort<serial_port>);
static_assert(hal::spi::Bus8<spi_bus>);
static_assert(hal::time::MonotonicClock<timer_clock>);
static_assert(hal::time::Delay<timer_clock>);
static_assert(hal::stm32h7::adc::publication_support.available());
static_assert(hal::stm32h7::adc::acquisition_support.available());
static_assert(hal::stm32h7::gpio::support.available());
static_assert(hal::stm32h7::serial::support.available());
static_assert(hal::stm32h7::spi::support.available());
static_assert(hal::stm32h7::time::support.available());
static_assert(hal::stm32h7::can::classic_support.available());
static_assert(hal::stm32h7::can::fd_support.available());

int main() {
  adc_channel channel;
  const auto before_first_conversion = channel.read_raw();
  if (before_first_conversion || before_first_conversion.error().kind() !=
                                     hal::adc::error_kind::not_ready) {
    return 1;
  }

  channel.publish_from_isr(123U, hal::instant{10U});
  const auto latest = channel.read_raw();
  if (!latest || latest.value().value != 123U ||
      latest.value().captured_at.nanoseconds_since_boot != 10U ||
      latest.value().sequence != 1U) {
    return 2;
  }

  std::array<hal::adc::raw_sample, 8U> samples{};
  const auto first_read = channel.try_read(samples);
  if (!first_read || first_read.value().count != 1U ||
      first_read.value().dropped != 0U || samples[0].sequence != 1U) {
    return 3;
  }

  for (std::uint32_t raw = 2U; raw <= 10U; ++raw) {
    channel.publish_from_isr(raw, hal::instant{raw});
  }
  const auto overrun_read = channel.try_read(samples);
  if (!overrun_read || overrun_read.value().count != 8U ||
      overrun_read.value().dropped != 1U || samples[0].sequence != 3U ||
      samples[7].sequence != 10U) {
    return 4;
  }

  gpio_registers registers{};
  if (!hal::stm32h7::gpio::configure_output(registers, 3U,
                                            hal::gpio::level::low) ||
      (registers.MODER & (3U << 6U)) != (1U << 6U)) {
    return 5;
  }
  output_pin output{registers, 3U};
  if (!output.write(hal::gpio::level::high) || registers.BSRR != 8U) {
    return 6;
  }
  registers.ODR = 8U;
  const auto latch = output.output_latch();
  if (!latch || latch.value() != hal::gpio::level::high) {
    return 7;
  }

  input_pin input{registers, 4U};
  registers.IDR = 16U;
  const auto level = input.read();
  if (!level || level.value() != hal::gpio::level::high) {
    return 8;
  }

  serial_registers usart{};
  usart.ISR = (1U << 21U) | (1U << 22U) | (1U << 7U) | (1U << 6U);
  serial_port serial{usart, 64'000'000U, hal::stm32h7::poll_budget{100U}};
  const auto baud =
      serial.configure({hal::hertz{115'200U}, hal::serial::data_bits::bits8,
                        hal::serial::parity::none, hal::serial::stop_bits::one,
                        hal::serial::flow_control::none});
  const std::array<std::byte, 2U> serial_bytes{std::byte{0x55U},
                                               std::byte{0xAAU}};
  if (!baud || baud.value().value < 115'000U ||
      !serial.try_write(serial_bytes) || usart.TDR != 0xAAU ||
      !serial.flush()) {
    return 9;
  }

  spi_registers spi{};
  spi.SR = (1U << 0U) | (1U << 1U) | (1U << 3U) | (1U << 12U);
  spi.RXDR = 0x5AU;
  spi_bus bus{spi, 64'000'000U, hal::stm32h7::poll_budget{100U}};
  const auto spi_clock =
      bus.configure({hal::hertz{8'000'000U}, hal::spi::mode::mode0,
                     hal::spi::bit_order::msb_first, std::byte{0xFFU}});
  const std::array<std::byte, 1U> transmitted{std::byte{0xA5U}};
  std::array<std::byte, 1U> received{};
  if (!spi_clock || spi_clock.value().value != 8'000'000U ||
      !bus.transfer(transmitted, received) || spi.TXDR != 0xA5U ||
      received[0] != std::byte{0x5AU}) {
    return 10;
  }

  constexpr hal::adc::characteristics invalid_adc_characteristics{
      16U, hal::microvolts{3'300'000}, hal::microvolts{0}};
  using invalid_adc_channel =
      hal::stm32h7::adc::ContinuousChannel<invalid_adc_characteristics, 2U>;
  invalid_adc_channel invalid_adc;
  invalid_adc.publish_from_isr(123U, hal::instant{20U});
  const auto invalid_voltage = invalid_adc.read_voltage();
  if (invalid_voltage ||
      invalid_voltage.error().kind() != hal::adc::error_kind::configuration) {
    return 11;
  }

  i2c_registers invalid_i2c_registers{};
  i2c_controller invalid_i2c{
      invalid_i2c_registers,
      {0U, hal::stm32h7::poll_budget{100U}, 0U, false}};
  const auto invalid_i2c_init = invalid_i2c.initialize();
  if (invalid_i2c_init ||
      invalid_i2c_init.error().kind() != hal::i2c::error_kind::configuration) {
    return 12;
  }

  serial_registers timeout_usart{};
  serial_port timeout_serial{timeout_usart, 64'000'000U,
                             hal::stm32h7::poll_budget{100U}};
  const auto timeout_configuration = timeout_serial.configure(
      {hal::hertz{115'200U}, hal::serial::data_bits::bits8,
       hal::serial::parity::none, hal::serial::stop_bits::one,
       hal::serial::flow_control::none});
  if (timeout_configuration ||
      timeout_configuration.error().kind() !=
          hal::serial::error_kind::timeout) {
    return 13;
  }
  return 0;
}
