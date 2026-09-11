#include <array>
#include <cstddef>
#include <cstdint>

#include <hal/adc.hpp>
#include <hal/can.hpp>
#include <hal/gpio.hpp>
#include <hal/i2c.hpp>
#include <hal/nv.hpp>
#include <hal/pwm.hpp>
#include <hal/rtc.hpp>
#include <hal/serial.hpp>
#include <hal/contracts/spi_bus.hpp>
#include <hal/mcu/stm32g484ce/device.hpp>
#include <hal/mcu/stm32g484ce/dma.hpp>
#include <hal/mcu/stm32g484ce/device.hpp>
#include <hal/mcu/stm32g484ce/interrupts.hpp>
#include <hal/mcu/stm32g484ce/memory.hpp>
#include <hal/mcu/stm32g484ce/pins.hpp>
#include <hal/adc.hpp>
#include <hal/contracts/block.hpp>
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
#include <hal/time.hpp>
#include <hal/watchdog.hpp>

namespace profile = hal::stm32g4::device::stm32g484ce;

struct gpio_registers {
  volatile std::uint32_t MODER{};
  volatile std::uint32_t OTYPER{};
  volatile std::uint32_t OSPEEDR{};
  volatile std::uint32_t PUPDR{};
  volatile std::uint32_t IDR{};
  volatile std::uint32_t ODR{};
  volatile std::uint32_t BSRR{};
  volatile std::uint32_t AFR[2U]{};
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
  volatile std::uint32_t SR{};
  volatile std::uint32_t DR{};
};
struct i2c_registers {
  volatile std::uint32_t CR1{};
  volatile std::uint32_t CR2{};
  volatile std::uint32_t TIMINGR{};
  volatile std::uint32_t ISR{};
  volatile std::uint32_t ICR{};
  volatile std::uint32_t RXDR{};
  volatile std::uint32_t TXDR{};
};
struct dma_registers {
  volatile std::uint32_t ISR{};
  volatile std::uint32_t IFCR{};
};
struct dma_channel_registers {
  volatile std::uint32_t CCR{};
  volatile std::uint32_t CNDTR{};
  volatile std::uint32_t CPAR{};
  volatile std::uint32_t CMAR{};
};
struct dmamux_registers { volatile std::uint32_t CCR{}; };
struct adc_registers {
  volatile std::uint32_t ISR{};
  volatile std::uint32_t IER{};
  volatile std::uint32_t CR{};
  volatile std::uint32_t CFGR{};
  volatile std::uint32_t CFGR2{};
  volatile std::uint32_t SMPR1{};
  volatile std::uint32_t SMPR2{};
  volatile std::uint32_t SQR1{};
  volatile std::uint32_t DIFSEL{};
  volatile std::uint32_t DR{};
};
struct adc_common_registers { volatile std::uint32_t CCR{}; };
struct can_registers {
  volatile std::uint32_t CCCR{};
  volatile std::uint32_t NBTP{};
  volatile std::uint32_t DBTP{};
  volatile std::uint32_t TSCC{};
  volatile std::uint32_t RXGFC{};
  volatile std::uint32_t TXBC{};
  volatile std::uint32_t RXF0S{};
  volatile std::uint32_t RXF0A{};
  volatile std::uint32_t TXFQS{};
  volatile std::uint32_t TXBAR{};
  volatile std::uint32_t IE{};
  volatile std::uint32_t ILE{};
  volatile std::uint32_t IR{};
};
struct flash_registers {
  volatile std::uint32_t KEYR{};
  volatile std::uint32_t SR{};
  volatile std::uint32_t CR{};
};
struct pwm_registers {
  volatile std::uint32_t CR1{};
  volatile std::uint32_t PSC{};
  volatile std::uint32_t ARR{};
  volatile std::uint32_t EGR{};
  volatile std::uint32_t CCMR1{};
  volatile std::uint32_t CCMR2{};
  volatile std::uint32_t CCER{};
  volatile std::uint32_t CCR1{};
  volatile std::uint32_t CCR2{};
  volatile std::uint32_t CCR3{};
  volatile std::uint32_t CCR4{};
  volatile std::uint32_t BDTR{};
};
struct rtc_registers {
  volatile std::uint32_t WPR{};
  volatile std::uint32_t TR{};
  volatile std::uint32_t DR{};
  volatile std::uint32_t ICSR{};
  volatile std::uint32_t CR{};
  volatile std::uint32_t ALRMAR{};
  volatile std::uint32_t SR{};
  volatile std::uint32_t SCR{};
};
struct watchdog_registers {
  volatile std::uint32_t KR{};
  volatile std::uint32_t PR{};
  volatile std::uint32_t RLR{};
  volatile std::uint32_t SR{};
};

constexpr hal::adc::characteristics adc_characteristics{
    12U, hal::microvolts{0}, hal::microvolts{3'300'000}};
using continuous_adc = hal::stm32g4::adc::ContinuousChannel<
    adc_characteristics, 8U>;
using adc_driver = hal::stm32g4::adc::AdcDma<
    adc_characteristics, 8U, 16U, adc_registers, adc_common_registers,
    dma_registers, dma_channel_registers, dmamux_registers, timer_registers>;
using serial_port = hal::stm32g4::serial::PollingPort<serial_registers>;
using spi_bus = hal::stm32g4::spi::PollingBus8<spi_registers>;
using i2c_controller = hal::stm32g4::i2c::Controller<i2c_registers>;
using can_controller = hal::stm32g4::can::Controller<can_registers>;
using flash_device = hal::stm32g4::nv::NorFlash<flash_registers>;
using pwm_output = hal::stm32g4::pwm::Output<pwm_registers, 170'000'000U, 1U>;
using rtc_clock = hal::stm32g4::rtc::Clock<rtc_registers>;
using timer_clock = hal::stm32g4::time::TimerClock<170'000'000U, 1'000'000U,
                                                    timer_registers>;
using watchdog_feeder = hal::stm32g4::watchdog::Feeder<watchdog_registers,
                                                        32'000U>;

static_assert(hal::adc::ContinuousChannel<continuous_adc>);
static_assert(hal::adc::ContinuousChannel<adc_driver>);
static_assert(hal::gpio::InputPin<hal::stm32g4::gpio::InputPin<gpio_registers>>);
static_assert(hal::gpio::StatefulOutputPin<hal::stm32g4::gpio::OutputPin<gpio_registers>>);
static_assert(hal::serial::ConfigurablePort<serial_port>);
static_assert(hal::spi::Bus8<spi_bus>);
static_assert(hal::i2c::Controller7<i2c_controller>);
static_assert(hal::i2c::Controller10<i2c_controller>);
static_assert(hal::can::ClassicController<can_controller>);
static_assert(hal::can::FdController<can_controller>);
static_assert(hal::can::FilterableController<can_controller>);
static_assert(hal::nv::NorFlash<flash_device>);
static_assert(hal::pwm::ComplementaryOutput<pwm_output>);
static_assert(hal::rtc::Alarm<rtc_clock>);
static_assert(hal::time::MonotonicClock<timer_clock>);
static_assert(hal::time::Delay<timer_clock>);
static_assert(hal::watchdog::Feeder<watchdog_feeder>);

static_assert(profile::capabilities::adc_instances == 5U);
static_assert(profile::capabilities::fdcan_instances == 3U);
static_assert(profile::dma_channel_count == 16U);
static_assert(!profile::capabilities::ethernet_mac);
static_assert(profile::data_cache_line_bytes == 0U);
static_assert(profile::flash.size == 512U * 1024U);
static_assert(profile::sram1.dma_accessible);
static_assert(!profile::ccm_sram.dma_accessible);
static_assert(profile::dmamux_request::adc1 == 5U);
static_assert(profile::interrupt_number::adc1_adc2 == 18);

int main() {
  continuous_adc channel;
  channel.publish_from_isr(1234U, hal::instant{100U});
  const auto value = channel.read_raw();
  if (!value || value.value().value != 1234U) return 1;

  pwm_registers pwm_registers_instance{};
  pwm_output pwm{pwm_registers_instance};
  const auto period = pwm.configure(
      {hal::nanoseconds{1'000U}, 1'000U, hal::pwm::polarity::active_high});
  if (!period || (pwm_registers_instance.CCMR1 & (7U << 4U)) != (6U << 4U)) {
    return 2;
  }
  if (!pwm.set_pulse_width(hal::nanoseconds{500U})) return 3;

  can_registers can_registers_instance{};
  std::array<std::uint32_t, profile::fdcan_message_ram::words> message_ram{};
  can_controller can{
      can_registers_instance, message_ram.data(), message_ram.size(),
      {1U, 0U, 0U, 0U, 1U, 1U, 3U, 3U, hal::stm32g4::poll_budget{10U}, false,
       false}};
  if (!can.initialize()) return 4;
  can_registers_instance.IR = 0U;
  can_registers_instance.TXFQS = 3U;
  std::array<std::byte, 4U> transmit_data{
      std::byte{0x11U}, std::byte{0x22U}, std::byte{0x33U}, std::byte{0x44U}};
  const auto transmit_frame = hal::can::classic_frame::make(
      {0x123U, false}, {transmit_data.data(), transmit_data.size()});
  if (!transmit_frame) return 5;
  const auto sent = can.try_send(transmit_frame.value());
  if (!sent || !sent.value() || message_ram[160U] != 0x44332211U) return 6;

  message_ram[44U] = 0x123U << 18U;
  message_ram[45U] = 4U << 16U;
  message_ram[46U] = 0x88776655U;
  can_registers_instance.RXF0S = 1U;
  const auto received = can.try_receive();
  if (!received || !received.value().has_frame() ||
      received.value().value().size() != 4U) {
    return 7;
  }
  return 0;
}
