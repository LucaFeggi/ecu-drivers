#include <array>
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
#include <hal/adapters/nv_journal_memory.hpp>
#include <poll.h>
#include <time.h>
#include <type_traits>

using LinuxI2c = hal::linux::i2c::Controller<>;
using LinuxSpiBus = hal::linux::spi::Bus<>;
using LinuxSpiDevice = hal::linux::spi::Device<>;
using LinuxCan = hal::linux::can::Controller<>;
using LinuxClassicCan = hal::linux::can::Controller<false>;
using LinuxTap = hal::linux::ethernet::TapFramePort<>;
using LinuxPacket = hal::linux::ethernet::PacketFramePort<>;
using LinuxBlock = hal::linux::block::Device;
using LinuxFlash = hal::linux::nv::FileNorFlash<4096U, 1U, 256U>;
using LinuxMemory = hal::nv::JournalMemory<LinuxFlash, 32U, 4U, 0U, 256U>;
using LinuxAdc = hal::linux::adc::BufferedChannel<12U, 0, 3300000, 16U>;
using LinuxSerial = hal::linux::serial::Port;
using LinuxPwm = hal::linux::pwm::Output;
using LinuxSystemClock = hal::linux::rtc::SystemClock;
using LinuxSystemAlarm = hal::linux::rtc::SystemClockAlarm;
using LinuxHardwareClock = hal::linux::rtc::HardwareClock<>;
using LinuxWatchdog = hal::linux::watchdog::Feeder<60'000'000'000ULL>;

static_assert(hal::i2c::Controller7<LinuxI2c>);
static_assert(hal::i2c::Controller10<LinuxI2c>);
static_assert(hal::spi::Device8<LinuxSpiDevice>);
static_assert(hal::can::ClassicController<LinuxCan>);
static_assert(hal::can::FdController<LinuxCan>);
static_assert(hal::can::ClassicController<LinuxClassicCan>);
static_assert(hal::ethernet::FramePort<LinuxTap>);
static_assert(hal::ethernet::FramePort<LinuxPacket>);
static_assert(hal::block::Device<LinuxBlock>);
static_assert(hal::block::TrimDevice<LinuxBlock>);
static_assert(hal::nv::NorFlash<LinuxFlash>);
static_assert(hal::nv::Memory<LinuxMemory>);
static_assert(hal::adc::ContinuousChannel<LinuxAdc>);
static_assert(hal::serial::ConfigurablePort<LinuxSerial>);
static_assert(hal::pwm::Output<LinuxPwm>);
static_assert(hal::rtc::Clock<LinuxSystemClock>);
static_assert(hal::rtc::Alarm<LinuxSystemAlarm>);
static_assert(hal::rtc::Clock<LinuxHardwareClock>);
static_assert(hal::time::MonotonicClock<hal::linux::time::MonotonicClock>);
static_assert(hal::time::Delay<hal::linux::time::Delay>);
static_assert(hal::time::OneShotAlarm<hal::linux::time::OneShotAlarm>);
static_assert(hal::watchdog::Feeder<LinuxWatchdog>);

int main() {
  static_assert(std::is_nothrow_move_constructible_v<LinuxBlock>);
  static_assert(std::is_nothrow_move_constructible_v<LinuxFlash>);
  if (hal::linux::gpio_support.status !=
      hal::linux::implementation_status::available) {
    return 1;
  }

  // Exercise the construction paths against inert host descriptors. These
  // calls also force the target-specific template bodies through the build;
  // real device tests remain opt-in and environment-specific.
  (void)hal::linux::gpio::OutputLine::open({"/dev/null"});
  (void)hal::linux::gpio::InputLine::open({"/dev/null"});
  (void)hal::linux::gpio::EdgeLine::open(
      {"/dev/null"}, hal::gpio::edge::both);
  (void)hal::linux::i2c::Controller<>::open({"/dev/null"});
  (void)hal::linux::spi::Bus<>::open("/dev/null");
  (void)hal::linux::spi::Device<>::open({"/dev/null", {hal::hertz{1U}}});
  (void)hal::linux::can::Controller<>::open({"hal-no-such-can0"});
  (void)hal::linux::ethernet::TapFramePort<>::open({});
  (void)hal::linux::ethernet::PacketFramePort<>::open("lo");
  (void)hal::linux::block::Device::open({"/dev/null"});
  (void)hal::linux::nv::FileNorFlash<4096U, 1U, 256U>::open({"/dev/null", false});
  (void)hal::linux::adc::BufferedChannel<12U, 0, 3300000, 16U>::open(
      {"/dev/null", nullptr, nullptr, 0U, 1U, 0U, 1U, 8U, 0U, false, 1, 1U, 0});
  (void)hal::linux::pwm::Output::open({"/sys/class/pwm/pwmchip0", 0U, false});
  (void)hal::linux::rtc::HardwareClock<>::open("/dev/null");
  (void)hal::linux::watchdog::Feeder<60'000'000'000ULL>::open({"/dev/null", 1U});

  auto serial = hal::linux::serial::Port::open({"/dev/null"});
  if (serial) {
    (void)serial.value().configure({hal::hertz{115200U}});
  }

  hal::linux::time::MonotonicClock clock;
  const hal::instant before = clock.now();
  const hal::instant after = clock.now();
  if (after.nanoseconds_since_boot < before.nanoseconds_since_boot) {
    return 1;
  }

  auto alarm = hal::linux::time::OneShotAlarm::open();
  if (!alarm) {
    return 1;
  }
  const hal::instant deadline{clock.now().nanoseconds_since_boot + 5'000'000U};
  if (!alarm.value().arm(deadline)) {
    return 1;
  }
  ::pollfd descriptor{alarm.value().native_fd(), POLLIN, 0};
  if (::poll(&descriptor, 1U, 100) != 1 || !alarm.value().expired() ||
      alarm.value().expired()) {
    return 1;
  }
  hal::linux::rtc::SystemClock wall_clock;
  return wall_clock.read() ? 0 : 1;
}
