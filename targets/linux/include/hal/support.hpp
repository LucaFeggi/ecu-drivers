#pragma once

#include <cstddef>
#include <cstdint>

#ifdef linux
#undef linux
#endif

namespace hal::linux {

enum class implementation_status : std::uint8_t {
  available,
  optional,
  unavailable
};

struct capability {
  implementation_status status{};
  const char* description{};
};

inline constexpr std::size_t max_transaction_operations{8U};

inline constexpr capability gpio_support{
    implementation_status::available, "Linux GPIO character-device v2"};
inline constexpr capability i2c_support{
    implementation_status::available, "Linux i2c-dev combined transactions"};
inline constexpr capability spi_support{implementation_status::available,
                                        "Linux spidev composite transfers"};
inline constexpr capability can_support{implementation_status::available,
                                        "Linux SocketCAN raw sockets"};
inline constexpr capability ethernet_support{
    implementation_status::available, "Linux TAP and AF_PACKET frame ports"};
inline constexpr capability block_support{
    implementation_status::available, "pread/pwrite regular and block files"};
inline constexpr capability nv_support{
    implementation_status::available, "file-backed NOR flash semantics"};
inline constexpr capability serial_support{
    implementation_status::available, "termios nonblocking tty ports"};
inline constexpr capability adc_support{implementation_status::optional,
                                        "Linux buffered IIO"};
inline constexpr capability pwm_support{implementation_status::optional,
                                        "Linux PWM userspace interface"};
inline constexpr capability rtc_support{
    implementation_status::available, "CLOCK_REALTIME and /dev/rtc"};
inline constexpr capability time_support{
    implementation_status::available, "CLOCK_MONOTONIC and timerfd"};
inline constexpr capability watchdog_support{
    implementation_status::optional, "Linux watchdog device"};

}  // namespace hal::linux
