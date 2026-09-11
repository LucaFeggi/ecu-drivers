#pragma once

#include <cstdint>

namespace hal::esp32s3_wroom_1_n16r8::device {

// Peripheral interrupt numbers are kept as policy values rather than hidden
// calls to esp_intr_alloc(). The firmware BSP owns routing to a CPU and must
// install the corresponding vector using its startup/interrupt policy.
enum class interrupt_source : std::uint8_t {
  gpio,
  i2c0,
  i2c1,
  spi2,
  spi3,
  uart0,
  uart1,
  uart2,
  twai,
  adc,
  ledc,
  systimer,
  sdmmc,
  gdma
};

}  // namespace hal::esp32s3_wroom_1_n16r8::device
