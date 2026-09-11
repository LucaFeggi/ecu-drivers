#pragma once

#include <cstdint>
#include <hal/support.hpp>
#include <string_view>

namespace hal::esp32s3_wroom_1_n16r8::device {

inline constexpr std::string_view name{"ESP32-S3-WROOM-1-N16R8"};

struct capabilities {
  static constexpr std::uint32_t flash_bytes{16U * 1024U * 1024U};
  static constexpr std::uint32_t psram_bytes{8U * 1024U * 1024U};
  static constexpr std::uint8_t gpio_count{49U};
  static constexpr std::uint8_t i2c_instances{2U};
  static constexpr std::uint8_t spi_instances{2U};
  static constexpr std::uint8_t uart_instances{3U};
  static constexpr std::uint8_t can_instances{1U};
  static constexpr std::uint8_t adc_instances{2U};
  static constexpr std::uint8_t ledc_timer_count{4U};
  static constexpr std::uint8_t ledc_channel_count{8U};
  static constexpr std::uint8_t sdmmc_instances{1U};

  static constexpr capability native_ethernet{
      implementation_status::unsupported,
      "ESP32-S3 has no integrated Ethernet MAC; use a device driver for an "
      "external controller"};
  static constexpr capability native_can_fd{
      implementation_status::unsupported,
      "ESP32-S3 TWAI is classic CAN only; use an external CAN-FD controller"};
  static constexpr capability spi_dma{implementation_status::available,
                                      "GDMA-backed GPSPI2/GPSPI3"};
  static constexpr capability uart_dma{implementation_status::available,
                                       "GDMA-backed UART TX; FIFO-safe RX"};
  static constexpr capability adc_dma{implementation_status::available,
                                      "ADC digital controller and GDMA"};
  static constexpr capability sdmmc_dma{implementation_status::available,
                                        "SDMMC internal descriptor DMA"};
};

}  // namespace hal::esp32s3_wroom_1_n16r8::device
