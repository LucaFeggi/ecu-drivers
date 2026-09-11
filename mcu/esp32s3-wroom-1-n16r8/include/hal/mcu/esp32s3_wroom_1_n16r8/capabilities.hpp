#pragma once

#include <cstdint>
#include <hal/support.hpp>

namespace hal::esp32s3_wroom_1_n16r8::device {

inline constexpr std::uint8_t i2c_controller_count = 2U;
inline constexpr std::uint8_t spi_controller_count = 2U;
inline constexpr std::uint8_t uart_count = 3U;
inline constexpr std::uint8_t twai_controller_count = 1U;
inline constexpr std::uint8_t adc_unit_count = 2U;
inline constexpr std::uint8_t ledc_timer_count = 4U;
inline constexpr std::uint8_t ledc_channel_count = 8U;
inline constexpr std::uint8_t sdmmc_host_count = 1U;

inline constexpr capability native_ethernet{
    implementation_status::unsupported,
    "ESP32-S3 has no integrated Ethernet MAC; use a device driver for an "
    "external controller"};
inline constexpr capability native_can_fd{
    implementation_status::unsupported,
    "ESP32-S3 TWAI is classic CAN only; use an external CAN-FD controller"};
inline constexpr capability spi_dma{implementation_status::available,
                                    "GDMA-backed GPSPI2/GPSPI3"};
inline constexpr capability uart_dma{implementation_status::available,
                                     "GDMA-backed UART TX; FIFO-safe RX"};
inline constexpr capability adc_dma{implementation_status::available,
                                    "ADC digital controller and GDMA"};
inline constexpr capability sdmmc_dma{implementation_status::available,
                                      "SDMMC internal descriptor DMA"};

}  // namespace hal::esp32s3_wroom_1_n16r8::device
