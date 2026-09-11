#pragma once

#include <cstddef>

#if defined(__XTENSA__)
#define ECU_ESP32S3_IRAM __attribute__((section(".iram1")))
#define ECU_ESP32S3_DRAM __attribute__((section(".dram0.data")))
#else
#define ECU_ESP32S3_IRAM
#define ECU_ESP32S3_DRAM
#endif

namespace hal::esp32s3_wroom_1_n16r8::memory {

// These are target-level storage helpers for DMA/cache and flash-execution
// policy. They are not exact module identity or memory-capacity facts.
template <class T>
struct alignas(4) dma_object {
  T value{};
};

template <class T, std::size_t N>
struct alignas(4) dma_array {
  T values[N]{};
};

}  // namespace hal::esp32s3_wroom_1_n16r8::memory
