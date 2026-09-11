#pragma once

#include <concepts>
#include <cstddef>

namespace hal {

// Use only when the BSP places the supplied storage in memory that is
// hardware-coherent or MPU-configured non-cacheable. Cacheable targets should
// provide a policy with cache-line-aware clean/invalidate implementations.
struct coherent_dma_policy {
  [[nodiscard]] constexpr bool valid_region(const void*,
                                            std::size_t) const noexcept {
    return true;
  }
  constexpr void prepare_for_device(const void*, std::size_t) const noexcept {}
  constexpr void prepare_for_cpu(const void*, std::size_t) const noexcept {}
};

template <class T>
concept DmaCoherencyPolicy = requires(const T& policy, const void* memory,
                                      std::size_t size) {
  { policy.valid_region(memory, size) } noexcept -> std::same_as<bool>;
  { policy.prepare_for_device(memory, size) } noexcept -> std::same_as<void>;
  { policy.prepare_for_cpu(memory, size) } noexcept -> std::same_as<void>;
};

// Caller-owned, statically sized DMA storage with target-selected alignment.
template <class T, std::size_t Count, std::size_t Alignment>
struct alignas(Alignment) dma_storage {
  static_assert(Count > 0U);
  static_assert(Alignment >= alignof(T));
  static_assert((Alignment & (Alignment - 1U)) == 0U);

  T values[Count]{};
};

}  // namespace hal
