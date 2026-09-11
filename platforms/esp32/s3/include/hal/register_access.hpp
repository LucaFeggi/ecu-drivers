#pragma once

#include <atomic>
#include <cstdint>

namespace hal::esp32s3_wroom_1_n16r8::register_access {

[[nodiscard]] inline std::uint32_t read(
    volatile std::uint32_t& reg) noexcept {
  return reg;
}

inline void write(volatile std::uint32_t& reg, std::uint32_t value) noexcept {
  reg = value;
}

inline void set(volatile std::uint32_t& reg, std::uint32_t mask) noexcept {
  reg = reg | mask;
}

inline void clear(volatile std::uint32_t& reg, std::uint32_t mask) noexcept {
  reg = reg & ~mask;
}

inline void replace(volatile std::uint32_t& reg, std::uint32_t mask,
                    std::uint32_t value) noexcept {
  reg = (reg & ~mask) | (value & mask);
}

inline void compiler_barrier() noexcept {
  std::atomic_signal_fence(std::memory_order_seq_cst);
}

inline void device_write_barrier() noexcept {
#if defined(__XTENSA__)
  asm volatile("memw" ::: "memory");
#elif defined(__riscv)
  asm volatile("fence iorw, iorw" ::: "memory");
#else
  std::atomic_thread_fence(std::memory_order_seq_cst);
#endif
}

struct block {
  volatile std::uint32_t* base{};

  [[nodiscard]] inline volatile std::uint32_t& at(
      std::uint32_t byte_offset) const noexcept {
    return base[byte_offset / sizeof(std::uint32_t)];
  }

  [[nodiscard]] inline std::uint32_t read(
      std::uint32_t byte_offset) const noexcept {
    return register_access::read(at(byte_offset));
  }

  inline void write(std::uint32_t byte_offset, std::uint32_t value) const
      noexcept {
    register_access::write(at(byte_offset), value);
  }

  inline void set(std::uint32_t byte_offset, std::uint32_t mask) const noexcept {
    register_access::set(at(byte_offset), mask);
  }

  inline void clear(std::uint32_t byte_offset,
                    std::uint32_t mask) const noexcept {
    register_access::clear(at(byte_offset), mask);
  }
};

}  // namespace hal::esp32s3_wroom_1_n16r8::register_access
