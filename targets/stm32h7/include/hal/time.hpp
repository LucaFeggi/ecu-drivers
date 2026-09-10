#pragma once

#include <atomic>
#include <cstdint>
#include <hal/stm32h7/support.hpp>
#include <hal/time.hpp>

namespace hal::stm32h7::time {
inline constexpr capability support{
    implementation_status::available,
    "free-running 32-bit timer extended by its overflow ISR"};

template <std::uint32_t TimerClockHz, std::uint32_t TickHz,
          class TimerRegisters>
class TimerClock {
  static_assert(TimerClockHz > 0U);
  static_assert(TickHz > 0U);
  static_assert((TimerClockHz % TickHz) == 0U);
  static_assert((TimerClockHz / TickHz) <= 65'536U);
  static_assert(std::atomic<std::uint32_t>::is_always_lock_free);

public:
  explicit TimerClock(TimerRegisters &timer) noexcept : timer_{timer} {}

  TimerClock(const TimerClock &) = delete;
  TimerClock &operator=(const TimerClock &) = delete;

  void initialize() noexcept {
    timer_.CR1 = 0U;
    timer_.DIER = 0U;
    timer_.PSC = (TimerClockHz / TickHz) - 1U;
    timer_.ARR = 0xFFFF'FFFFU;
    timer_.CNT = 0U;
    timer_.EGR = update_generation;
    timer_.SR = 0U;
    wraps_.store(0U, std::memory_order_relaxed);
    timer_.DIER = update_interrupt_enable;
    timer_.CR1 = counter_enable;
  }

  void on_interrupt() noexcept {
    if ((timer_.SR & update_flag) != 0U) {
      timer_.SR = timer_.SR & ~update_flag;
      wraps_.fetch_add(1U, std::memory_order_release);
    }
  }

  [[nodiscard]] instant now() noexcept {
    for (;;) {
      const std::uint32_t before = wraps_.load(std::memory_order_acquire);
      std::uint32_t counter = timer_.CNT;
      const bool overflow_pending = (timer_.SR & update_flag) != 0U;
      const std::uint32_t after = wraps_.load(std::memory_order_acquire);
      if (before != after) {
        continue;
      }
      std::uint64_t ticks =
          (static_cast<std::uint64_t>(before) << 32U) | counter;
      if (overflow_pending) {
        counter = timer_.CNT;
        ticks = (static_cast<std::uint64_t>(before + 1U) << 32U) | counter;
      }
      const std::uint64_t seconds = ticks / TickHz;
      const std::uint64_t remainder = ticks % TickHz;
      return instant{seconds * 1'000'000'000ULL +
                     (remainder * 1'000'000'000ULL) / TickHz};
    }
  }

  void delay_for(nanoseconds duration) noexcept {
    const std::uint64_t ticks =
        (duration.value / 1'000'000'000ULL) * TickHz +
        ((duration.value % 1'000'000'000ULL) * TickHz + 999'999'999ULL) /
            1'000'000'000ULL;
    std::uint64_t remaining = ticks;
    while (remaining > 0U) {
      const std::uint32_t chunk = remaining > 0x7FFF'FFFFULL
                                      ? 0x7FFF'FFFFU
                                      : static_cast<std::uint32_t>(remaining);
      const std::uint32_t start = timer_.CNT;
      while (static_cast<std::uint32_t>(timer_.CNT - start) < chunk) {
      }
      remaining -= chunk;
    }
  }

private:
  static constexpr std::uint32_t counter_enable{1U << 0U};
  static constexpr std::uint32_t update_interrupt_enable{1U << 0U};
  static constexpr std::uint32_t update_flag{1U << 0U};
  static constexpr std::uint32_t update_generation{1U << 0U};

  TimerRegisters &timer_;
  std::atomic<std::uint32_t> wraps_{};
};

template <class Clock> class MicrosecondDelay {
public:
  explicit MicrosecondDelay(Clock &clock) noexcept : clock_{clock} {}
  void operator()(std::uint32_t microseconds) noexcept {
    clock_.delay_for(
        nanoseconds{static_cast<std::uint64_t>(microseconds) * 1'000ULL});
  }

private:
  Clock &clock_;
};

} // namespace hal::stm32h7::time
