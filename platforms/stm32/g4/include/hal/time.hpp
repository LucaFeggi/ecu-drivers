#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/time.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include <atomic>
#include <cstdint>
#include "support.hpp"

namespace hal::stm32g4::time {

inline constexpr capability support{
    implementation_status::available,
    "free-running timer extended by its overflow interrupt"};

template <std::uint32_t TimerClockHz, std::uint32_t TickHz,
          class TimerRegisters, unsigned CounterBits = 32U>
class TimerClock {
  static_assert(TimerClockHz > 0U && TickHz > 0U);
  static_assert((TimerClockHz % TickHz) == 0U);
  static_assert((TimerClockHz / TickHz) <= 65'536U);
  static_assert(CounterBits == 16U || CounterBits == 32U);
  static_assert(std::atomic<std::uint32_t>::is_always_lock_free);

public:
  explicit TimerClock(TimerRegisters &timer) noexcept : timer_{timer} {}

  TimerClock(const TimerClock &) = delete;
  TimerClock &operator=(const TimerClock &) = delete;

  void initialize() noexcept {
    timer_.CR1 = 0U;
    timer_.DIER = 0U;
    timer_.PSC = (TimerClockHz / TickHz) - 1U;
    timer_.ARR = counter_max;
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

  [[nodiscard]] hal::instant now() noexcept {
    for (;;) {
      const std::uint32_t before = wraps_.load(std::memory_order_acquire);
      std::uint32_t counter = timer_.CNT;
      const bool pending = (timer_.SR & update_flag) != 0U;
      const std::uint32_t after = wraps_.load(std::memory_order_acquire);
      if (before != after) {
        continue;
      }
      std::uint64_t ticks = (static_cast<std::uint64_t>(before) << CounterBits) |
                            static_cast<std::uint64_t>(counter);
      if (pending) {
        counter = timer_.CNT;
        ticks = (static_cast<std::uint64_t>(before + 1U) << CounterBits) |
                (static_cast<std::uint64_t>(counter) & counter_mask);
      }
      const std::uint64_t seconds = ticks / TickHz;
      const std::uint64_t remainder = ticks % TickHz;
      return {seconds * 1'000'000'000ULL +
              (remainder * 1'000'000'000ULL) / TickHz};
    }
  }

  void delay_for(hal::nanoseconds duration) noexcept {
    const std::uint64_t ticks =
        (duration.value / 1'000'000'000ULL) * TickHz +
        ((duration.value % 1'000'000'000ULL) * TickHz + 999'999'999ULL) /
            1'000'000'000ULL;
    std::uint64_t remaining = ticks;
    while (remaining > 0U) {
      const std::uint32_t chunk = remaining > maximum_delay_ticks
                                      ? maximum_delay_ticks
                                      : static_cast<std::uint32_t>(remaining);
      const std::uint32_t start = timer_.CNT;
      while (((static_cast<std::uint32_t>(timer_.CNT) - start) &
              counter_mask) < chunk) {
      }
      remaining -= chunk;
    }
  }

private:
  static constexpr std::uint32_t counter_enable{1U};
  static constexpr std::uint32_t update_interrupt_enable{1U};
  static constexpr std::uint32_t update_flag{1U};
  static constexpr std::uint32_t update_generation{1U};
  static constexpr std::uint32_t counter_max =
      CounterBits == 16U ? 0xFFFFU : 0xFFFF'FFFFU;
  static constexpr std::uint32_t counter_mask = counter_max;
  static constexpr std::uint64_t maximum_delay_ticks =
      (static_cast<std::uint64_t>(counter_max) + 1U) / 2U - 1U;

  TimerRegisters &timer_;
  std::atomic<std::uint32_t> wraps_{};
};

template <class Clock> class MicrosecondDelay {
public:
  explicit MicrosecondDelay(Clock &clock) noexcept : clock_{clock} {}

  void operator()(std::uint32_t microseconds) noexcept {
    clock_.delay_for(
        hal::nanoseconds{static_cast<std::uint64_t>(microseconds) * 1'000ULL});
  }

private:
  Clock &clock_;
};

} // namespace hal::stm32g4::time
