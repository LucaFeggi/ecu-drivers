#pragma once

#include <cstdint>
#include <hal/foundation/result.hpp>
#include <hal/foundation/units.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/watchdog.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif
#include <soc/timer_group_struct.h>

namespace hal::esp32s3_wroom_1_n16r8::watchdog {

class error {
 public:
  [[nodiscard]] static constexpr error hardware_fault() noexcept {
    return error{hal::watchdog::error_kind::hardware_fault};
  }
  [[nodiscard]] constexpr hal::watchdog::error_kind kind() const noexcept {
    return kind_;
  }

 private:
  constexpr explicit error(hal::watchdog::error_kind kind) noexcept : kind_{kind} {}
  hal::watchdog::error_kind kind_;
};

template <std::uint32_t SourceHz = 80'000'000U,
          std::uint64_t TimeoutNs = 2'000'000'000ULL>
class Feeder {
  static_assert(SourceHz != 0U && TimeoutNs != 0ULL);

  static constexpr std::uint32_t selected_prescaler = []() constexpr {
    const std::uint64_t minimum =
        (TimeoutNs * static_cast<std::uint64_t>(SourceHz) +
         1'000'000'000ULL * 0xFFFFULL - 1ULL) /
        (1'000'000'000ULL * 0xFFFFULL);
    return minimum == 0ULL ? 1U : static_cast<std::uint32_t>(minimum);
  }();
  static constexpr std::uint64_t hold_cycles =
      (TimeoutNs * static_cast<std::uint64_t>(SourceHz)) /
      (1'000'000'000ULL * selected_prescaler);

  static_assert(selected_prescaler <= 0xFFFFU && hold_cycles != 0ULL &&
                hold_cycles <= 0xFFFF'FFFFULL);

 public:
  using error_type = error;

  explicit Feeder(timg_dev_t& peripheral) noexcept : peripheral_{&peripheral} {}

  Feeder(const Feeder&) = delete;
  Feeder& operator=(const Feeder&) = delete;

  [[nodiscard]] static constexpr hal::watchdog::characteristics properties() noexcept {
    return {nanoseconds{0U}, nanoseconds{TimeoutNs}, false};
  }

  [[nodiscard]] result<void, error_type> start() noexcept {
    unlock();
    peripheral_->wdtconfig0.wdt_flashboot_mod_en = 0U;
    peripheral_->wdtconfig0.wdt_appcpu_reset_en = 0U;
    peripheral_->wdtconfig0.wdt_procpu_reset_en = 1U;
    peripheral_->wdtconfig0.wdt_sys_reset_length = 3U;
    peripheral_->wdtconfig0.wdt_cpu_reset_length = 3U;
    peripheral_->wdtconfig0.wdt_stg0 = 3U;  // reset system
    peripheral_->wdtconfig0.wdt_stg1 = 0U;
    peripheral_->wdtconfig0.wdt_stg2 = 0U;
    peripheral_->wdtconfig0.wdt_stg3 = 0U;
    peripheral_->wdtconfig1.wdt_clk_prescale = selected_prescaler;
    peripheral_->wdtconfig2.wdt_stg0_hold = static_cast<std::uint32_t>(hold_cycles);
    peripheral_->wdtconfig0.wdt_en = 1U;
    feed_unlocked();
    lock();
    enabled_ = true;
    return result<void, error_type>::success();
  }

  [[nodiscard]] result<void, error_type> stop() noexcept {
    unlock();
    peripheral_->wdtconfig0.wdt_en = 0U;
    lock();
    enabled_ = false;
    return result<void, error_type>::success();
  }

  [[nodiscard]] result<void, error_type> feed() noexcept {
    if (!enabled_) {
      return result<void, error_type>::failure(error_type::hardware_fault());
    }
    unlock();
    feed_unlocked();
    lock();
    return result<void, error_type>::success();
  }

  [[nodiscard]] static constexpr std::uint32_t prescaler() noexcept {
    return selected_prescaler;
  }

  [[nodiscard]] static constexpr std::uint32_t timeout_cycles() noexcept {
    return static_cast<std::uint32_t>(hold_cycles);
  }

 private:
  void unlock() noexcept { peripheral_->wdtwprotect.wdt_wkey = 0x50D83AA1U; }
  void lock() noexcept { peripheral_->wdtwprotect.wdt_wkey = 0U; }
  void feed_unlocked() noexcept { peripheral_->wdtfeed.wdt_feed = 1U; }

  timg_dev_t* peripheral_;
  bool enabled_{};
};

}  // namespace hal::esp32s3_wroom_1_n16r8::watchdog
