#pragma once

#include <cstddef>
#include <cstdint>
#include <hal/foundation/result.hpp>
#include <hal/foundation/units.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/pwm.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif
#include <soc/ledc_struct.h>

namespace hal::esp32s3_wroom_1_n16r8::pwm {

class error {
 public:
  [[nodiscard]] static constexpr error unrepresentable() noexcept {
    return error{hal::pwm::error_kind::unrepresentable};
  }
  [[nodiscard]] static constexpr error not_configured() noexcept {
    return error{hal::pwm::error_kind::not_configured};
  }
  [[nodiscard]] static constexpr error io() noexcept {
    return error{hal::pwm::error_kind::io};
  }
  [[nodiscard]] constexpr hal::pwm::error_kind kind() const noexcept {
    return kind_;
  }

 private:
  constexpr explicit error(hal::pwm::error_kind kind) noexcept : kind_{kind} {}
  hal::pwm::error_kind kind_;
};

struct timer_solution {
  std::uint32_t divider{};       // Q8 fixed-point LEDC divider.
  std::uint8_t resolution{};    // 1..14 timer bits.
  std::uint64_t period_ns{};
};

template <std::size_t Timer, std::size_t Channel,
          std::uint32_t SourceHz = 80'000'000U>
class Output {
  static_assert(Timer < 4U && Channel < 8U);

 public:
  using error_type = error;

  explicit Output(ledc_dev_t& peripheral) noexcept : peripheral_{&peripheral} {}

  Output(const Output&) = delete;
  Output& operator=(const Output&) = delete;

  [[nodiscard]] result<nanoseconds, error_type> configure(
      hal::pwm::config configuration) noexcept {
    if (configuration.requested_period.value == 0U || SourceHz == 0U) {
      return result<nanoseconds, error_type>::failure(error_type::unrepresentable());
    }
    const timer_solution solution = find_solution(configuration.requested_period);
    if (solution.divider == 0U) {
      return result<nanoseconds, error_type>::failure(error_type::unrepresentable());
    }
    if (configuration.max_error_ppm != 0U) {
      const std::uint64_t difference =
          solution.period_ns > configuration.requested_period.value
              ? solution.period_ns - configuration.requested_period.value
              : configuration.requested_period.value - solution.period_ns;
      if (difference * 1'000'000ULL /
              configuration.requested_period.value >
          configuration.max_error_ppm) {
        return result<nanoseconds, error_type>::failure(error_type::unrepresentable());
      }
    }

    auto& timer = peripheral_->timer_group[0].timer[Timer];
    auto& channel = peripheral_->channel_group[0].channel[Channel];
    timer.conf.rst = 1U;
    timer.conf.rst = 0U;
    timer.conf.pause = 1U;
    timer.conf.tick_sel = 0U;  // APB clock on ESP32-S3.
    timer.conf.val =
        (timer.conf.val & ~((0x0FU << 0U) | (0x3FFFFU << 4U))) |
        (static_cast<std::uint32_t>(solution.resolution) << 0U) |
        ((solution.divider & 0x3FFFFU) << 4U);
    timer.conf.low_speed_update = 1U;
    channel.conf0.timer_sel = Timer;
    channel.conf0.sig_out_en = 0U;
    channel.conf0.idle_lv =
        configuration.output_polarity == hal::pwm::polarity::active_low ? 1U : 0U;
    channel.conf1.duty_num = 0U;
    channel.conf1.duty_cycle = 0U;
    channel.conf1.duty_scale = 0U;
    solution_ = solution;
    configuration_ = configuration;
    configured_ = true;
    set_pulse_register(0U);
    timer.conf.pause = 0U;
    return result<nanoseconds, error_type>::success(nanoseconds{solution.period_ns});
  }

  [[nodiscard]] result<void, error_type> set_pulse_width(
      nanoseconds pulse) noexcept {
    if (!configured_) {
      return result<void, error_type>::failure(error_type::not_configured());
    }
    if (pulse.value > solution_.period_ns) {
      return result<void, error_type>::failure(error_type::unrepresentable());
    }
    const std::uint32_t max_duty = std::uint32_t{1U} << solution_.resolution;
    const std::uint32_t active_duty = static_cast<std::uint32_t>(
        (pulse.value * max_duty + solution_.period_ns / 2U) /
        solution_.period_ns);
    set_pulse_register(active_duty);
    return result<void, error_type>::success();
  }

  [[nodiscard]] result<void, error_type> enable_output() noexcept {
    if (!configured_) {
      return result<void, error_type>::failure(error_type::not_configured());
    }
    auto& channel = peripheral_->channel_group[0].channel[Channel];
    channel.conf0.sig_out_en = 1U;
    channel.conf1.duty_start = 1U;
    channel.conf0.low_speed_update = 1U;
    return result<void, error_type>::success();
  }

  [[nodiscard]] result<void, error_type> disable_output() noexcept {
    if (!configured_) {
      return result<void, error_type>::failure(error_type::not_configured());
    }
    peripheral_->channel_group[0].channel[Channel].conf0.sig_out_en = 0U;
    return result<void, error_type>::success();
  }

  [[nodiscard]] timer_solution solution() const noexcept { return solution_; }

 private:
  [[nodiscard]] static timer_solution find_solution(nanoseconds period) noexcept {
    timer_solution selected{};
    std::uint64_t best_error = UINT64_MAX;
    for (std::uint8_t resolution = 14U; resolution >= 1U; --resolution) {
      const std::uint64_t precision = std::uint64_t{1U} << resolution;
      const std::uint64_t denominator = 1'000'000'000ULL * precision;
      const std::uint64_t numerator =
          static_cast<std::uint64_t>(SourceHz) * 256ULL * period.value;
      const std::uint64_t divider = (numerator + denominator / 2U) / denominator;
      if (divider < 256U || divider > 0x3FFFFU) {
        continue;
      }
      const std::uint64_t actual_period =
          (denominator * divider + static_cast<std::uint64_t>(SourceHz) * 128ULL) /
          (static_cast<std::uint64_t>(SourceHz) * 256ULL);
      const std::uint64_t error = actual_period > period.value
                                      ? actual_period - period.value
                                      : period.value - actual_period;
      if (error < best_error) {
        best_error = error;
        selected = {static_cast<std::uint32_t>(divider), resolution, actual_period};
      }
    }
    return selected;
  }

  void set_pulse_register(std::uint32_t active_duty) noexcept {
    const std::uint32_t max_duty = std::uint32_t{1U} << solution_.resolution;
    const std::uint32_t duty =
        configuration_.output_polarity == hal::pwm::polarity::active_low
            ? max_duty - active_duty
            : active_duty;
    auto& channel = peripheral_->channel_group[0].channel[Channel];
    channel.duty.val = (channel.duty.val & ~0x0007'FFFFU) |
                       (duty & 0x0007'FFFFU);
    channel.conf1.duty_start = 1U;
    channel.conf0.low_speed_update = 1U;
  }

  ledc_dev_t* peripheral_;
  timer_solution solution_{};
  hal::pwm::config configuration_{};
  bool configured_{};
};

}  // namespace hal::esp32s3_wroom_1_n16r8::pwm
