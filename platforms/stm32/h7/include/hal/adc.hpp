#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/adc.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <hal/foundation/assert.hpp>
#include "support.hpp"

namespace hal::stm32h7::adc {

inline constexpr capability publication_support{
    implementation_status::available,
    "bounded ISR-to-task continuous sample publication"};

inline constexpr capability acquisition_support{
    implementation_status::available,
    "ADC1/2 external-trigger circular-DMA acquisition engine"};

class error {
public:
  explicit constexpr error(hal::adc::error_kind kind) noexcept : kind_{kind} {}
  [[nodiscard]] constexpr hal::adc::error_kind kind() const noexcept {
    return kind_;
  }

private:
  hal::adc::error_kind kind_{};
};

template <hal::adc::characteristics Characteristics, std::size_t Capacity>
class ContinuousChannel {
  static_assert(Capacity > 1U);
  static_assert(std::atomic<std::uint32_t>::is_always_lock_free);

public:
  using error_type = error;

  [[nodiscard]] static constexpr hal::adc::characteristics
  properties() noexcept {
    return Characteristics;
  }

  ContinuousChannel() noexcept = default;
  ContinuousChannel(const ContinuousChannel &) = delete;
  ContinuousChannel &operator=(const ContinuousChannel &) = delete;

  // One acquisition ISR/DMA-completion producer publishes samples. Any task
  // may read the latest value; try_read() has one task-side consumer cursor
  // and must be externally serialized if that task can be re-entered.
  void publish_from_isr(std::uint32_t raw, instant captured_at) noexcept {
    const std::uint64_t sequence = produced_by_isr_ + 1U;
    produced_by_isr_ = sequence;
    generation_.fetch_add(1U, std::memory_order_acq_rel);
    slot &destination = samples_[sequence % Capacity];
    destination.raw.store(raw, std::memory_order_relaxed);
    destination.timestamp_low.store(
        static_cast<std::uint32_t>(captured_at.nanoseconds_since_boot),
        std::memory_order_relaxed);
    destination.timestamp_high.store(
        static_cast<std::uint32_t>(captured_at.nanoseconds_since_boot >> 32U),
        std::memory_order_relaxed);
    destination.sequence_low.store(static_cast<std::uint32_t>(sequence),
                                   std::memory_order_relaxed);
    destination.sequence_high.store(static_cast<std::uint32_t>(sequence >> 32U),
                                    std::memory_order_relaxed);
    published_low_.store(static_cast<std::uint32_t>(sequence),
                         std::memory_order_relaxed);
    published_high_.store(static_cast<std::uint32_t>(sequence >> 32U),
                          std::memory_order_relaxed);
    generation_.fetch_add(1U, std::memory_order_release);
  }

  [[nodiscard]] auto latest() -> result<hal::adc::raw_sample, error_type> {
    std::uint64_t produced{};
    hal::adc::raw_sample value{};
    if (!snapshot_latest(produced, value)) {
      return result<hal::adc::raw_sample, error_type>::failure(
          error_type{hal::adc::error_kind::overrun});
    }
    if (produced == 0U) {
      return result<hal::adc::raw_sample, error_type>::failure(
          error_type{hal::adc::error_kind::not_ready});
    }
    return result<hal::adc::raw_sample, error_type>::success(value);
  }

  [[nodiscard]] auto read_raw()
      -> result<hal::adc::raw_sample, error_type> {
    return latest();
  }

  [[nodiscard]] auto read_voltage()
      -> result<hal::adc::voltage_sample, error_type> {
    const auto raw = read_raw();
    if (!raw) {
      return result<hal::adc::voltage_sample, error_type>::failure(raw.error());
    }
    constexpr std::uint64_t code_count =
        Characteristics.resolution_bits >= 32U
            ? std::numeric_limits<std::uint32_t>::max()
            : (std::uint64_t{1U} << Characteristics.resolution_bits) - 1U;
    if (code_count == 0U || Characteristics.nominal_max.value <
                                 Characteristics.nominal_min.value) {
      return result<hal::adc::voltage_sample, error_type>::failure(
          error_type{hal::adc::error_kind::configuration});
    }
    const std::int64_t span_uv = Characteristics.nominal_max.value -
                                 Characteristics.nominal_min.value;
    const std::int64_t offset = static_cast<std::int64_t>(
        (static_cast<std::uint64_t>(raw.value().value) *
         static_cast<std::uint64_t>(span_uv)) /
        code_count);
    return result<hal::adc::voltage_sample, error_type>::success(
        {hal::microvolts{Characteristics.nominal_min.value + offset},
         raw.value().captured_at, raw.value().sequence});
  }

  [[nodiscard]] auto try_read(hal::span<hal::adc::raw_sample> output)
      -> result<hal::adc::stream_read, error_type> {
    HAL_CORE_ASSERT(output.valid());
    if (!output.valid()) {
      return result<hal::adc::stream_read, error_type>::failure(
          error_type{hal::adc::error_kind::other});
    }

    for (unsigned attempt = 0U; attempt < 3U; ++attempt) {
      const std::uint32_t before = generation_.load(std::memory_order_acquire);
      if ((before & 1U) != 0U) {
        continue;
      }

      const std::uint64_t produced = published_sequence();
      std::uint64_t next = consumed_ + 1U;
      std::uint64_t dropped = 0U;
      if (produced >= next && produced - next >= Capacity) {
        const std::uint64_t oldest = produced - Capacity + 1U;
        dropped = oldest - next;
        next = oldest;
      }

      std::size_t count = 0U;
      while (next <= produced && count < output.size()) {
        output[count] = load(samples_[next % Capacity]);
        ++next;
        ++count;
      }

      const std::uint32_t after = generation_.load(std::memory_order_acquire);
      if (before == after) {
        consumed_ = next - 1U;
        return result<hal::adc::stream_read, error_type>::success(
            {count, dropped});
      }
    }
    return result<hal::adc::stream_read, error_type>::failure(
        error_type{hal::adc::error_kind::overrun});
  }

private:
  struct slot {
    std::atomic<std::uint32_t> raw{};
    std::atomic<std::uint32_t> timestamp_low{};
    std::atomic<std::uint32_t> timestamp_high{};
    std::atomic<std::uint32_t> sequence_low{};
    std::atomic<std::uint32_t> sequence_high{};
  };

  [[nodiscard]] static hal::adc::raw_sample load(const slot &source) noexcept {
    const auto timestamp =
        (static_cast<std::uint64_t>(
             source.timestamp_high.load(std::memory_order_relaxed))
         << 32U) |
        source.timestamp_low.load(std::memory_order_relaxed);
    const auto sequence = (static_cast<std::uint64_t>(source.sequence_high.load(
                               std::memory_order_relaxed))
                           << 32U) |
                          source.sequence_low.load(std::memory_order_relaxed);
    return {source.raw.load(std::memory_order_relaxed), instant{timestamp},
            sequence};
  }

  [[nodiscard]] std::uint64_t published_sequence() const noexcept {
    return (static_cast<std::uint64_t>(
                published_high_.load(std::memory_order_relaxed))
            << 32U) |
           published_low_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] bool snapshot_latest(std::uint64_t &produced,
                                     hal::adc::raw_sample &value) const noexcept {
    for (unsigned attempt = 0U; attempt < 3U; ++attempt) {
      const std::uint32_t before = generation_.load(std::memory_order_acquire);
      if ((before & 1U) != 0U) {
        continue;
      }
      produced = published_sequence();
      if (produced != 0U) {
        value = load(samples_[produced % Capacity]);
      }
      const std::uint32_t after = generation_.load(std::memory_order_acquire);
      if (before == after) {
        return true;
      }
    }
    return false;
  }

  std::array<slot, Capacity> samples_{};
  std::atomic<std::uint32_t> generation_{};
  std::atomic<std::uint32_t> published_low_{};
  std::atomic<std::uint32_t> published_high_{};
  std::uint64_t produced_by_isr_{};
  std::uint64_t consumed_{};
};

enum class asynchronous_prescaler : std::uint8_t {
  divide_by_1 = 0U,
  divide_by_2 = 1U,
  divide_by_4 = 2U,
  divide_by_6 = 3U,
  divide_by_8 = 4U,
  divide_by_10 = 5U,
  divide_by_12 = 6U,
  divide_by_16 = 7U,
  divide_by_32 = 8U,
  divide_by_64 = 9U,
  divide_by_128 = 10U,
  divide_by_256 = 11U
};

// Runtime selections owned by a BSP. The acquisition engine itself remains
// board-independent: it only receives already selected register instances and
// the operating point selected by the BSP.
struct dma_config {
  std::uint32_t adc_input_clock_hz{};
  asynchronous_prescaler adc_prescaler{asynchronous_prescaler::divide_by_1};
  std::uint8_t channel{};
  std::uint8_t sample_time_code{};
  std::uint16_t oversampling_ratio{1U};
  std::uint8_t oversampling_shift{};
  std::uint8_t external_trigger{};
  std::uint8_t dmamux_request{};
  std::uint16_t timer_prescaler{};
  std::uint32_t timer_auto_reload{};
  std::uint64_t sample_period_ns{};
  poll_budget timeout{};
  // ADC1/2 on STM32H723 support 10/12/14/16-bit native conversion
  // resolution. Hardware oversampling may produce a wider published result.
  std::uint8_t adc_resolution_bits{16U};

  [[nodiscard]] constexpr bool valid() const noexcept {
    const std::uint32_t analog_clock = analog_clock_hz();
    return adc_input_clock_hz > 0U && prescaler_divisor() != 0U &&
           analog_clock > 0U && analog_clock <= 50'000'000U && channel < 20U &&
           sample_time_code < 8U && oversampling_ratio > 0U &&
           oversampling_ratio <= 1024U && oversampling_shift < 12U &&
           external_trigger < 32U && sample_period_ns > 0U && timeout.valid() &&
           (adc_resolution_bits == 10U || adc_resolution_bits == 12U ||
            adc_resolution_bits == 14U || adc_resolution_bits == 16U);
  }

  [[nodiscard]] constexpr std::uint32_t prescaler_divisor() const noexcept {
    constexpr std::array<std::uint16_t, 12U> divisors{
        1U, 2U, 4U, 6U, 8U, 10U, 12U, 16U, 32U, 64U, 128U, 256U};
    const auto code = static_cast<std::size_t>(adc_prescaler);
    return code < divisors.size() ? divisors[code] : 0U;
  }

  // RM0468 applies a fixed divide-by-two between the ADC1/2 kernel-clock
  // input/prescaler output and the analog converter block.
  [[nodiscard]] constexpr std::uint32_t analog_clock_hz() const noexcept {
    const std::uint32_t divisor = prescaler_divisor();
    return divisor == 0U ? 0U : adc_input_clock_hz / divisor / 2U;
  }
};

enum class acquisition_fault : std::uint8_t {
  none,
  invalid_configuration,
  adc_busy,
  adc_regulator_timeout,
  adc_calibration_timeout,
  adc_ready_timeout,
  adc_stop_timeout,
  dma_stop_timeout,
  dma_transfer,
  dma_direct_mode,
  dma_fifo,
  adc_overrun
};

// ADC1/ADC2 + DMAMUX1 + DMA1/2 stream 0 + basic-timer acquisition.
//
// Register types are template parameters so the reusable driver does not
// include a part-specific CMSIS header. A device profile supplies the concrete
// register blocks. The DMA buffer must be DMA-accessible and coherent; an H7
// BSP normally places it in an MPU non-cacheable SRAM section.
template <hal::adc::characteristics Characteristics, std::size_t QueueCapacity,
          std::size_t DmaSampleCount, class AdcRegisters,
          class AdcCommonRegisters, class DmaRegisters,
          class DmaStreamRegisters, class DmamuxChannelRegisters,
          class TimerRegisters>
class Adc12DmaStream0 {
  static_assert(DmaSampleCount >= 2U);
  static_assert((DmaSampleCount % 2U) == 0U);
  static_assert(DmaSampleCount <= 65'535U);

public:
  using error_type = error;

  [[nodiscard]] static constexpr hal::adc::characteristics
  properties() noexcept {
    return Characteristics;
  }

  Adc12DmaStream0(AdcRegisters &adc, AdcCommonRegisters &common,
                  DmaRegisters &dma, DmaStreamRegisters &stream,
                  DmamuxChannelRegisters &dmamux, TimerRegisters &timer,
                  volatile std::uint32_t (&buffer)[DmaSampleCount],
                  dma_config configuration) noexcept
      : adc_{adc}, common_{common}, dma_{dma}, stream_{stream}, dmamux_{dmamux},
        timer_{timer}, buffer_{buffer}, configuration_{configuration} {}

  Adc12DmaStream0(const Adc12DmaStream0 &) = delete;
  Adc12DmaStream0 &operator=(const Adc12DmaStream0 &) = delete;

  // delay_us is supplied by the BSP because regulator and post-calibration
  // delays are clock-policy concerns. It must be callable as
  // delay_us(uint32_t).
  template <class MicrosecondDelay>
  [[nodiscard]] auto initialize(MicrosecondDelay &delay_us)
      -> result<void, error_type> {
    if (!configuration_.valid() || !oversampling_matches_characteristics()) {
      return fail(acquisition_fault::invalid_configuration,
                  hal::adc::error_kind::configuration);
    }
    if ((adc_.CR & (cr_aden | cr_adstart | cr_adcal)) != 0U) {
      return fail(acquisition_fault::adc_busy, hal::adc::error_kind::io);
    }
    if (!disable_dma()) {
      return fail(acquisition_fault::dma_stop_timeout,
                  hal::adc::error_kind::timeout);
    }

    timer_.CR1 = 0U;
    timer_.DIER = 0U;
    timer_.CR2 = (timer_.CR2 & ~tim_cr2_mms_mask) | tim_cr2_mms_update;
    timer_.PSC = configuration_.timer_prescaler;
    timer_.ARR = configuration_.timer_auto_reload;
    timer_.CNT = 0U;
    timer_.EGR = tim_egr_ug;
    timer_.SR = 0U;

    clear_dma_flags();
    dmamux_.CCR = configuration_.dmamux_request;
    stream_.PAR = pointer_word(&adc_.DR);
    stream_.M0AR = pointer_word(buffer_);
    stream_.NDTR = static_cast<std::uint32_t>(DmaSampleCount);
    stream_.FCR = 0U;
    stream_.CR =
        dma_cr_priority_high | dma_cr_memory_word | dma_cr_peripheral_halfword |
        dma_cr_memory_increment | dma_cr_circular |
        dma_cr_transfer_complete_interrupt | dma_cr_half_transfer_interrupt |
        dma_cr_transfer_error_interrupt | dma_cr_direct_mode_error_interrupt;

    adc_.CR = adc_.CR & ~cr_deep_power_down;
    adc_.CR = adc_.CR | cr_voltage_regulator;
    delay_us(10U);
    if (!wait_set(adc_.ISR, isr_ldo_ready)) {
      return fail(acquisition_fault::adc_regulator_timeout,
                  hal::adc::error_kind::timeout);
    }

    // In the STM32H723 CMSIS layout these ADC1/2 registers overlap ADC3
    // registers and are therefore exposed as DIFSEL_RES12 and PCSEL_RES0.
    adc_.DIFSEL_RES12 =
        adc_.DIFSEL_RES12 & ~(std::uint32_t{1U} << configuration_.channel);
    common_.CCR = (common_.CCR & ~(ccr_clock_mode_mask | ccr_prescaler_mask)) |
                  (static_cast<std::uint32_t>(configuration_.adc_prescaler)
                   << ccr_prescaler_position);

    adc_.CR = (adc_.CR & ~(cr_calibration_differential | cr_boost_mask)) |
              cr_linearity_calibration | boost_code();
    adc_.CR = adc_.CR | cr_start_calibration;
    if (!wait_clear(adc_.CR, cr_start_calibration)) {
      return fail(acquisition_fault::adc_calibration_timeout,
                  hal::adc::error_kind::calibration);
    }
    // RM0468 forbids ADEN during the first four analog ADC clock cycles after
    // calibration. Derive the delay from the selected operating point so a
    // slower BSP configuration cannot violate that requirement.
    delay_us(post_calibration_delay_us());

    adc_.ISR = isr_ready;
    adc_.CR = adc_.CR | cr_aden;
    if (!wait_set(adc_.ISR, isr_ready)) {
      return fail(acquisition_fault::adc_ready_timeout,
                  hal::adc::error_kind::timeout);
    }

    const std::uint32_t channel_mask = std::uint32_t{1U}
                                       << configuration_.channel;
    replace_sample_time();
    adc_.PCSEL_RES0 = channel_mask;
    adc_.SQR1 = (static_cast<std::uint32_t>(configuration_.channel)
                 << sqr1_first_channel_position);
    adc_.CFGR = cfgr_dma_circular | resolution_code() |
                cfgr_overwrite_on_overrun |
                (static_cast<std::uint32_t>(configuration_.external_trigger)
                 << cfgr_external_trigger_position) |
                cfgr_external_trigger_rising;
    configure_oversampling();
    adc_.IER = 0U;
    fault_.store(static_cast<std::uint32_t>(acquisition_fault::none),
                 std::memory_order_release);
    initialized_ = true;
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto start(instant epoch) -> result<void, error_type> {
    if (!initialized_) {
      return fail(acquisition_fault::invalid_configuration,
                  hal::adc::error_kind::not_ready);
    }
    if (running_) {
      return result<void, error_type>::success();
    }

    clear_dma_flags();
    fault_.store(static_cast<std::uint32_t>(acquisition_fault::none),
                 std::memory_order_release);
    pending_halves_.store(0U, std::memory_order_release);
    first_half_generation_.store(0U, std::memory_order_release);
    second_half_generation_.store(0U, std::memory_order_release);
    stream_.NDTR = static_cast<std::uint32_t>(DmaSampleCount);
    next_timestamp_ns_ =
        epoch.nanoseconds_since_boot + configuration_.sample_period_ns;
    stream_.CR = stream_.CR | dma_cr_enable;
    adc_.ISR = isr_overrun;
    adc_.CR = adc_.CR | cr_adstart;
    timer_.CNT = 0U;
    timer_.SR = 0U;
    timer_.CR1 = timer_.CR1 | tim_cr1_enable;
    running_ = true;
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto stop() -> result<void, error_type> {
    timer_.CR1 = timer_.CR1 & ~tim_cr1_enable;
    if ((adc_.CR & cr_adstart) != 0U) {
      adc_.CR = adc_.CR | cr_adstp;
      if (!wait_clear(adc_.CR, cr_adstart | cr_adstp)) {
        return fail(acquisition_fault::adc_stop_timeout,
                    hal::adc::error_kind::timeout);
      }
    }
    if (!disable_dma()) {
      return fail(acquisition_fault::dma_stop_timeout,
                  hal::adc::error_kind::timeout);
    }
    clear_dma_flags();
    running_ = false;
    return result<void, error_type>::success();
  }

  // Call only from the selected DMA stream's IRQ. This path has constant
  // execution time: it acknowledges hardware and records completed halves.
  // Copying and publication are deferred to service_dma_completions(), which
  // latest()/try_read() invoke from task context. The DMA storage must be
  // coherent (normally by an MPU non-cacheable region on H7).
  void on_dma_interrupt() noexcept {
    barrier();
    const std::uint32_t flags = dma_.LISR;
    clear_dma_flags(flags);

    if ((flags & dma_error_flags) != 0U) {
      const auto fault = (flags & dma_transfer_error) != 0U
                             ? acquisition_fault::dma_transfer
                             : ((flags & dma_direct_mode_error) != 0U
                                    ? acquisition_fault::dma_direct_mode
                                    : acquisition_fault::dma_fifo);
      fault_.store(static_cast<std::uint32_t>(fault),
                   std::memory_order_release);
      return;
    }
    if ((adc_.ISR & isr_overrun) != 0U) {
      adc_.ISR = isr_overrun;
      fault_.store(static_cast<std::uint32_t>(acquisition_fault::adc_overrun),
                   std::memory_order_release);
    }
    if ((flags & dma_half_transfer) != 0U) {
      first_half_generation_.fetch_add(1U, std::memory_order_relaxed);
      const std::uint32_t previous =
          pending_halves_.fetch_or(first_half_pending,
                                   std::memory_order_release);
      if ((previous & first_half_pending) != 0U) {
        fault_.store(static_cast<std::uint32_t>(acquisition_fault::adc_overrun),
                     std::memory_order_release);
      }
    }
    if ((flags & dma_transfer_complete) != 0U) {
      second_half_generation_.fetch_add(1U, std::memory_order_relaxed);
      const std::uint32_t previous =
          pending_halves_.fetch_or(second_half_pending,
                                   std::memory_order_release);
      if ((previous & second_half_pending) != 0U) {
        fault_.store(static_cast<std::uint32_t>(acquisition_fault::adc_overrun),
                     std::memory_order_release);
      }
    }
  }

  // Task-context maintenance. It must run at least once per complete DMA
  // buffer period. A generation change while a half is copied is detected as
  // overrun so consumers never accept a silently torn sample block.
  [[nodiscard]] auto service_dma_completions()
      -> result<void, error_type> {
    const std::uint32_t pending =
        pending_halves_.exchange(0U, std::memory_order_acq_rel);
    if ((pending & first_half_pending) != 0U &&
        !publish_stable_half(0U, first_half_generation_)) {
      return fail(acquisition_fault::adc_overrun,
                  hal::adc::error_kind::overrun);
    }
    if ((pending & second_half_pending) != 0U &&
        !publish_stable_half(DmaSampleCount / 2U, second_half_generation_)) {
      return fail(acquisition_fault::adc_overrun,
                  hal::adc::error_kind::overrun);
    }
    const acquisition_fault observed = fault();
    return observed == acquisition_fault::none
               ? result<void, error_type>::success()
               : result<void, error_type>::failure(error{fault_kind(observed)});
  }

  [[nodiscard]] auto latest() -> result<hal::adc::raw_sample, error_type> {
    const auto serviced = service_dma_completions();
    if (!serviced) {
      return result<hal::adc::raw_sample, error_type>::failure(
          serviced.error());
    }
    const acquisition_fault observed_fault = fault();
    if (observed_fault != acquisition_fault::none) {
      return result<hal::adc::raw_sample, error_type>::failure(
          error{fault_kind(observed_fault)});
    }
    return publication_.latest();
  }

  [[nodiscard]] auto read_raw()
      -> result<hal::adc::raw_sample, error_type> {
    return latest();
  }

  [[nodiscard]] auto read_voltage()
      -> result<hal::adc::voltage_sample, error_type> {
    const auto raw = read_raw();
    if (!raw) {
      return result<hal::adc::voltage_sample, error_type>::failure(raw.error());
    }
    constexpr std::uint64_t code_count =
        Characteristics.resolution_bits >= 32U
            ? std::numeric_limits<std::uint32_t>::max()
            : (std::uint64_t{1U} << Characteristics.resolution_bits) - 1U;
    if (code_count == 0U || Characteristics.nominal_max.value <
                                 Characteristics.nominal_min.value) {
      return result<hal::adc::voltage_sample, error_type>::failure(
          error_type{hal::adc::error_kind::configuration});
    }
    const std::int64_t span_uv = Characteristics.nominal_max.value -
                                 Characteristics.nominal_min.value;
    const std::int64_t offset = static_cast<std::int64_t>(
        (static_cast<std::uint64_t>(raw.value().value) *
         static_cast<std::uint64_t>(span_uv)) /
        code_count);
    return result<hal::adc::voltage_sample, error_type>::success(
        {hal::microvolts{Characteristics.nominal_min.value + offset},
         raw.value().captured_at, raw.value().sequence});
  }

  [[nodiscard]] auto try_read(hal::span<hal::adc::raw_sample> output)
      -> result<hal::adc::stream_read, error_type> {
    const auto serviced = service_dma_completions();
    if (!serviced) {
      return result<hal::adc::stream_read, error_type>::failure(
          serviced.error());
    }
    const acquisition_fault observed_fault = fault();
    if (observed_fault != acquisition_fault::none) {
      return result<hal::adc::stream_read, error_type>::failure(
          error{fault_kind(observed_fault)});
    }
    return publication_.try_read(output);
  }

  [[nodiscard]] acquisition_fault fault() const noexcept {
    return static_cast<acquisition_fault>(
        fault_.load(std::memory_order_acquire));
  }
  [[nodiscard]] bool running() const noexcept { return running_; }

private:
  static constexpr std::uint32_t cr_aden{1U << 0U};
  static constexpr std::uint32_t cr_adstart{1U << 2U};
  static constexpr std::uint32_t cr_adstp{1U << 4U};
  static constexpr std::uint32_t cr_boost_mask{3U << 8U};
  static constexpr std::uint32_t cr_boost_position{8U};
  static constexpr std::uint32_t cr_linearity_calibration{1U << 16U};
  static constexpr std::uint32_t cr_voltage_regulator{1U << 28U};
  static constexpr std::uint32_t cr_deep_power_down{1U << 29U};
  static constexpr std::uint32_t cr_calibration_differential{1U << 30U};
  static constexpr std::uint32_t cr_adcal{1U << 31U};
  static constexpr std::uint32_t cr_start_calibration{1U << 31U};
  static constexpr std::uint32_t isr_ready{1U << 0U};
  static constexpr std::uint32_t isr_overrun{1U << 4U};
  static constexpr std::uint32_t isr_ldo_ready{1U << 12U};
  static constexpr std::uint32_t cfgr_dma_circular{3U};
  static constexpr std::uint32_t cfgr_resolution_mask{7U << 2U};
  // ADC1/2 EXTSEL is bits 9:5 on STM32H723. EXTEN is bits 11:10.
  static constexpr std::uint32_t cfgr_external_trigger_position{5U};
  static constexpr std::uint32_t cfgr_external_trigger_rising{1U << 10U};
  static constexpr std::uint32_t cfgr_overwrite_on_overrun{1U << 12U};
  static constexpr std::uint32_t cfgr2_oversampling_enable{1U << 0U};
  static constexpr std::uint32_t cfgr2_shift_position{5U};
  static constexpr std::uint32_t cfgr2_ratio_position{16U};
  static constexpr std::uint32_t sqr1_first_channel_position{6U};
  static constexpr std::uint32_t ccr_clock_mode_mask{3U << 16U};
  static constexpr std::uint32_t ccr_prescaler_mask{15U << 18U};
  static constexpr std::uint32_t ccr_prescaler_position{18U};
  static constexpr std::uint32_t tim_cr1_enable{1U << 0U};
  static constexpr std::uint32_t tim_cr2_mms_mask{7U << 4U};
  static constexpr std::uint32_t tim_cr2_mms_update{2U << 4U};
  static constexpr std::uint32_t tim_egr_ug{1U << 0U};
  static constexpr std::uint32_t dma_cr_enable{1U << 0U};
  static constexpr std::uint32_t dma_cr_direct_mode_error_interrupt{1U << 1U};
  static constexpr std::uint32_t dma_cr_transfer_error_interrupt{1U << 2U};
  static constexpr std::uint32_t dma_cr_half_transfer_interrupt{1U << 3U};
  static constexpr std::uint32_t dma_cr_transfer_complete_interrupt{1U << 4U};
  static constexpr std::uint32_t dma_cr_circular{1U << 8U};
  static constexpr std::uint32_t dma_cr_memory_increment{1U << 10U};
  static constexpr std::uint32_t dma_cr_peripheral_halfword{1U << 11U};
  static constexpr std::uint32_t dma_cr_memory_word{2U << 13U};
  static constexpr std::uint32_t dma_cr_priority_high{2U << 16U};
  static constexpr std::uint32_t dma_fifo_error{1U << 0U};
  static constexpr std::uint32_t dma_direct_mode_error{1U << 2U};
  static constexpr std::uint32_t dma_transfer_error{1U << 3U};
  static constexpr std::uint32_t dma_half_transfer{1U << 4U};
  static constexpr std::uint32_t dma_transfer_complete{1U << 5U};
  static constexpr std::uint32_t dma_all_flags{
      dma_fifo_error | dma_direct_mode_error | dma_transfer_error |
      dma_half_transfer | dma_transfer_complete};
  static constexpr std::uint32_t dma_error_flags{
      dma_fifo_error | dma_direct_mode_error | dma_transfer_error};
  static constexpr std::uint32_t first_half_pending{1U << 0U};
  static constexpr std::uint32_t second_half_pending{1U << 1U};

  [[nodiscard]] static std::uint32_t
  pointer_word(const volatile void *pointer) noexcept {
    return static_cast<std::uint32_t>(
        reinterpret_cast<std::uintptr_t>(pointer));
  }

  [[nodiscard]] constexpr std::uint32_t resolution_code() const noexcept {
    const std::uint32_t code = configuration_.adc_resolution_bits == 16U ? 0U
                               : configuration_.adc_resolution_bits == 14U ? 1U
                               : configuration_.adc_resolution_bits == 12U ? 2U
                                                                            : 3U;
    return (code << 2U) & cfgr_resolution_mask;
  }

  [[nodiscard]] bool wait_clear(volatile std::uint32_t &reg,
                                std::uint32_t mask) const noexcept {
    for (std::uint32_t remaining = configuration_.timeout.iterations;
         remaining > 0U; --remaining) {
      if ((reg & mask) == 0U) {
        return true;
      }
    }
    return false;
  }

  [[nodiscard]] bool wait_set(volatile std::uint32_t &reg,
                              std::uint32_t mask) const noexcept {
    for (std::uint32_t remaining = configuration_.timeout.iterations;
         remaining > 0U; --remaining) {
      if ((reg & mask) == mask) {
        return true;
      }
    }
    return false;
  }

  [[nodiscard]] bool disable_dma() noexcept {
    stream_.CR = stream_.CR & ~dma_cr_enable;
    return wait_clear(stream_.CR, dma_cr_enable);
  }

  void clear_dma_flags() noexcept { dma_.LIFCR = dma_all_flags; }

  void clear_dma_flags(std::uint32_t observed) noexcept {
    dma_.LIFCR = observed & dma_all_flags;
  }

  void replace_sample_time() noexcept {
    const std::uint32_t position =
        static_cast<std::uint32_t>((configuration_.channel % 10U) * 3U);
    volatile std::uint32_t &register_value =
        configuration_.channel < 10U ? adc_.SMPR1 : adc_.SMPR2;
    const std::uint32_t mask = 7U << position;
    register_value =
        (register_value & ~mask) |
        (static_cast<std::uint32_t>(configuration_.sample_time_code)
         << position);
  }

  void configure_oversampling() noexcept {
    if (configuration_.oversampling_ratio == 1U) {
      adc_.CFGR2 = 0U;
      return;
    }
    adc_.CFGR2 =
        cfgr2_oversampling_enable |
        (static_cast<std::uint32_t>(configuration_.oversampling_shift)
         << cfgr2_shift_position) |
        (static_cast<std::uint32_t>(oversampling_code(
             configuration_.oversampling_ratio))
         << cfgr2_ratio_position);
  }

  [[nodiscard]] static constexpr std::uint8_t
  oversampling_code(std::uint16_t ratio) noexcept {
    switch (ratio) {
    case 2U:
      return 0U;
    case 4U:
      return 1U;
    case 8U:
      return 2U;
    case 16U:
      return 3U;
    case 32U:
      return 4U;
    case 64U:
      return 5U;
    case 128U:
      return 6U;
    case 256U:
      return 7U;
    case 512U:
      return 8U;
    case 1024U:
      return 9U;
    default:
      return 0xFFU;
    }
  }

  [[nodiscard]] constexpr std::uint32_t boost_code() const noexcept {
    const std::uint32_t clock = configuration_.analog_clock_hz();
    const std::uint32_t code = clock <= 6'250'000U    ? 0U
                               : clock <= 12'500'000U ? 1U
                               : clock <= 25'000'000U ? 2U
                                                      : 3U;
    return code << cr_boost_position;
  }

  [[nodiscard]] constexpr std::uint32_t
  post_calibration_delay_us() const noexcept {
    constexpr std::uint64_t four_cycles_in_microhertz{4'000'000ULL};
    const std::uint64_t clock = configuration_.analog_clock_hz();
    return static_cast<std::uint32_t>((four_cycles_in_microhertz + clock - 1U) /
                                      clock);
  }

  [[nodiscard]] constexpr bool
  oversampling_matches_characteristics() const noexcept {
    std::uint16_t ratio = configuration_.oversampling_ratio;
    std::uint8_t added_bits = 0U;
    while (ratio > 1U && (ratio & 1U) == 0U) {
      ratio = static_cast<std::uint16_t>(ratio / 2U);
      ++added_bits;
    }
    if (ratio != 1U ||
        configuration_.oversampling_shift >
            added_bits + configuration_.adc_resolution_bits) {
      return false;
    }
    if (configuration_.adc_resolution_bits + added_bits <=
        configuration_.oversampling_shift) {
      return false;
    }
    const auto effective_bits = static_cast<std::uint8_t>(
        configuration_.adc_resolution_bits + added_bits -
        configuration_.oversampling_shift);
    return effective_bits == Characteristics.resolution_bits &&
           oversampling_code(configuration_.oversampling_ratio) != 0xFFU;
  }

  [[nodiscard]] bool
  publish_stable_half(std::size_t first,
                      const std::atomic<std::uint32_t> &generation) noexcept {
    const std::uint32_t before = generation.load(std::memory_order_acquire);
    constexpr std::size_t half = DmaSampleCount / 2U;
    for (std::size_t index = 0U; index < half; ++index) {
      publication_.publish_from_isr(buffer_[first + index],
                                    instant{next_timestamp_ns_});
      next_timestamp_ns_ += configuration_.sample_period_ns;
    }
    barrier();
    return before == generation.load(std::memory_order_acquire);
  }

  [[nodiscard]] static hal::adc::error_kind
  fault_kind(acquisition_fault fault) noexcept {
    switch (fault) {
    case acquisition_fault::adc_calibration_timeout:
      return hal::adc::error_kind::calibration;
    case acquisition_fault::adc_ready_timeout:
    case acquisition_fault::adc_regulator_timeout:
    case acquisition_fault::adc_stop_timeout:
    case acquisition_fault::dma_stop_timeout:
      return hal::adc::error_kind::timeout;
    case acquisition_fault::adc_overrun:
      return hal::adc::error_kind::overrun;
    case acquisition_fault::none:
      return hal::adc::error_kind::not_ready;
    default:
      return hal::adc::error_kind::io;
    }
  }

  [[nodiscard]] auto fail(acquisition_fault fault, hal::adc::error_kind kind)
      -> result<void, error_type> {
    fault_.store(static_cast<std::uint32_t>(fault), std::memory_order_release);
    return result<void, error_type>::failure(error{kind});
  }

  static void barrier() noexcept {
#if defined(__arm__) || defined(__thumb__)
    __asm volatile("dsb 0xF" ::: "memory");
#else
    std::atomic_thread_fence(std::memory_order_seq_cst);
#endif
  }

  AdcRegisters &adc_;
  AdcCommonRegisters &common_;
  DmaRegisters &dma_;
  DmaStreamRegisters &stream_;
  DmamuxChannelRegisters &dmamux_;
  TimerRegisters &timer_;
  volatile std::uint32_t (&buffer_)[DmaSampleCount];
  dma_config configuration_{};
  ContinuousChannel<Characteristics, QueueCapacity> publication_{};
  std::uint64_t next_timestamp_ns_{};
  std::atomic<std::uint32_t> fault_{};
  std::atomic<std::uint32_t> pending_halves_{};
  std::atomic<std::uint32_t> first_half_generation_{};
  std::atomic<std::uint32_t> second_half_generation_{};
  bool initialized_{};
  bool running_{};
};

} // namespace hal::stm32h7::adc
