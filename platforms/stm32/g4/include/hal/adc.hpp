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

namespace hal::stm32g4::adc {

inline constexpr capability publication_support{
    implementation_status::available,
    "bounded ISR-to-task continuous sample publication"};
inline constexpr capability acquisition_support{
    implementation_status::available,
    "G4 regular ADC timer-triggered circular DMA acquisition"};

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

  void publish_from_isr(std::uint32_t raw, hal::instant captured_at) noexcept {
    const std::uint64_t sequence = produced_by_isr_ + 1U;
    produced_by_isr_ = sequence;
    generation_.fetch_add(1U, std::memory_order_acq_rel);
    slot &destination = samples_[sequence % Capacity];
    destination.raw.store(raw, std::memory_order_relaxed);
    destination.time_low.store(
        static_cast<std::uint32_t>(captured_at.nanoseconds_since_boot),
        std::memory_order_relaxed);
    destination.time_high.store(
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

  [[nodiscard]] auto read_raw()
      -> result<hal::adc::raw_sample, error_type> {
    std::uint64_t produced{};
    hal::adc::raw_sample value{};
    if (!snapshot_latest(produced, value)) return failure_raw(hal::adc::error_kind::overrun);
    if (produced == 0U) return failure_raw(hal::adc::error_kind::not_ready);
    return result<hal::adc::raw_sample, error_type>::success(value);
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
    if (code_count == 0U ||
        Characteristics.nominal_max.value < Characteristics.nominal_min.value) {
      return result<hal::adc::voltage_sample, error_type>::failure(
          error{hal::adc::error_kind::configuration});
    }
    const std::int64_t span = Characteristics.nominal_max.value -
                              Characteristics.nominal_min.value;
    const std::int64_t offset = static_cast<std::int64_t>(
        raw.value().value * static_cast<std::uint64_t>(span) / code_count);
    return result<hal::adc::voltage_sample, error_type>::success(
        {hal::microvolts{Characteristics.nominal_min.value + offset},
         raw.value().captured_at, raw.value().sequence});
  }

  [[nodiscard]] auto try_read(hal::span<hal::adc::raw_sample> output)
      -> result<hal::adc::stream_read, error_type> {
    HAL_CORE_ASSERT(output.valid());
    if (!output.valid()) return failure_stream(hal::adc::error_kind::other);
    for (unsigned attempt = 0U; attempt < 3U; ++attempt) {
      const std::uint32_t before = generation_.load(std::memory_order_acquire);
      if ((before & 1U) != 0U) continue;
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
        return result<hal::adc::stream_read, error_type>::success({count, dropped});
      }
    }
    return failure_stream(hal::adc::error_kind::overrun);
  }

private:
  struct slot {
    std::atomic<std::uint32_t> raw{};
    std::atomic<std::uint32_t> time_low{};
    std::atomic<std::uint32_t> time_high{};
    std::atomic<std::uint32_t> sequence_low{};
    std::atomic<std::uint32_t> sequence_high{};
  };

  [[nodiscard]] static hal::adc::raw_sample load(const slot &source) noexcept {
    const auto timestamp =
        (static_cast<std::uint64_t>(source.time_high.load(std::memory_order_relaxed))
         << 32U) | source.time_low.load(std::memory_order_relaxed);
    const auto sequence =
        (static_cast<std::uint64_t>(source.sequence_high.load(std::memory_order_relaxed))
         << 32U) | source.sequence_low.load(std::memory_order_relaxed);
    return {source.raw.load(std::memory_order_relaxed), hal::instant{timestamp},
            sequence};
  }

  [[nodiscard]] std::uint64_t published_sequence() const noexcept {
    return (static_cast<std::uint64_t>(published_high_.load(std::memory_order_relaxed))
            << 32U) | published_low_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] bool snapshot_latest(std::uint64_t &produced,
                                      hal::adc::raw_sample &value) const noexcept {
    for (unsigned attempt = 0U; attempt < 3U; ++attempt) {
      const auto before = generation_.load(std::memory_order_acquire);
      if ((before & 1U) != 0U) continue;
      produced = published_sequence();
      if (produced != 0U) value = load(samples_[produced % Capacity]);
      const auto after = generation_.load(std::memory_order_acquire);
      if (before == after) return true;
    }
    return false;
  }

  [[nodiscard]] static auto failure_raw(hal::adc::error_kind kind)
      -> result<hal::adc::raw_sample, error_type> {
    return result<hal::adc::raw_sample, error_type>::failure(error{kind});
  }

  [[nodiscard]] static auto failure_stream(hal::adc::error_kind kind)
      -> result<hal::adc::stream_read, error_type> {
    return result<hal::adc::stream_read, error_type>::failure(error{kind});
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

struct dma_config {
  std::uint32_t adc_input_clock_hz{};
  asynchronous_prescaler adc_prescaler{asynchronous_prescaler::divide_by_1};
  std::uint8_t channel{};
  std::uint8_t sample_time_code{};
  std::uint16_t oversampling_ratio{1U};
  std::uint8_t oversampling_shift{};
  std::uint8_t external_trigger{};
  std::uint8_t dma_request{};
  std::uint16_t timer_prescaler{};
  std::uint32_t timer_auto_reload{};
  std::uint64_t sample_period_ns{};
  poll_budget timeout{};
  std::uint8_t adc_resolution_bits{12U};

  [[nodiscard]] constexpr std::uint32_t prescaler_divisor() const noexcept {
    constexpr std::uint16_t values[] = {1U, 2U, 4U, 6U, 8U, 10U,
                                        12U, 16U, 32U, 64U, 128U, 256U};
    const auto index = static_cast<std::size_t>(adc_prescaler);
    return index < sizeof(values) / sizeof(values[0U]) ? values[index] : 0U;
  }

  [[nodiscard]] constexpr std::uint32_t analog_clock_hz() const noexcept {
    const auto divisor = prescaler_divisor();
    return divisor == 0U ? 0U : adc_input_clock_hz / divisor;
  }

  [[nodiscard]] constexpr bool valid() const noexcept {
    const auto ratio = oversampling_ratio;
    const bool ratio_valid = ratio == 1U ||
                             (ratio >= 2U && ratio <= 256U &&
                              (ratio & (ratio - 1U)) == 0U);
    return adc_input_clock_hz > 0U && analog_clock_hz() <= 60'000'000U &&
           channel < 19U && sample_time_code < 8U && ratio_valid &&
           oversampling_shift <= 8U && external_trigger < 32U &&
           timer_auto_reload > 0U && sample_period_ns > 0U && timeout.valid() &&
           dma_request < 128U &&
           (adc_resolution_bits == 6U || adc_resolution_bits == 8U ||
            adc_resolution_bits == 10U || adc_resolution_bits == 12U);
  }
};

enum class acquisition_fault : std::uint8_t {
  none,
  invalid_configuration,
  adc_busy,
  regulator_timeout,
  calibration_timeout,
  ready_timeout,
  stop_timeout,
  dma_transfer,
  adc_overrun
};

template <hal::adc::characteristics Characteristics, std::size_t QueueCapacity,
          std::size_t DmaSampleCount, class AdcRegisters,
          class AdcCommonRegisters, class DmaRegisters,
          class DmaChannelRegisters, class DmamuxChannelRegisters,
          class TimerRegisters, unsigned TimerCounterBits = 16U>
class AdcDma {
  static_assert(DmaSampleCount >= 2U && (DmaSampleCount % 2U) == 0U);
  static_assert(DmaSampleCount <= 65'535U);
  static_assert(TimerCounterBits == 16U || TimerCounterBits == 32U);
  static_assert(std::atomic<std::uint8_t>::is_always_lock_free);

public:
  using error_type = error;

  [[nodiscard]] static constexpr hal::adc::characteristics
  properties() noexcept {
    return Characteristics;
  }

  AdcDma(AdcRegisters &adc, AdcCommonRegisters &common, DmaRegisters &dma,
         DmaChannelRegisters &channel, DmamuxChannelRegisters &dmamux,
         TimerRegisters &timer, volatile std::uint16_t (&buffer)[DmaSampleCount],
         std::uint8_t dma_channel_number, dma_config configuration) noexcept
      : adc_{adc}, common_{common}, dma_{dma}, channel_{channel},
        dmamux_{dmamux}, timer_{timer}, buffer_{buffer},
        dma_channel_number_{dma_channel_number}, configuration_{configuration} {}

  AdcDma(const AdcDma &) = delete;
  AdcDma &operator=(const AdcDma &) = delete;

  template <class MicrosecondDelay>
  [[nodiscard]] auto initialize(MicrosecondDelay &delay_us)
      -> result<void, error_type> {
    if (!configuration_.valid() ||
        Characteristics.resolution_bits != configuration_.adc_resolution_bits ||
        configuration_.timer_auto_reload > timer_counter_max ||
        dma_channel_number_ == 0U || dma_channel_number_ > 8U) {
      return fail(acquisition_fault::invalid_configuration,
                  hal::adc::error_kind::configuration);
    }
    if ((adc_.CR & (cr_aden | cr_adstart | cr_adcal)) != 0U) {
      return fail(acquisition_fault::adc_busy, hal::adc::error_kind::io);
    }
    stop_dma();
    timer_.CR1 = 0U;
    timer_.DIER = 0U;
    timer_.CR2 = (timer_.CR2 & ~(7U << 4U)) | (2U << 4U);
    timer_.PSC = configuration_.timer_prescaler;
    timer_.ARR = configuration_.timer_auto_reload;
    timer_.CNT = 0U;
    timer_.EGR = 1U;
    timer_.SR = 0U;

    dmamux_.CCR = dmamux_request();
    channel_.CPAR = pointer_word(&adc_.DR);
    channel_.CMAR = pointer_word(buffer_);
    channel_.CNDTR = static_cast<std::uint32_t>(DmaSampleCount);
    channel_.CCR = dma_htie | dma_tcie | dma_teie | dma_minc | dma_circ |
                   dma_psize_halfword | dma_msize_halfword;
    adc_.CR = adc_.CR & ~cr_deeppwd;
    adc_.CR = adc_.CR | cr_advregen;
    delay_us(20U);
    if (!wait_adc_status(isr_ldordy)) {
      return fail(acquisition_fault::regulator_timeout,
                  hal::adc::error_kind::timeout);
    }

    common_.CCR = (common_.CCR & ~(0xFU << 18U)) |
                  (static_cast<std::uint32_t>(configuration_.adc_prescaler) << 18U);
    adc_.DIFSEL = adc_.DIFSEL & ~(std::uint32_t{1U} << configuration_.channel);
    adc_.CR = adc_.CR | cr_adcal;
    if (!wait_adc_clear(cr_adcal)) {
      return fail(acquisition_fault::calibration_timeout,
                  hal::adc::error_kind::calibration);
    }
    delay_us(2U);
    adc_.ISR = isr_adrdy | isr_ovr | isr_eoc | isr_eos;
    adc_.CR = adc_.CR | cr_aden;
    if (!wait_adc_status(isr_adrdy)) {
      return fail(acquisition_fault::ready_timeout,
                  hal::adc::error_kind::timeout);
    }

    const std::uint32_t shift = static_cast<std::uint32_t>(configuration_.channel) * 3U;
    if (configuration_.channel < 10U) {
      adc_.SMPR1 = (adc_.SMPR1 & ~(7U << shift)) |
                   (static_cast<std::uint32_t>(configuration_.sample_time_code) << shift);
    } else {
      const std::uint32_t second_shift =
          static_cast<std::uint32_t>(configuration_.channel - 10U) * 3U;
      adc_.SMPR2 = (adc_.SMPR2 & ~(7U << second_shift)) |
                   (static_cast<std::uint32_t>(configuration_.sample_time_code)
                    << second_shift);
    }
    adc_.SQR1 = static_cast<std::uint32_t>(configuration_.channel) << 6U;
    adc_.CFGR = dma_enable | dma_circular | dma_overwrite |
                (static_cast<std::uint32_t>(configuration_.external_trigger) << 5U) |
                external_trigger_rising | resolution_code();
    configure_oversampling();
    adc_.IER = 0U;
    fault_.store(static_cast<std::uint8_t>(acquisition_fault::none),
                 std::memory_order_release);
    initialized_ = true;
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto start(hal::instant epoch) -> result<void, error_type> {
    if (!initialized_) return fail(acquisition_fault::invalid_configuration,
                                   hal::adc::error_kind::not_ready);
    if (running_) return result<void, error_type>::success();
    clear_dma_flags();
    channel_.CNDTR = static_cast<std::uint32_t>(DmaSampleCount);
    next_timestamp_ns_ = epoch.nanoseconds_since_boot +
                         configuration_.sample_period_ns;
    fault_.store(static_cast<std::uint8_t>(acquisition_fault::none),
                 std::memory_order_release);
    channel_.CCR = channel_.CCR | dma_enable;
    adc_.ISR = isr_ovr | isr_eoc | isr_eos;
    adc_.CR = adc_.CR | cr_adstart;
    timer_.CNT = 0U;
    timer_.SR = 0U;
    timer_.CR1 = timer_.CR1 | 1U;
    running_ = true;
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto stop() -> result<void, error_type> {
    timer_.CR1 = timer_.CR1 & ~1U;
    if ((adc_.CR & cr_adstart) != 0U) {
      adc_.CR = adc_.CR | cr_adstp;
      if (!wait_adc_clear(cr_adstart | cr_adstp)) {
        return fail(acquisition_fault::stop_timeout, hal::adc::error_kind::timeout);
      }
    }
    stop_dma();
    running_ = false;
    return result<void, error_type>::success();
  }

  void on_dma_interrupt() noexcept {
    const std::uint32_t flags = dma_.ISR;
    const unsigned shift = (dma_channel_number_ - 1U) * 4U;
    const std::uint32_t channel_flags = 0xFU << shift;
    dma_.IFCR = channel_flags;
    if ((flags & (1U << (shift + 3U))) != 0U) {
      fault_.store(static_cast<std::uint8_t>(acquisition_fault::dma_transfer),
                   std::memory_order_release);
      return;
    }
    if ((adc_.ISR & isr_ovr) != 0U) {
      adc_.ISR = isr_ovr;
      fault_.store(static_cast<std::uint8_t>(acquisition_fault::adc_overrun),
                   std::memory_order_release);
      return;
    }
    if ((flags & (1U << (shift + 2U))) != 0U) publish_half(0U, DmaSampleCount / 2U);
    if ((flags & (1U << (shift + 1U))) != 0U)
      publish_half(DmaSampleCount / 2U, DmaSampleCount);
  }

  [[nodiscard]] auto read_raw() -> result<hal::adc::raw_sample, error_type> {
    return publication_.read_raw();
  }
  [[nodiscard]] auto read_voltage()
      -> result<hal::adc::voltage_sample, error_type> {
    return publication_.read_voltage();
  }
  [[nodiscard]] auto try_read(hal::span<hal::adc::raw_sample> output)
      -> result<hal::adc::stream_read, error_type> {
    return publication_.try_read(output);
  }
  [[nodiscard]] acquisition_fault fault() const noexcept {
    return static_cast<acquisition_fault>(fault_.load(std::memory_order_acquire));
  }
  [[nodiscard]] bool running() const noexcept { return running_; }

private:
  static constexpr std::uint32_t cr_aden{1U};
  static constexpr std::uint32_t cr_adstart{1U << 2U};
  static constexpr std::uint32_t cr_adstp{1U << 4U};
  static constexpr std::uint32_t cr_advregen{1U << 28U};
  static constexpr std::uint32_t cr_deeppwd{1U << 29U};
  static constexpr std::uint32_t cr_adcal{1U << 31U};
  static constexpr std::uint32_t isr_adrdy{1U};
  static constexpr std::uint32_t isr_eoc{1U << 2U};
  static constexpr std::uint32_t isr_eos{1U << 3U};
  static constexpr std::uint32_t isr_ovr{1U << 4U};
  static constexpr std::uint32_t isr_ldordy{1U << 12U};
  static constexpr std::uint32_t dma_enable{1U};
  static constexpr std::uint32_t dma_circular{1U << 1U};
  static constexpr std::uint32_t dma_overwrite{1U << 12U};
  static constexpr std::uint32_t external_trigger_rising{1U << 10U};
  static constexpr std::uint32_t dma_enable_channel{1U};
  static constexpr std::uint32_t dma_htie{1U << 2U};
  static constexpr std::uint32_t dma_tcie{1U << 1U};
  static constexpr std::uint32_t dma_teie{1U << 3U};
  static constexpr std::uint32_t dma_minc{1U << 7U};
  static constexpr std::uint32_t dma_circ{1U << 5U};
  static constexpr std::uint32_t dma_psize_halfword{1U << 8U};
  static constexpr std::uint32_t dma_msize_halfword{1U << 10U};
  static constexpr std::uint32_t timer_counter_max =
      TimerCounterBits == 16U ? 0xFFFFU : 0xFFFF'FFFFU;

  [[nodiscard]] std::uint8_t dmamux_request() const noexcept {
    return configuration_.dma_request;
  }

  [[nodiscard]] bool wait_adc_status(std::uint32_t mask) const noexcept {
    for (std::uint32_t left = configuration_.timeout.iterations; left > 0U; --left) {
      if ((adc_.ISR & mask) != 0U) return true;
    }
    return false;
  }

  [[nodiscard]] bool wait_adc_clear(std::uint32_t mask) const noexcept {
    for (std::uint32_t left = configuration_.timeout.iterations; left > 0U; --left) {
      if ((adc_.CR & mask) == 0U) return true;
    }
    return false;
  }

  [[nodiscard]] std::uint32_t resolution_code() const noexcept {
    return configuration_.adc_resolution_bits == 12U ? 0U
           : configuration_.adc_resolution_bits == 10U ? 1U << 3U
           : configuration_.adc_resolution_bits == 8U ? 2U << 3U
                                                       : 3U << 3U;
  }

  void configure_oversampling() noexcept {
    constexpr std::uint32_t oversampling_fields =
        (1U << 0U) | (7U << 2U) | (0xFU << 5U) | (1U << 9U) | (1U << 10U);
    if (configuration_.oversampling_ratio == 1U) {
      adc_.CFGR2 = adc_.CFGR2 & ~oversampling_fields;
      return;
    }
    unsigned ratio_code = 0U;
    std::uint16_t ratio = configuration_.oversampling_ratio;
    while (ratio > 2U) {
      ratio = static_cast<std::uint16_t>(ratio / 2U);
      ++ratio_code;
    }
    adc_.CFGR2 = (adc_.CFGR2 & ~oversampling_fields) | 1U |
                 (static_cast<std::uint32_t>(ratio_code) << 2U) |
                 (static_cast<std::uint32_t>(configuration_.oversampling_shift) << 5U);
  }

  void publish_half(std::size_t first, std::size_t end) noexcept {
    if (!running_) return;
    for (std::size_t i = first; i < end; ++i) {
      publication_.publish_from_isr(buffer_[i], hal::instant{next_timestamp_ns_});
      next_timestamp_ns_ += configuration_.sample_period_ns;
    }
  }

  void clear_dma_flags() noexcept {
    const unsigned shift = (dma_channel_number_ - 1U) * 4U;
    dma_.IFCR = 0xFU << shift;
  }

  void stop_dma() noexcept { channel_.CCR = channel_.CCR & ~dma_enable_channel; }

  [[nodiscard]] static std::uint32_t pointer_word(const volatile void *pointer)
      noexcept {
    return static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(pointer));
  }

  [[nodiscard]] auto fail(acquisition_fault fault,
                          hal::adc::error_kind kind)
      -> result<void, error_type> {
    fault_.store(static_cast<std::uint8_t>(fault), std::memory_order_release);
    stop_dma();
    return result<void, error_type>::failure(error{kind});
  }

  AdcRegisters &adc_;
  AdcCommonRegisters &common_;
  DmaRegisters &dma_;
  DmaChannelRegisters &channel_;
  DmamuxChannelRegisters &dmamux_;
  TimerRegisters &timer_;
  volatile std::uint16_t (&buffer_)[DmaSampleCount];
  std::uint8_t dma_channel_number_{};
  dma_config configuration_{};
  ContinuousChannel<Characteristics, QueueCapacity> publication_{};
  std::atomic<std::uint8_t> fault_{
      static_cast<std::uint8_t>(acquisition_fault::none)};
  std::uint64_t next_timestamp_ns_{};
  bool initialized_{};
  bool running_{};
};

} // namespace hal::stm32g4::adc
