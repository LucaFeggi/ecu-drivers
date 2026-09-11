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
#include <hal/foundation/assert.hpp>
#include "support.hpp"
#include <limits>

namespace hal::stm32h5::adc {

inline constexpr capability publication_support{
    implementation_status::available,
    "bounded ISR-to-task continuous sample publication"};

inline constexpr capability acquisition_support{
    implementation_status::available,
    "ADC1/2 DMA acquisition engine with external-trigger support"};

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

  [[nodiscard]] auto read_raw() -> result<hal::adc::raw_sample, error_type> {
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
    if (code_count == 0U ||
        Characteristics.nominal_max.value < Characteristics.nominal_min.value) {
      return result<hal::adc::voltage_sample, error_type>::failure(
          error_type{hal::adc::error_kind::configuration});
    }
    const std::int64_t span_uv =
        Characteristics.nominal_max.value - Characteristics.nominal_min.value;
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

  [[nodiscard]] bool
  snapshot_latest(std::uint64_t &produced,
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

struct dma_config {
  std::uint32_t input_clock_hz{};
  std::uint8_t channel{};
  std::uint8_t sample_time_code{};
  std::uint8_t external_trigger{};
  std::uint8_t dma_request{};
  poll_budget timeout{};
  [[nodiscard]] constexpr bool valid() const noexcept {
    return input_clock_hz != 0U && channel < 20U && sample_time_code < 8U &&
           external_trigger < 32U && dma_request < 128U && timeout.valid();
  }
};

enum class acquisition_fault : std::uint8_t {
  none,
  invalid_configuration,
  adc_busy,
  calibration_timeout,
  ready_timeout,
  stop_timeout,
  dma_transfer,
  adc_overrun
};

template <hal::adc::characteristics Characteristics, std::size_t SampleCount,
          class AdcRegisters, class AdcCommonRegisters, class DmaChannel>
class Adc12Gpdma {
  static_assert(SampleCount > 1U);
  static_assert(SampleCount * sizeof(std::uint32_t) <= 65'535U);

public:
  using error_type = error;
  Adc12Gpdma(AdcRegisters &adc, AdcCommonRegisters &common, DmaChannel &dma,
             volatile std::uint32_t (&buffer)[SampleCount],
             dma_config config) noexcept
      : adc_{adc}, common_{common}, dma_{dma}, buffer_{buffer},
        config_{config} {}
  Adc12Gpdma(const Adc12Gpdma &) = delete;
  Adc12Gpdma &operator=(const Adc12Gpdma &) = delete;

  [[nodiscard]] auto initialize() -> result<void, error_type> {
    struct bounded_startup_delay {
      std::uint32_t input_clock_hz;
      void operator()(std::uint32_t microseconds) const noexcept {
        const std::uint32_t cycles_per_microsecond = input_clock_hz / 1'000'000U;
        const std::uint32_t iterations =
            cycles_per_microsecond == 0U
                ? microseconds
                : cycles_per_microsecond * microseconds;
        for (std::uint32_t count = iterations; count > 0U; --count)
          asm volatile("" ::: "memory");
      }
    } delay{config_.input_clock_hz};
    return initialize(delay);
  }

  template <class MicrosecondDelay>
  [[nodiscard]] auto initialize(MicrosecondDelay &delay_us)
      -> result<void, error_type> {
    initialized_ = false;
    running_ = false;
    if (!config_.valid() || Characteristics.resolution_bits > 16U)
      return fail(acquisition_fault::invalid_configuration,
                  hal::adc::error_kind::configuration);
    if ((adc_.CR & (cr_aden | cr_adstart | cr_adcal)) != 0U)
      return fail(acquisition_fault::adc_busy, hal::adc::error_kind::io);

    dma_.CCR = dma_.CCR & ~dma_enable;
    dma_.CFCR = dma_all_flags;
    adc_.CR = adc_.CR & ~cr_deeppwd;
    adc_.CR = adc_.CR | cr_advregen;
    delay_us(20U);
    adc_.CR = adc_.CR | cr_adcal;
    if (!wait_clear(adc_.CR, cr_adcal))
      return fail(acquisition_fault::calibration_timeout,
                  hal::adc::error_kind::calibration);

    adc_.ISR = isr_adrdy | isr_eoc | isr_eos | isr_overrun;
    adc_.CFGR = ((static_cast<std::uint32_t>(
                      Characteristics.resolution_bits == 8U    ? 2U
                      : Characteristics.resolution_bits == 10U ? 1U
                                                               : 0U)
                  << 3U) |
                 (1U << 0U) | (1U << 1U));
    adc_.CFGR |= static_cast<std::uint32_t>(config_.external_trigger) << 5U;
    if (config_.external_trigger != 0U)
      adc_.CFGR |= 1U << 10U;
    if (config_.channel < 10U) {
      const auto shift = static_cast<std::uint32_t>(config_.channel) * 3U;
      adc_.SMPR1 =
          (adc_.SMPR1 & ~(7U << shift)) |
          (static_cast<std::uint32_t>(config_.sample_time_code) << shift);
    } else {
      const auto shift =
          (static_cast<std::uint32_t>(config_.channel) - 10U) * 3U;
      adc_.SMPR2 =
          (adc_.SMPR2 & ~(7U << shift)) |
          (static_cast<std::uint32_t>(config_.sample_time_code) << shift);
    }
    adc_.SQR1 = (static_cast<std::uint32_t>(config_.channel) << 6U);
    common_.CCR = common_.CCR & ~(3U << 16U);
    adc_.CR = adc_.CR | cr_aden;
    if (!wait_set(adc_.ISR, isr_adrdy))
      return fail(acquisition_fault::ready_timeout,
                  hal::adc::error_kind::timeout);
    fault_.store(acquisition_fault::none, std::memory_order_release);
    initialized_ = true;
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto start() -> result<void, error_type> {
    if (!initialized_)
      return fail(acquisition_fault::invalid_configuration,
                  hal::adc::error_kind::not_ready);
    dma_.CCR = dma_.CCR & ~dma_enable;
    dma_.CFCR = dma_all_flags;
    dma_.CTR1 = 2U | (2U << 16U) | (1U << 19U);
    dma_.CTR2 = config_.dma_request;
    dma_.CBR1 = static_cast<std::uint32_t>(SampleCount * sizeof(std::uint32_t));
    dma_.CSAR =
        static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(&adc_.DR));
    dma_.CDAR =
        static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(buffer_));
    dma_.CCR = dma_enable | dma_transfer_complete_interrupt |
               dma_transfer_error_interrupt;
    adc_.CFGR |= (1U << 0U) | (1U << 1U);
    adc_.ISR = isr_overrun | isr_eoc | isr_eos;
    adc_.CR |= cr_adstart;
    fault_.store(acquisition_fault::none, std::memory_order_release);
    running_ = true;
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto stop() -> result<void, error_type> {
    dma_.CCR = dma_.CCR & ~dma_enable;
    if ((adc_.CR & cr_adstart) != 0U) {
      adc_.CR = adc_.CR | cr_adstp;
      if (!wait_clear(adc_.CR, cr_adstart | cr_adstp))
        return fail(acquisition_fault::stop_timeout,
                    hal::adc::error_kind::timeout);
    }
    adc_.CFGR &= ~((1U << 0U) | (1U << 1U));
    running_ = false;
    return result<void, error_type>::success();
  }

  // captured_at is the DMA block-completion time for every sample in this
  // block; this target cannot reconstruct per-trigger timestamps by itself.
  void on_dma_interrupt(instant captured_at = {}) noexcept {
    if (!running_)
      return;
    const std::uint32_t dma_status = dma_.CSR;
    dma_.CFCR = dma_status & dma_all_flags;
    if ((dma_status & dma_error_flags) != 0U) {
      dma_.CCR = dma_.CCR & ~dma_enable;
      fault_.store(acquisition_fault::dma_transfer,
                   std::memory_order_release);
      running_ = false;
      return;
    }
    if ((dma_status & dma_transfer_complete) == 0U)
      return;
    if ((adc_.ISR & isr_overrun) != 0U) {
      adc_.ISR = isr_overrun;
      fault_.store(acquisition_fault::adc_overrun,
                   std::memory_order_release);
    }
    for (std::size_t i = 0U; i < SampleCount; ++i)
      channel_.publish_from_isr(buffer_[i] & 0xFFFFU, captured_at);
    (void)start();
  }
  [[nodiscard]] bool running() const noexcept { return running_; }
  [[nodiscard]] acquisition_fault fault() const noexcept {
    return fault_.load(std::memory_order_acquire);
  }
  [[nodiscard]] ContinuousChannel<Characteristics, SampleCount> &
  channel() noexcept {
    return channel_;
  }

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
  static constexpr std::uint32_t isr_overrun{1U << 4U};
  static constexpr std::uint32_t dma_enable{1U};
  static constexpr std::uint32_t dma_transfer_complete_interrupt{1U << 8U};
  static constexpr std::uint32_t dma_transfer_error_interrupt{1U << 10U};
  static constexpr std::uint32_t dma_transfer_complete{1U << 8U};
  static constexpr std::uint32_t dma_error_flags{(1U << 10U) | (1U << 11U) |
                                                  (1U << 12U) | (1U << 14U)};
  static constexpr std::uint32_t dma_all_flags{(0x7FU << 8U)};

  [[nodiscard]] bool wait_set(volatile const std::uint32_t &register_value,
                              std::uint32_t mask) const noexcept {
    for (std::uint32_t remaining = config_.timeout.iterations; remaining > 0U;
         --remaining)
      if ((register_value & mask) == mask)
        return true;
    return false;
  }
  [[nodiscard]] bool wait_clear(volatile const std::uint32_t &register_value,
                                std::uint32_t mask) const noexcept {
    for (std::uint32_t remaining = config_.timeout.iterations; remaining > 0U;
         --remaining)
      if ((register_value & mask) == 0U)
        return true;
    return false;
  }

  [[nodiscard]] auto fail(acquisition_fault fault,
                          hal::adc::error_kind kind)
      -> result<void, error_type> {
    dma_.CCR = dma_.CCR & ~dma_enable;
    running_ = false;
    fault_.store(fault, std::memory_order_release);
    return result<void, error_type>::failure(error{kind});
  }
  [[nodiscard]] auto failure(hal::adc::error_kind k)
      -> result<void, error_type> {
    return result<void, error_type>::failure(error{k});
  }
  template <class T>
  [[nodiscard]] auto failure(hal::adc::error_kind k) -> result<T, error_type> {
    return result<T, error_type>::failure(error{k});
  }
  AdcRegisters &adc_;
  AdcCommonRegisters &common_;
  DmaChannel &dma_;
  volatile std::uint32_t (&buffer_)[SampleCount];
  dma_config config_{};
  ContinuousChannel<Characteristics, SampleCount> channel_{};
  std::atomic<acquisition_fault> fault_{acquisition_fault::none};
  bool initialized_{};
  bool running_{};
};

} // namespace hal::stm32h5::adc
