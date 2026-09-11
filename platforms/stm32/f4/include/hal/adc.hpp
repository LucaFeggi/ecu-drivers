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

namespace hal::stm32f4::adc {

inline constexpr capability support{
    implementation_status::available,
    "ADC1/2/3 continuous circular DMA acquisition"};
class error {
public:
  explicit constexpr error(hal::adc::error_kind k) noexcept : kind_{k} {}
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
  void publish_from_isr(std::uint32_t raw, instant timestamp) noexcept {
    const auto next = produced_by_isr_ + 1U;
    produced_by_isr_ = next;
    generation_.fetch_add(1U, std::memory_order_acq_rel);
    auto &destination = values_[next % Capacity];
    destination.raw.store(raw, std::memory_order_relaxed);
    destination.timestamp_low.store(
        static_cast<std::uint32_t>(timestamp.nanoseconds_since_boot),
        std::memory_order_relaxed);
    destination.timestamp_high.store(
        static_cast<std::uint32_t>(timestamp.nanoseconds_since_boot >> 32U),
        std::memory_order_relaxed);
    destination.sequence_low.store(static_cast<std::uint32_t>(next),
                                   std::memory_order_relaxed);
    destination.sequence_high.store(static_cast<std::uint32_t>(next >> 32U),
                                    std::memory_order_relaxed);
    published_low_.store(static_cast<std::uint32_t>(next),
                         std::memory_order_relaxed);
    published_high_.store(static_cast<std::uint32_t>(next >> 32U),
                          std::memory_order_relaxed);
    generation_.fetch_add(1U, std::memory_order_release);
  }
  [[nodiscard]] auto read_raw() -> result<hal::adc::raw_sample, error_type> {
    std::uint64_t produced{};
    hal::adc::raw_sample value{};
    if (!snapshot(produced, value))
      return failure<hal::adc::raw_sample>(hal::adc::error_kind::overrun);
    if (produced == 0U)
      return failure<hal::adc::raw_sample>(hal::adc::error_kind::not_ready);
    return result<hal::adc::raw_sample, error_type>::success(value);
  }
  [[nodiscard]] auto read_voltage()
      -> result<hal::adc::voltage_sample, error_type> {
    auto raw = read_raw();
    if (!raw)
      return result<hal::adc::voltage_sample, error_type>::failure(raw.error());
    const auto count =
        (std::uint64_t{1U} << Characteristics.resolution_bits) - 1U;
    const auto span =
        Characteristics.nominal_max.value - Characteristics.nominal_min.value;
    return result<hal::adc::voltage_sample, error_type>::success(
        {hal::microvolts{
             Characteristics.nominal_min.value +
             static_cast<std::int64_t>(
                 (raw.value().value * static_cast<std::uint64_t>(span)) /
                 count)},
         raw.value().captured_at, raw.value().sequence});
  }
  [[nodiscard]] auto try_read(hal::span<hal::adc::raw_sample> output)
      -> result<hal::adc::stream_read, error_type> {
    HAL_CORE_ASSERT(output.valid());
    if (!output.valid())
      return failure<hal::adc::stream_read>(hal::adc::error_kind::other);
    for (unsigned attempt = 0U; attempt < 3U; ++attempt) {
      const auto before = generation_.load(std::memory_order_acquire);
      if ((before & 1U) != 0U)
        continue;
      const auto produced = published_sequence();
      auto next = consumed_ + 1U;
      std::uint64_t dropped = 0U;
      if (produced >= next && produced - next >= Capacity) {
        const auto oldest = produced - Capacity + 1U;
        dropped = oldest - next;
        next = oldest;
      }
      const auto available = produced >= next ? produced - next + 1U : 0U;
      const auto count = output.size() < available ? output.size() : available;
      for (std::size_t i = 0U; i < count; ++i)
        output[i] = load(values_[(next + i) % Capacity]);
      const auto after = generation_.load(std::memory_order_acquire);
      if (before == after) {
        consumed_ = next + count - 1U;
        return result<hal::adc::stream_read, error_type>::success(
            {count, dropped});
      }
    }
    return failure<hal::adc::stream_read>(hal::adc::error_kind::overrun);
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
  [[nodiscard]] bool snapshot(std::uint64_t &produced,
                              hal::adc::raw_sample &value) const noexcept {
    for (unsigned attempt = 0U; attempt < 3U; ++attempt) {
      const auto before = generation_.load(std::memory_order_acquire);
      if ((before & 1U) != 0U)
        continue;
      produced = published_sequence();
      if (produced != 0U)
        value = load(values_[produced % Capacity]);
      const auto after = generation_.load(std::memory_order_acquire);
      if (before == after)
        return true;
    }
    return false;
  }
  [[nodiscard]] std::uint64_t published_sequence() const noexcept {
    return (static_cast<std::uint64_t>(
                published_high_.load(std::memory_order_relaxed))
            << 32U) |
           published_low_.load(std::memory_order_relaxed);
  }
  template <class T>
  [[nodiscard]] static auto failure(hal::adc::error_kind k)
      -> result<T, error_type> {
    return result<T, error_type>::failure(error{k});
  }
  std::array<slot, Capacity> values_{};
  std::atomic<std::uint32_t> generation_{};
  std::atomic<std::uint32_t> published_low_{};
  std::atomic<std::uint32_t> published_high_{};
  std::uint64_t produced_by_isr_{};
  std::uint64_t consumed_{};
};

struct dma_config {
  std::uint32_t adc_clock_hz{};
  std::uint8_t channel{};
  std::uint8_t sample_time_code{};
  std::uint8_t dma_channel{};
  poll_budget timeout{};
  [[nodiscard]] constexpr bool valid() const noexcept {
    return adc_clock_hz != 0U && channel < 19U && sample_time_code < 8U &&
           timeout.valid();
  }
};

template <hal::adc::characteristics Characteristics, std::size_t SampleCount,
          class AdcRegisters, class CommonRegisters, class DmaStream>
class AdcDma {
  static_assert(SampleCount > 1U && SampleCount <= 65'535U);

public:
  using error_type = error;
  AdcDma(AdcRegisters &adc, CommonRegisters &common, DmaStream &dma,
         volatile std::uint16_t (&buffer)[SampleCount],
         dma_config config) noexcept
      : adc_{adc}, common_{common}, dma_{dma}, buffer_{buffer},
        config_{config} {}
  AdcDma(const AdcDma &) = delete;
  AdcDma &operator=(const AdcDma &) = delete;
  [[nodiscard]] auto initialize() -> result<void, error_type> {
    if (!config_.valid() || Characteristics.resolution_bits != 12U)
      return failure(hal::adc::error_kind::configuration);
    adc_.CR2 = 0U;
    adc_.CR1 = 0U;
    if (config_.channel < 10U) {
      const auto shift = static_cast<std::uint32_t>(config_.channel) * 3U;
      adc_.SMPR2 =
          (adc_.SMPR2 & ~(7U << shift)) |
          (static_cast<std::uint32_t>(config_.sample_time_code) << shift);
    } else {
      const auto shift =
          (static_cast<std::uint32_t>(config_.channel) - 10U) * 3U;
      adc_.SMPR1 =
          (adc_.SMPR1 & ~(7U << shift)) |
          (static_cast<std::uint32_t>(config_.sample_time_code) << shift);
    }
    adc_.SQR1 = 0U;
    adc_.SQR3 = config_.channel;
    adc_.CR2 = (1U << 8U) | (1U << 9U) | (1U << 1U) | (1U << 0U);
    configured_ = true;
    return result<void, error_type>::success();
  }
  [[nodiscard]] auto start() -> result<void, error_type> {
    if (!configured_)
      return failure(hal::adc::error_kind::configuration);
    dma_.CR &= ~1U;
    dma_.PAR =
        static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(&adc_.DR));
    dma_.M0AR =
        static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(buffer_));
    dma_.NDTR = static_cast<std::uint32_t>(SampleCount);
    dma_.FCR = 0U;
    dma_.CR = (static_cast<std::uint32_t>(config_.dma_channel) << 25U) |
              (1U << 8U) | (1U << 10U) | (1U << 4U) | (1U << 2U) | (1U << 11U) |
              (1U << 13U) | 1U;
    adc_.CR2 |= 1U << 30U;
    adc_.CR2 |= 1U << 22U;
    running_ = true;
    return result<void, error_type>::success();
  }
  [[nodiscard]] auto stop() -> result<void, error_type> {
    dma_.CR &= ~1U;
    adc_.CR2 &= ~(1U << 30U);
    running_ = false;
    return result<void, error_type>::success();
  }
  void on_dma_interrupt(instant timestamp = {}) noexcept {
    if (!running_)
      return;
    for (std::size_t i = 0U; i < SampleCount; ++i)
      channel_.publish_from_isr(buffer_[i], timestamp);
  }
  [[nodiscard]] auto &channel() noexcept { return channel_; }

private:
  [[nodiscard]] auto failure(hal::adc::error_kind k)
      -> result<void, error_type> {
    return result<void, error_type>::failure(error{k});
  }
  template <class T>
  [[nodiscard]] auto failure(hal::adc::error_kind k) -> result<T, error_type> {
    return result<T, error_type>::failure(error{k});
  }
  AdcRegisters &adc_;
  CommonRegisters &common_;
  DmaStream &dma_;
  volatile std::uint16_t (&buffer_)[SampleCount];
  dma_config config_{};
  ContinuousChannel<Characteristics, SampleCount> channel_{};
  bool configured_{};
  bool running_{};
};
} // namespace hal::stm32f4::adc
