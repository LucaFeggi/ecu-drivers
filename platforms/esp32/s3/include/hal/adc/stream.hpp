#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/adc.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

namespace hal::esp32s3_wroom_1_n16r8::adc {

class error {
 public:
  explicit constexpr error(hal::adc::error_kind kind) noexcept : kind_{kind} {}
  [[nodiscard]] constexpr hal::adc::error_kind kind() const noexcept {
    return kind_;
  }

 private:
  hal::adc::error_kind kind_;
};

template <hal::adc::characteristics Characteristics, std::size_t Capacity>
class Stream {
  static_assert(Capacity > 1U);

 public:
  using error_type = error;

  [[nodiscard]] static constexpr hal::adc::characteristics properties() noexcept {
    return Characteristics;
  }

  Stream() noexcept = default;
  Stream(const Stream&) = delete;
  Stream& operator=(const Stream&) = delete;

  void publish(std::uint32_t raw, instant captured_at) noexcept {
    const std::uint64_t sequence = produced_ + 1U;
    produced_ = sequence;
    generation_.fetch_add(1U, std::memory_order_acq_rel);
    slot& destination = samples_[sequence % Capacity];
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

  [[nodiscard]] result<hal::adc::raw_sample, error_type> read_raw() noexcept {
    std::uint64_t sequence{};
    hal::adc::raw_sample value{};
    if (!snapshot(sequence, value)) {
      return result<hal::adc::raw_sample, error_type>::failure(
          error_type{hal::adc::error_kind::overrun});
    }
    if (sequence == 0U) {
      return result<hal::adc::raw_sample, error_type>::failure(
          error_type{hal::adc::error_kind::not_ready});
    }
    return result<hal::adc::raw_sample, error_type>::success(value);
  }

  [[nodiscard]] result<hal::adc::voltage_sample, error_type>
  read_voltage() noexcept {
    const auto raw = read_raw();
    if (!raw) {
      return result<hal::adc::voltage_sample, error_type>::failure(raw.error());
    }
    if (Characteristics.resolution_bits == 0U ||
        Characteristics.resolution_bits > 31U ||
        Characteristics.nominal_max.value < Characteristics.nominal_min.value) {
      return result<hal::adc::voltage_sample, error_type>::failure(
          error_type{hal::adc::error_kind::configuration});
    }
    const std::uint64_t code_count =
        (std::uint64_t{1U} << Characteristics.resolution_bits) - 1U;
    const std::int64_t voltage_span = Characteristics.nominal_max.value -
                                      Characteristics.nominal_min.value;
    const std::int64_t offset = static_cast<std::int64_t>(
        (static_cast<std::uint64_t>(raw.value().value) *
         static_cast<std::uint64_t>(voltage_span)) /
        code_count);
    return result<hal::adc::voltage_sample, error_type>::success(
        {microvolts{Characteristics.nominal_min.value + offset},
         raw.value().captured_at, raw.value().sequence});
  }

  [[nodiscard]] result<hal::adc::stream_read, error_type> try_read(
      span<hal::adc::raw_sample> output) noexcept {
    if (!output.valid()) {
      return result<hal::adc::stream_read, error_type>::failure(
          error_type{hal::adc::error_kind::other});
    }
    for (unsigned attempt = 0U; attempt < 3U; ++attempt) {
      const std::uint32_t before = generation_.load(std::memory_order_acquire);
      if ((before & 1U) != 0U) {
        continue;
      }
      const std::uint64_t published = published_sequence();
      std::uint64_t next = consumed_ + 1U;
      std::uint64_t dropped = 0U;
      if (published >= next && published - next >= Capacity) {
        const std::uint64_t oldest = published - Capacity + 1U;
        dropped = oldest - next;
        next = oldest;
      }
      std::size_t count = 0U;
      while (next <= published && count < output.size()) {
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

  [[nodiscard]] static hal::adc::raw_sample load(const slot& source) noexcept {
    const std::uint64_t timestamp =
        (static_cast<std::uint64_t>(source.timestamp_high.load(
             std::memory_order_relaxed))
         << 32U) |
        source.timestamp_low.load(std::memory_order_relaxed);
    const std::uint64_t sequence =
        (static_cast<std::uint64_t>(source.sequence_high.load(
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

  [[nodiscard]] bool snapshot(std::uint64_t& sequence,
                              hal::adc::raw_sample& value) const noexcept {
    for (unsigned attempt = 0U; attempt < 3U; ++attempt) {
      const std::uint32_t before = generation_.load(std::memory_order_acquire);
      if ((before & 1U) != 0U) {
        continue;
      }
      sequence = published_sequence();
      if (sequence != 0U) {
        value = load(samples_[sequence % Capacity]);
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
  std::uint64_t produced_{};
  std::uint64_t consumed_{};
};

}  // namespace hal::esp32s3_wroom_1_n16r8::adc
