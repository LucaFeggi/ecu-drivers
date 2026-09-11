#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <hal/contracts/adc.hpp>
#include <limits>
#include <type_traits>

namespace hal::adc {

using averaged_raw_sample_uint = sample<std::uint32_t>;
using averaged_raw_sample_float = sample<float>;

template <class Value>
struct average_output {
  bool available{};
  sample<Value> value{};
};

// Emits one result for each non-overlapping Samples input values.
template <std::size_t Samples, class ResultValue = float>
class BlockAverage {
  static_assert(Samples > 0U);
  static_assert(Samples <= std::numeric_limits<std::uint32_t>::max());
  static_assert(std::same_as<ResultValue, float> ||
                std::same_as<ResultValue, std::uint32_t>);

 public:
  [[nodiscard]] constexpr average_output<ResultValue> push(
      raw_sample input) noexcept {
    sum_ += input.value;
    ++count_;
    if (count_ != Samples) {
      return {};
    }

    const auto average = to_result(sum_);
    sum_ = 0U;
    count_ = 0U;
    return {true, {average, input.captured_at, input.sequence}};
  }

  constexpr void reset() noexcept {
    sum_ = 0U;
    count_ = 0U;
  }

 private:
  [[nodiscard]] static constexpr ResultValue to_result(
      std::uint64_t sum) noexcept {
    if constexpr (std::same_as<ResultValue, float>) {
      return static_cast<float>(sum) / static_cast<float>(Samples);
    } else {
      return static_cast<std::uint32_t>((sum + (Samples / 2U)) / Samples);
    }
  }

  std::uint64_t sum_{};
  std::size_t count_{};
};

// Emits one result per input after the first complete Samples-value window.
template <std::size_t Samples, class ResultValue = float>
class MovingAverage {
  static_assert(Samples > 0U);
  static_assert(Samples <= std::numeric_limits<std::uint32_t>::max());
  static_assert(std::same_as<ResultValue, float> ||
                std::same_as<ResultValue, std::uint32_t>);

 public:
  [[nodiscard]] constexpr average_output<ResultValue> push(
      raw_sample input) noexcept {
    if (count_ < Samples) {
      values_[next_] = input.value;
      sum_ += input.value;
      ++count_;
    } else {
      sum_ -= values_[next_];
      values_[next_] = input.value;
      sum_ += input.value;
    }

    next_ = (next_ + 1U) % Samples;
    if (count_ != Samples) {
      return {};
    }

    const auto average = to_result(sum_);
    return {true, {average, input.captured_at, input.sequence}};
  }

  constexpr void reset() noexcept {
    values_.fill(0U);
    sum_ = 0U;
    count_ = 0U;
    next_ = 0U;
  }

 private:
  [[nodiscard]] static constexpr ResultValue to_result(
      std::uint64_t sum) noexcept {
    if constexpr (std::same_as<ResultValue, float>) {
      return static_cast<float>(sum) / static_cast<float>(Samples);
    } else {
      return static_cast<std::uint32_t>((sum + (Samples / 2U)) / Samples);
    }
  }

  std::array<std::uint32_t, Samples> values_{};
  std::uint64_t sum_{};
  std::size_t count_{};
  std::size_t next_{};
};

}  // namespace hal::adc
