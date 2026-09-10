#include <cstdint>
#include <hal/adc/averaging.hpp>

namespace {

struct Error {
  hal::adc::error_kind kind() const noexcept;
};

struct Channel {
  using error_type = Error;

  static hal::adc::characteristics properties() noexcept;
  hal::result<hal::adc::raw_sample, error_type> read_raw();
  hal::result<hal::adc::voltage_sample, error_type> read_voltage();
  hal::result<hal::adc::stream_read, error_type> try_read(
      hal::span<hal::adc::raw_sample> output);
};

static_assert(hal::adc::Channel<Channel>);
static_assert(hal::adc::ContinuousChannel<Channel>);

}  // namespace

int main() {
  hal::adc::BlockAverage<2U> block;
  if (block.push({10U, {1U}, 1U}).available) {
    return 1;
  }
  const auto block_average = block.push({11U, {2U}, 2U});
  if (!block_average.available || block_average.value.value != 10.5F) {
    return 2;
  }

  hal::adc::BlockAverage<2U, std::uint32_t> integer_block;
  const auto incomplete_integer_average = integer_block.push({10U, {1U}, 1U});
  if (incomplete_integer_average.available) {
    return 3;
  }
  const auto integer_average = integer_block.push({11U, {2U}, 2U});
  if (!integer_average.available || integer_average.value.value != 11U) {
    return 4;
  }

  hal::adc::MovingAverage<2U> moving;
  if (moving.push({10U, {1U}, 1U}).available) {
    return 5;
  }
  const auto first_moving_average = moving.push({12U, {2U}, 2U});
  const auto second_moving_average = moving.push({15U, {3U}, 3U});
  if (!first_moving_average.available ||
      first_moving_average.value.value != 11.0F ||
      !second_moving_average.available ||
      second_moving_average.value.value != 13.5F) {
    return 6;
  }

  // Integer output rounds instead of truncating its fractional raw code.
  hal::adc::BlockAverage<4U, std::uint32_t> rounding;
  (void)rounding.push({10U, {}, 1U});
  (void)rounding.push({10U, {}, 2U});
  (void)rounding.push({10U, {}, 3U});
  const auto rounded = rounding.push({11U, {4U}, 4U});
  if (!rounded.available || rounded.value.value != 10U ||
      rounded.value.sequence != 4U) {
    return 7;
  }

  hal::adc::MovingAverage<3U> resettable;
  (void)resettable.push({1U, {}, 1U});
  (void)resettable.push({2U, {}, 2U});
  resettable.reset();
  if (resettable.push({9U, {}, 3U}).available ||
      resettable.push({12U, {}, 4U}).available) {
    return 8;
  }
  const auto after_reset = resettable.push({15U, {5U}, 5U});
  if (!after_reset.available || after_reset.value.value != 12.0F ||
      after_reset.value.sequence != 5U) {
    return 9;
  }

  hal::adc::MovingAverage<2U> maximum_codes;
  (void)maximum_codes.push({0xFFFF'FFFFU, {}, 1U});
  const auto maximum_average = maximum_codes.push({0xFFFF'FFFFU, {}, 2U});
  if (!maximum_average.available ||
      maximum_average.value.value != 4'294'967'295.0F) {
    return 10;
  }

  return 0;
}
