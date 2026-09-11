#include <array>
#include <cstddef>
#include <cstdint>
#include <hal/contracts/adc.hpp>

namespace {

class source_error {
  public:
    [[nodiscard]] constexpr hal::adc::error_kind kind() const noexcept {
        return hal::adc::error_kind::io;
    }
};

class source {
  public:
    using error_type = source_error;

    [[nodiscard]] static constexpr hal::adc::characteristics
    properties() noexcept {
        return {12U, false, hal::microvolts{0}, hal::microvolts{3'300'000}};
    }

    [[nodiscard]] auto latest() -> hal::result<hal::adc::sample, error_type> {
        return hal::result<hal::adc::sample, error_type>::success(values_[3]);
    }

    [[nodiscard]] auto try_read(hal::span<hal::adc::sample> output)
        -> hal::result<hal::adc::stream_read, error_type> {
        std::size_t copied = 0U;
        while (next_ < values_.size() && copied < output.size()) {
            output[copied] = values_[next_];
            ++copied;
            ++next_;
        }
        return hal::result<hal::adc::stream_read, error_type>::success(
            {copied, 0U});
    }

  private:
    std::array<hal::adc::sample, 4U> values_{
        hal::adc::sample{10U, hal::instant{1U}, 1U},
        hal::adc::sample{20U, hal::instant{2U}, 2U},
        hal::adc::sample{30U, hal::instant{3U}, 3U},
        hal::adc::sample{40U, hal::instant{4U}, 4U}};
    std::size_t next_{};
};

class gapped_source {
  public:
    using error_type = source_error;

    [[nodiscard]] static constexpr hal::adc::characteristics
    properties() noexcept {
        return source::properties();
    }

    [[nodiscard]] auto latest() -> hal::result<hal::adc::sample, error_type> {
        return hal::result<hal::adc::sample, error_type>::success(
            {50U, hal::instant{5U}, 5U});
    }

    [[nodiscard]] auto try_read(hal::span<hal::adc::sample> output)
        -> hal::result<hal::adc::stream_read, error_type> {
        if (call_ == 0U) {
            output[0] = {10U, hal::instant{1U}, 1U};
            output[1] = {20U, hal::instant{2U}, 2U};
            ++call_;
            return hal::result<hal::adc::stream_read, error_type>::success(
                {2U, 0U});
        }
        if (call_ == 1U) {
            output[0] = {30U, hal::instant{3U}, 3U};
            ++call_;
            return hal::result<hal::adc::stream_read, error_type>::success(
                {1U, 1U});
        }
        if (call_ == 2U) {
            output[0] = {40U, hal::instant{4U}, 4U};
            output[1] = {50U, hal::instant{5U}, 5U};
            ++call_;
            return hal::result<hal::adc::stream_read, error_type>::success(
                {2U, 0U});
        }
        return hal::result<hal::adc::stream_read, error_type>::success(
            {0U, 0U});
    }

  private:
    std::size_t call_{};
};

static_assert(hal::adc::SampleStream<source>);
using block_channel = hal::adc::BlockMeanChannel<source, 2U, 4U, 2U>;
static_assert(hal::adc::Channel<block_channel>);
static_assert(hal::adc::SampleStream<block_channel>);

} // namespace

int main() {
    source samples;
    block_channel averaged{samples};

    const auto before = averaged.latest();
    if (before || before.error().kind() != hal::adc::error_kind::not_ready) {
        return 1;
    }

    const auto progress = averaged.poll();
    if (!progress || progress.value().consumed != 4U ||
        progress.value().produced != 2U) {
        return 2;
    }

    const auto latest = averaged.latest();
    if (!latest || latest.value().raw != 35U || latest.value().sequence != 2U ||
        latest.value().captured_at.nanoseconds_since_boot != 4U) {
        return 3;
    }

    std::array<hal::adc::sample, 2U> output{};
    const auto read = averaged.try_read(output);
    if (!read || read.value().count != 2U || read.value().dropped != 0U ||
        output[0].raw != 15U || output[1].raw != 35U) {
        return 4;
    }

    hal::adc::MovingMean<3U> moving;
    if (moving.push({1U, hal::instant{1U}, 1U}).available ||
        moving.push({2U, hal::instant{2U}, 2U}).available) {
        return 5;
    }
    const auto first_mean = moving.push({6U, hal::instant{3U}, 3U});
    const auto second_mean = moving.push({10U, hal::instant{4U}, 4U});
    if (!first_mean.available || first_mean.value.raw != 3U ||
        !second_mean.available || second_mean.value.raw != 6U) {
        return 6;
    }

    hal::adc::Decimator<2U> decimator;
    if (decimator.push({7U, hal::instant{1U}, 1U}).available) {
        return 7;
    }
    const auto decimated = decimator.push({8U, hal::instant{2U}, 2U});
    if (!decimated.available || decimated.value.raw != 8U ||
        decimated.value.sequence != 1U) {
        return 8;
    }

    gapped_source gapped_samples;
    hal::adc::BlockMeanChannel<gapped_source, 3U, 2U, 1U> gap_safe_mean{
        gapped_samples};
    const auto before_gap = gap_safe_mean.poll();
    const auto after_gap = gap_safe_mean.poll();
    if (!before_gap || before_gap.value().produced != 0U || !after_gap ||
        after_gap.value().source_dropped != 1U ||
        after_gap.value().produced != 0U) {
        return 9;
    }
    const auto completed_after_gap = gap_safe_mean.poll();
    const auto gap_safe_output = gap_safe_mean.latest();
    if (!completed_after_gap || completed_after_gap.value().produced != 1U ||
        !gap_safe_output || gap_safe_output.value().raw != 40U ||
        gap_safe_output.value().sequence != 1U) {
        return 10;
    }

    return 0;
}
