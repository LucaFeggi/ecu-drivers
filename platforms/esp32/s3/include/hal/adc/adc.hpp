#pragma once

#include <cstddef>
#include <cstdint>
#include <hal/adc/stream.hpp>
#include <hal/dma/channel.hpp>
#include <hal/foundation/result.hpp>
#include <hal/foundation/span.hpp>
#include <hal/foundation/units.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/time.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif
#include <soc/apb_saradc_struct.h>
#include <soc/sens_struct.h>
#include <type_traits>

namespace hal::esp32s3_wroom_1_n16r8::adc {

enum class unit : std::uint8_t { one = 1U, two = 2U };

struct config {
  std::uint8_t attenuation{};  // 0..3, as encoded by the SAR analog block.
  std::uint32_t poll_limit{100'000U};
};

template <unit Unit, std::uint8_t Channel,
          hal::adc::characteristics Characteristics>
class ChannelReader {
  static_assert((Unit == unit::one || Unit == unit::two) && Channel < 10U);

 public:
  using error_type = error;

  ChannelReader(apb_saradc_dev_t& adc, sens_dev_t& sens,
                config configuration = {}) noexcept
      : adc_{&adc}, sens_{&sens}, configuration_{configuration} {}

  [[nodiscard]] static constexpr hal::adc::characteristics properties() noexcept {
    return Characteristics;
  }

  [[nodiscard]] result<void, error_type> configure() noexcept {
    if (configuration_.attenuation > 3U || configuration_.poll_limit == 0U) {
      return result<void, error_type>::failure(
          error_type{hal::adc::error_kind::configuration});
    }
    const std::uint32_t attenuation_shift = static_cast<std::uint32_t>(Channel) * 2U;
    if constexpr (Unit == unit::one) {
      adc_->ctrl.sar_clk_gated = 1U;
      adc_->ctrl.xpd_sar_force = 3U;
      sens_->sar_atten1 =
          (sens_->sar_atten1 & ~(0x3U << attenuation_shift)) |
          (static_cast<std::uint32_t>(configuration_.attenuation)
           << attenuation_shift);
      sens_->sar_meas1_mux.sar1_dig_force = 0U;
      sens_->sar_meas1_ctrl2.meas1_start_force = 1U;
      sens_->sar_meas1_ctrl2.sar1_en_pad_force = 1U;
      sens_->sar_meas1_ctrl2.sar1_en_pad = std::uint32_t{1U} << Channel;
      sens_->sar_reader1_ctrl.sar1_clk_div = 1U;
    } else {
      adc_->ctrl.sar_clk_gated = 1U;
      adc_->ctrl.xpd_sar_force = 3U;
      sens_->sar_atten2 =
          (sens_->sar_atten2 & ~(0x3U << attenuation_shift)) |
          (static_cast<std::uint32_t>(configuration_.attenuation)
           << attenuation_shift);
      sens_->sar_meas2_ctrl2.meas2_start_force = 1U;
      sens_->sar_meas2_ctrl2.sar2_en_pad_force = 1U;
      sens_->sar_meas2_ctrl2.sar2_en_pad = std::uint32_t{1U} << Channel;
      sens_->sar_reader2_ctrl.sar2_clk_div = 1U;
    }
    configured_ = true;
    return result<void, error_type>::success();
  }

  [[nodiscard]] result<hal::adc::raw_sample, error_type> read_raw() noexcept {
    if (!configured_) {
      return result<hal::adc::raw_sample, error_type>::failure(
          error_type{hal::adc::error_kind::not_ready});
    }
    if constexpr (Unit == unit::one) {
      std::uint32_t remaining = configuration_.poll_limit;
      while (remaining != 0U && sens_->sar_slave_addr1.meas_status != 0U) {
        --remaining;
      }
      if (remaining == 0U) {
        return result<hal::adc::raw_sample, error_type>::failure(
            error_type{hal::adc::error_kind::timeout});
      }
      sens_->sar_meas1_ctrl2.meas1_start_sar = 0U;
      sens_->sar_meas1_ctrl2.meas1_start_sar = 1U;
    } else {
      sens_->sar_meas2_ctrl2.meas2_start_sar = 0U;
      sens_->sar_meas2_ctrl2.meas2_start_sar = 1U;
    }
    std::uint32_t remaining = configuration_.poll_limit;
    while (remaining != 0U && !done()) {
      --remaining;
    }
    if (remaining == 0U) {
      return result<hal::adc::raw_sample, error_type>::failure(
          error_type{hal::adc::error_kind::timeout});
    }
    const std::uint32_t value = raw_value() & 0x0FFFU;
    return result<hal::adc::raw_sample, error_type>::success(
        {value, instant{}, ++sequence_});
  }

  [[nodiscard]] result<hal::adc::voltage_sample, error_type>
  read_voltage() noexcept {
    const auto raw = read_raw();
    if (!raw) {
      return result<hal::adc::voltage_sample, error_type>::failure(raw.error());
    }
    if (Characteristics.nominal_max.value < Characteristics.nominal_min.value) {
      return result<hal::adc::voltage_sample, error_type>::failure(
          error_type{hal::adc::error_kind::configuration});
    }
    const std::int64_t range = Characteristics.nominal_max.value -
                               Characteristics.nominal_min.value;
    const std::int64_t offset = static_cast<std::int64_t>(
        (static_cast<std::uint64_t>(raw.value().value) *
         static_cast<std::uint64_t>(range)) /
        4095U);
    return result<hal::adc::voltage_sample, error_type>::success(
        {microvolts{Characteristics.nominal_min.value + offset},
         raw.value().captured_at, raw.value().sequence});
  }

 private:
  [[nodiscard]] bool done() const noexcept {
    if constexpr (Unit == unit::one) {
      return sens_->sar_meas1_ctrl2.meas1_done_sar != 0U;
    }
    return sens_->sar_meas2_ctrl2.meas2_done_sar != 0U;
  }

  [[nodiscard]] std::uint32_t raw_value() const noexcept {
    if constexpr (Unit == unit::one) {
      return sens_->sar_meas1_ctrl2.meas1_data_sar;
    }
    return sens_->sar_meas2_ctrl2.meas2_data_sar;
  }

  apb_saradc_dev_t* adc_;
  sens_dev_t* sens_;
  config configuration_;
  std::uint64_t sequence_{};
  bool configured_{};
};

struct pattern {
  std::uint8_t channel{};
  std::uint8_t attenuation{};
};

struct continuous_config {
  span<const pattern> patterns{};
  hertz sample_frequency{};
  std::uint32_t poll_limit{100'000U};
};

struct no_clock {
  [[nodiscard]] instant now() const noexcept { return {}; }
};

template <hal::adc::characteristics Characteristics, std::size_t QueueCapacity,
          std::size_t DmaChannel = 2U, class Clock = no_clock>
class Continuous {
  static_assert(QueueCapacity > 1U);

 public:
  using error_type = error;

  Continuous(apb_saradc_dev_t& adc, sens_dev_t& sens, gdma_dev_t& gdma,
             Clock& clock, span<device::gdma_descriptor> descriptors,
             span<std::uint32_t> dma_samples) noexcept
      : adc_{&adc}, sens_{&sens}, dma_{gdma}, clock_{clock},
        descriptors_{descriptors}, dma_samples_{dma_samples} {}

  [[nodiscard]] static constexpr hal::adc::characteristics properties() noexcept {
    return Characteristics;
  }

  Continuous(const Continuous&) = delete;
  Continuous& operator=(const Continuous&) = delete;

  [[nodiscard]] result<hertz, error_type> configure(
      continuous_config configuration) noexcept {
    if (!configuration.patterns.valid() || configuration.patterns.empty() ||
        configuration.patterns.size() > 12U ||
        configuration.sample_frequency.value == 0U ||
        configuration.poll_limit == 0U || descriptors_.empty() ||
        dma_samples_.empty() || descriptors_.size() > dma_samples_.size() ||
        dma_samples_.size() % descriptors_.size() != 0U ||
        (reinterpret_cast<std::uintptr_t>(descriptors_.data()) % 4U) != 0U ||
        (reinterpret_cast<std::uintptr_t>(dma_samples_.data()) % 4U) != 0U) {
      return result<hertz, error_type>::failure(
          error_type{hal::adc::error_kind::configuration});
    }
    for (const pattern& selected : configuration.patterns) {
      if (selected.channel >= 10U || selected.attenuation > 3U) {
        return result<hertz, error_type>::failure(
            error_type{hal::adc::error_kind::configuration});
      }
    }

    const std::uint64_t requested = configuration.sample_frequency.value;
    const std::uint64_t maximum_controller =
        requested * 2ULL * 4095ULL < 80'000'000ULL
            ? requested * 2ULL * 4095ULL
            : 80'000'000ULL;
    std::uint64_t controller_divider =
        (80'000'000ULL + maximum_controller - 1ULL) / maximum_controller;
    if (controller_divider == 0ULL) {
      controller_divider = 1ULL;
    }
    const std::uint64_t interval =
        80'000'000ULL / (controller_divider * 2ULL * requested);
    if (controller_divider > 256ULL || interval < 30ULL || interval > 4095ULL) {
      return result<hertz, error_type>::failure(
          error_type{hal::adc::error_kind::configuration});
    }

    sens_->sar_meas1_mux.sar1_dig_force = 1U;
    sens_->sar_meas1_ctrl2.meas1_start_force = 1U;
    sens_->sar_meas1_ctrl2.sar1_en_pad_force = 1U;
    adc_->ctrl.sar_clk_gated = 1U;
    adc_->ctrl.xpd_sar_force = 3U;
    adc_->ctrl.sar1_patt_p_clear = 1U;
    adc_->ctrl.sar1_patt_p_clear = 0U;
    for (auto& table : adc_->sar1_patt_tab) {
      table.val = 0x00FF'FFFFU;
    }
    std::uint32_t enabled_channels = 0U;
    for (std::size_t index = 0U; index < configuration.patterns.size(); ++index) {
      const pattern selected = configuration.patterns[index];
      enabled_channels |= std::uint32_t{1U} << selected.channel;
      const std::size_t table = index / 4U;
      const std::uint32_t shift =
          18U - static_cast<std::uint32_t>((index % 4U) * 6U);
      const std::uint32_t encoded =
          static_cast<std::uint32_t>(selected.attenuation) |
          (static_cast<std::uint32_t>(selected.channel) << 2U);
      const std::uint32_t mask = 0x3FU << shift;
      adc_->sar1_patt_tab[table].val =
          (adc_->sar1_patt_tab[table].val & ~mask) | ((encoded & 0x3FU) << shift);
    }
    sens_->sar_meas1_ctrl2.val =
        (sens_->sar_meas1_ctrl2.val & ~(0x0FFFU << 18U)) |
        ((enabled_channels & 0x0FFFU) << 18U);
    adc_->ctrl.start_force = 1U;
    adc_->ctrl.work_mode = 0U;
    adc_->ctrl.sar_sel = 0U;
    adc_->ctrl.data_sar_sel = 1U;
    adc_->ctrl.val =
        (adc_->ctrl.val & ~(0x0FU << 14U)) |
        (static_cast<std::uint32_t>(configuration.patterns.size() - 1U) << 14U);
    adc_->ctrl2.meas_num_limit = 0U;
    adc_->ctrl2.timer_sel = 1U;
    adc_->ctrl2.val = (adc_->ctrl2.val & ~(0x0FFFU << 2U)) |
                      (static_cast<std::uint32_t>(interval) << 2U);
    adc_->ctrl2.timer_en = 0U;
    adc_->apb_adc_clkm_conf.val =
        (adc_->apb_adc_clkm_conf.val & ~0x0000'00FFU) |
        (static_cast<std::uint32_t>(controller_divider - 1ULL) & 0xFFU);
    adc_->apb_adc_clkm_conf.clkm_div_b = 1U;
    adc_->apb_adc_clkm_conf.clkm_div_a = 0U;
    adc_->apb_adc_clkm_conf.clk_sel = 2U;  // APB clock.
    adc_->apb_adc_clkm_conf.clk_en = 1U;

    records_per_descriptor_ = dma_samples_.size() / descriptors_.size();
    const std::size_t bytes = records_per_descriptor_ * sizeof(std::uint32_t);
    if (bytes == 0U || bytes > 4092U) {
      return result<hertz, error_type>::failure(
          error_type{hal::adc::error_kind::configuration});
    }
    for (std::size_t index = 0U; index < descriptors_.size(); ++index) {
      dma::make_rx_descriptor(descriptors_[index],
                              dma_samples_.data() + index * records_per_descriptor_,
                              bytes, false);
      descriptors_[index].next_descriptor = dma::address(
          &descriptors_[(index + 1U) % descriptors_.size()]);
    }
    configuration_ = configuration;
    configured_ = true;
    const std::uint64_t actual =
        80'000'000ULL / (controller_divider * 2ULL * interval);
    return result<hertz, error_type>::success(hertz{actual});
  }

  [[nodiscard]] result<void, error_type> start() noexcept {
    if (!configured_ || records_per_descriptor_ == 0U) {
      return result<void, error_type>::failure(
          error_type{hal::adc::error_kind::not_ready});
    }
    dma_.prepare_rx(dma::peripheral::adc_dac,
                    dma::address(descriptors_.data()));
    dma_.clear_rx_interrupts();
    adc_->dma_conf.val =
        (adc_->dma_conf.val & ~0x0000'FFFFU) |
        (static_cast<std::uint32_t>(records_per_descriptor_) & 0xFFFFU);
    adc_->dma_conf.apb_adc_reset_fsm = 1U;
    adc_->dma_conf.apb_adc_reset_fsm = 0U;
    adc_->dma_conf.apb_adc_trans = 1U;
    adc_->ctrl2.timer_en = 1U;
    dma_.start_rx();
    running_ = true;
    return result<void, error_type>::success();
  }

  [[nodiscard]] result<void, error_type> stop() noexcept {
    if (!configured_) {
      return result<void, error_type>::failure(
          error_type{hal::adc::error_kind::not_ready});
    }
    if (!running_) {
      return result<void, error_type>::success();
    }
    adc_->ctrl2.timer_en = 0U;
    adc_->dma_conf.apb_adc_trans = 0U;
    dma_.stop_rx();
    dma_.reset_rx();
    running_ = false;
    return result<void, error_type>::success();
  }

  // Called by the fixed GDMA ISR or by a polling task. It performs bounded
  // descriptor publication and leaves scheduling/notifications to firmware.
  void service() noexcept {
    if (!running_ || (dma_.rx_interrupts() & (1U << 1U)) == 0U) {
      return;
    }
    const std::uint32_t eof_address = dma_.rx_success_eof_descriptor();
    std::size_t eof_index = descriptors_.size();
    for (std::size_t index = 0U; index < descriptors_.size(); ++index) {
      if (dma::address(&descriptors_[index]) == eof_address) {
        eof_index = index;
        break;
      }
    }
    if (eof_index == descriptors_.size()) {
      dma_.clear_rx_interrupts();
      return;
    }
    std::size_t index = next_descriptor_;
    for (std::size_t processed = 0U; processed < descriptors_.size(); ++processed) {
      const instant timestamp = clock_.now();
      const std::size_t base = index * records_per_descriptor_;
      for (std::size_t sample = 0U; sample < records_per_descriptor_; ++sample) {
        const std::uint32_t word = dma_samples_[base + sample];
        stream_.publish(word & 0x0FFFU, timestamp);
      }
      descriptors_[index].control = device::descriptor_control(
          records_per_descriptor_ * sizeof(std::uint32_t), 0U, true, false);
      index = (index + 1U) % descriptors_.size();
      if (index == (eof_index + 1U) % descriptors_.size()) {
        break;
      }
    }
    next_descriptor_ = (eof_index + 1U) % descriptors_.size();
    dma_.clear_rx_interrupts();
  }

  [[nodiscard]] result<hal::adc::raw_sample, error_type> read_raw() noexcept {
    service();
    return stream_.read_raw();
  }

  [[nodiscard]] result<hal::adc::voltage_sample, error_type>
  read_voltage() noexcept {
    service();
    return stream_.read_voltage();
  }

  [[nodiscard]] result<hal::adc::stream_read, error_type> try_read(
      span<hal::adc::raw_sample> output) noexcept {
    service();
    return stream_.try_read(output);
  }

 private:
  apb_saradc_dev_t* adc_;
  sens_dev_t* sens_;
  dma::Channel<DmaChannel> dma_;
  Clock& clock_;
  span<device::gdma_descriptor> descriptors_;
  span<std::uint32_t> dma_samples_;
  continuous_config configuration_{};
  Stream<Characteristics, QueueCapacity> stream_{};
  std::size_t records_per_descriptor_{};
  std::size_t next_descriptor_{};
  bool configured_{};
  bool running_{};
};

}  // namespace hal::esp32s3_wroom_1_n16r8::adc
