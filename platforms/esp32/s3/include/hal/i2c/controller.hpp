#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <concepts>
#include <hal/foundation/result.hpp>
#include <hal/foundation/span.hpp>
#include <hal/foundation/units.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/i2c.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif
#include <soc/i2c_struct.h>

namespace hal::esp32s3_wroom_1_n16r8::i2c {

class error {
 public:
  [[nodiscard]] static constexpr error configuration() noexcept {
    return error{hal::i2c::error_kind::configuration};
  }
  [[nodiscard]] static constexpr error busy() noexcept {
    return error{hal::i2c::error_kind::busy};
  }
  [[nodiscard]] static constexpr error timeout() noexcept {
    return error{hal::i2c::error_kind::timeout};
  }
  [[nodiscard]] static constexpr error nack() noexcept {
    return error{hal::i2c::error_kind::no_acknowledge};
  }
  [[nodiscard]] static constexpr error arbitration() noexcept {
    return error{hal::i2c::error_kind::arbitration_lost};
  }
  [[nodiscard]] static constexpr error bus() noexcept {
    return error{hal::i2c::error_kind::bus_error};
  }
  [[nodiscard]] constexpr hal::i2c::error_kind kind() const noexcept {
    return kind_;
  }

 private:
  constexpr explicit error(hal::i2c::error_kind kind) noexcept : kind_{kind} {}
  hal::i2c::error_kind kind_;
};

struct config {
  hertz bus_frequency{400'000U};
  std::uint8_t digital_filter_cycles{};
  std::uint32_t poll_limit{1'000'000U};
};

enum class opcode : std::uint8_t { restart = 6U, write = 1U, read = 3U, stop = 2U };

struct command {
  opcode operation{};
  std::uint8_t bytes{};
  bool check_ack{};
  bool expected_ack{};
  bool sent_ack{};
  span<std::byte> receive{};
};

[[nodiscard]] constexpr std::uint32_t encode(command value) noexcept {
  return static_cast<std::uint32_t>(value.bytes) |
         (value.check_ack ? 1U << 8U : 0U) |
         (value.expected_ack ? 1U << 9U : 0U) |
         (value.sent_ack ? 1U << 10U : 0U) |
         (static_cast<std::uint32_t>(value.operation) << 11U);
}

template <std::uint32_t SourceHz = 40'000'000U>
class Controller {
 public:
  using error_type = error;

  Controller(i2c_dev_t& peripheral, config configuration = {}) noexcept
      : peripheral_{&peripheral}, configuration_{configuration} {}

  [[nodiscard]] result<hertz, error_type> configure(
      config configuration) noexcept {
    if (configuration.bus_frequency.value == 0U ||
        configuration.bus_frequency.value > SourceHz ||
        configuration.poll_limit == 0U || configuration.digital_filter_cycles > 15U) {
      return result<hertz, error_type>::failure(error_type::configuration());
    }
    // The ESP32-S3 I2C clock divider feeds the timing FSM; programming only
    // this divider leaves the SCL low/high period registers at reset values.
    // Derive the complete timing tuple using the same clock model as the
    // silicon implementation: one low period plus one high period per bit.
    const std::uint64_t requested = configuration.bus_frequency.value;
    const std::uint32_t divider = static_cast<std::uint32_t>(
        static_cast<std::uint64_t>(SourceHz) / (requested * 1024ULL) + 1ULL);
    if (divider == 0U || divider > 256U) {
      return result<hertz, error_type>::failure(error_type::configuration());
    }
    const std::uint32_t module_hz = SourceHz / divider;
    const std::uint32_t half_cycle =
        static_cast<std::uint32_t>(module_hz / (2ULL * requested));
    if (half_cycle < 4U || half_cycle > 512U) {
      return result<hertz, error_type>::failure(error_type::configuration());
    }
    const std::uint32_t wait_high =
        requested >= 80'000U ? half_cycle / 2U - 2U : half_cycle / 4U;
    const std::uint32_t high_period = half_cycle - wait_high;
    const std::uint32_t sda_sample = half_cycle / 2U;
    if (wait_high >= sda_sample || sda_sample >= high_period) {
      return result<hertz, error_type>::failure(error_type::configuration());
    }

    peripheral_->clk_conf.sclk_sel = 0U;  // XTAL clock
    peripheral_->clk_conf.sclk_active = 1U;
    peripheral_->clk_conf.val =
        (peripheral_->clk_conf.val & ~0x0000'00FFU) | (divider - 1U);
    peripheral_->clk_conf.sclk_div_a = 0U;
    peripheral_->clk_conf.sclk_div_b = 0U;
    const std::uint32_t period_minus_one = half_cycle - 1U;
    peripheral_->scl_low_period.val = period_minus_one & 0x1FFU;
    peripheral_->scl_high_period.val =
        (high_period & 0x1FFU) | ((wait_high & 0x7FU) << 9U);
    peripheral_->sda_hold.val = (half_cycle / 4U - 1U) & 0x1FFU;
    peripheral_->sda_sample.val = (sda_sample - 1U) & 0x1FFU;
    peripheral_->scl_rstart_setup.val = period_minus_one & 0x1FFU;
    peripheral_->scl_stop_setup.val = period_minus_one & 0x1FFU;
    peripheral_->scl_start_hold.val = period_minus_one & 0x1FFU;
    peripheral_->scl_stop_hold.val = period_minus_one & 0x1FFU;
    peripheral_->filter_cfg.val =
        (peripheral_->filter_cfg.val & ~0x0000'03FFU) |
        (static_cast<std::uint32_t>(configuration.digital_filter_cycles) << 0U) |
        (static_cast<std::uint32_t>(configuration.digital_filter_cycles) << 4U);
    peripheral_->filter_cfg.scl_filter_en = configuration.digital_filter_cycles != 0U;
    peripheral_->filter_cfg.sda_filter_en = configuration.digital_filter_cycles != 0U;
    peripheral_->ctr.ms_mode = 1U;
    peripheral_->ctr.sda_force_out = 1U;
    peripheral_->ctr.scl_force_out = 1U;
    peripheral_->to.time_out_en = 1U;
    peripheral_->to.time_out_value = 31U;
    peripheral_->ctr.conf_upgate = 1U;
    configuration_ = configuration;

    configured_ = true;
    return result<hertz, error_type>::success(
        hertz{module_hz / (2U * half_cycle)});
  }

  template <class Address>
  [[nodiscard]] result<void, error_type> transaction(
      Address address, span<const hal::i2c::operation> operations) noexcept
    requires hal::i2c::Address<Address>
  {
    if (!configured_ || !address.valid() || !operations.valid() ||
        operations.empty() ||
        configuration_.poll_limit == 0U) {
      return result<void, error_type>::failure(error_type::configuration());
    }
    if (peripheral_->sr.bus_busy != 0U) {
      return result<void, error_type>::failure(error_type::busy());
    }

    std::array<command, 8U> commands{};
    std::size_t command_count = 0U;
    std::array<std::byte, 32U> tx_fifo{};
    std::size_t tx_count = 0U;
    std::size_t previous_direction = 255U;

    auto append = [&](command value) noexcept -> bool {
      if (command_count >= commands.size()) {
        return false;
      }
      commands[command_count++] = value;
      return true;
    };

    // The controller's address phase is represented as normal WRITE/READ
    // bytes. This preserves repeated starts when operation direction changes.
    for (std::size_t op_index = 0U; op_index < operations.size(); ++op_index) {
      const hal::i2c::operation& operation = operations[op_index];
      if (!operation.valid() || operation.size() > 32U) {
        return result<void, error_type>::failure(error_type::configuration());
      }
      const std::size_t direction =
          operation.dir() == hal::i2c::direction::write ? 0U : 1U;
      const bool direction_changed = direction != previous_direction;
      if (direction_changed) {
        if (!append({opcode::restart, 0U, false, false, false, {}})) {
          return result<void, error_type>::failure(error_type::configuration());
        }
        if constexpr (std::same_as<Address, hal::i2c::address7>) {
          const std::byte address_byte = static_cast<std::byte>(
              static_cast<std::uint8_t>((address.value << 1U) | direction));
          if (tx_count >= tx_fifo.size()) {
            return result<void, error_type>::failure(error_type::configuration());
          }
          tx_fifo[tx_count++] = address_byte;
          if (!append({opcode::write, 1U, true, false, false, {}})) {
            return result<void, error_type>::failure(error_type::configuration());
          }
        } else {
          const std::uint8_t prefix = static_cast<std::uint8_t>(
              0xF0U | ((address.value >> 7U) & 0x06U));
          if (direction == 0U) {
            if (tx_count + 2U > tx_fifo.size()) {
              return result<void, error_type>::failure(error_type::configuration());
            }
            tx_fifo[tx_count++] = static_cast<std::byte>(prefix);
            tx_fifo[tx_count++] = static_cast<std::byte>(address.value & 0xFFU);
            if (!append({opcode::write, 2U, true, false, false, {}})) {
              return result<void, error_type>::failure(error_type::configuration());
            }
          } else {
            // A first 10-bit read requires the write prefix and low address
            // byte, followed by a repeated start and the read prefix. For a
            // later read group only the read prefix is needed; the outer
            // restart above already separates it from the previous group.
            if (previous_direction == 255U) {
              if (tx_count + 2U > tx_fifo.size()) {
                return result<void, error_type>::failure(error_type::configuration());
              }
              tx_fifo[tx_count++] = static_cast<std::byte>(prefix);
              tx_fifo[tx_count++] = static_cast<std::byte>(address.value & 0xFFU);
              if (!append({opcode::write, 2U, true, false, false, {}}) ||
                  !append({opcode::restart, 0U, false, false, false, {}})) {
                return result<void, error_type>::failure(error_type::configuration());
              }
            }
            if (tx_count >= tx_fifo.size()) {
              return result<void, error_type>::failure(error_type::configuration());
            }
            tx_fifo[tx_count++] = static_cast<std::byte>(prefix | 1U);
            if (!append({opcode::write, 1U, true, false, false, {}})) {
              return result<void, error_type>::failure(error_type::configuration());
            }
          }
        }
        previous_direction = direction;
      }

      if (operation.size() != 0U) {
        if (direction == 0U) {
          if (tx_count + operation.size() > tx_fifo.size()) {
            return result<void, error_type>::failure(error_type::configuration());
          }
          for (std::size_t index = 0U; index < operation.size(); ++index) {
            tx_fifo[tx_count++] = operation.write_buffer()[index];
          }
          if (!append({opcode::write, static_cast<std::uint8_t>(operation.size()),
                       true, false, false, {}})) {
            return result<void, error_type>::failure(error_type::configuration());
          }
        } else {
          const bool final_read = op_index + 1U == operations.size() ||
                                  operations[op_index + 1U].dir() !=
                                      hal::i2c::direction::read;
          if (!append({opcode::read, static_cast<std::uint8_t>(operation.size()),
                       false, false, final_read, operation.read_buffer()})) {
            return result<void, error_type>::failure(error_type::configuration());
          }
        }
      }
    }
    if (!append({opcode::stop, 0U, false, false, false, {}})) {
      return result<void, error_type>::failure(error_type::configuration());
    }

    peripheral_->fifo_conf.rx_fifo_rst = 1U;
    peripheral_->fifo_conf.rx_fifo_rst = 0U;
    peripheral_->fifo_conf.tx_fifo_rst = 1U;
    peripheral_->fifo_conf.tx_fifo_rst = 0U;
    for (std::size_t index = 0U; index < tx_count; ++index) {
      peripheral_->data.val = std::to_integer<unsigned char>(tx_fifo[index]);
    }
    peripheral_->int_clr.val = 0xFFFFFFFFU;
    peripheral_->ctr.trans_start = 0U;
    for (std::size_t index = 0U; index < command_count; ++index) {
      peripheral_->comd[index].val = encode(commands[index]);
    }
    peripheral_->int_ena.trans_complete_int_ena = 1U;
    peripheral_->int_ena.nack_int_ena = 1U;
    peripheral_->int_ena.arbitration_lost_int_ena = 1U;
    peripheral_->int_ena.time_out_int_ena = 1U;
    peripheral_->ctr.trans_start = 1U;

    std::uint32_t remaining = configuration_.poll_limit;
    while (remaining != 0U && peripheral_->int_raw.trans_complete_int_raw == 0U &&
           peripheral_->int_raw.nack_int_raw == 0U &&
           peripheral_->int_raw.arbitration_lost_int_raw == 0U &&
           peripheral_->int_raw.time_out_int_raw == 0U) {
      --remaining;
    }
    const bool nack = peripheral_->int_raw.nack_int_raw != 0U;
    const bool arbitration = peripheral_->int_raw.arbitration_lost_int_raw != 0U;
    const bool timed_out = peripheral_->int_raw.time_out_int_raw != 0U;
    peripheral_->int_clr.val = 0xFFFFFFFFU;
    if (remaining == 0U) {
      return result<void, error_type>::failure(error_type::timeout());
    }
    if (nack) {
      return result<void, error_type>::failure(error_type::nack());
    }
    if (arbitration) {
      return result<void, error_type>::failure(error_type::arbitration());
    }
    if (timed_out) {
      return result<void, error_type>::failure(error_type::timeout());
    }
    for (std::size_t index = 0U; index < command_count; ++index) {
      if (commands[index].operation == opcode::read) {
        for (std::size_t byte = 0U; byte < commands[index].bytes; ++byte) {
          commands[index].receive[byte] =
              static_cast<std::byte>(peripheral_->data.val & 0xFFU);
        }
      }
    }
    return result<void, error_type>::success();
  }

 private:
  i2c_dev_t* peripheral_;
  config configuration_;
  bool configured_{};
};

}  // namespace hal::esp32s3_wroom_1_n16r8::i2c
