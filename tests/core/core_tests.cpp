#include <array>
#include <cstddef>
#include <cstdint>
#include <hal/contracts/can.hpp>
#include <hal/foundation/crc32.hpp>
#include <hal/foundation/result.hpp>
#include <hal/foundation/span.hpp>
#include <hal/contracts/i2c.hpp>
#include <hal/contracts/spi.hpp>
#include <utility>

namespace {

enum class TestError : std::uint8_t { failed };

int failures = 0;

void check(bool condition) {
  if (!condition) {
    ++failures;
  }
}

void test_span_and_result() {
  std::array<int, 3U> values{1, 2, 3};
  hal::span<int> view{values};
  hal::span<const int> const_view{view};
  check(view.valid());
  check(view.size() == 3U);
  check(view[1U] == 2);
  check(const_view[2U] == 3);

  hal::span<int> empty{};
  check(empty.valid());
  check(empty.begin() == empty.end());

  auto success = hal::result<int, TestError>::success(42);
  check(success.has_value());
  check(success.value() == 42);

  auto failure = hal::result<int, TestError>::failure(TestError::failed);
  check(!failure);
  check(failure.error() == TestError::failed);

  auto void_success = hal::result<void, TestError>::success();
  check(void_success.has_value());
}

void test_can_values() {
  const hal::can::id standard{0x321U, false};
  const hal::can::id extended{0x1ABCDEU, true};
  check(standard.valid());
  check(extended.valid());
  check(!hal::can::id{0x800U, false}.valid());

  std::array<std::byte, 8U> payload{};
  auto data_frame = hal::can::classic_frame::make(standard, payload);
  check(data_frame.has_value());
  check(data_frame.value().size() == 8U);

  std::array<std::byte, 12U> fd_payload{};
  auto fd = hal::can::fd_frame::make(extended, fd_payload, true, false);
  check(fd.has_value());
  check(fd.value().dlc() == 9U);
  check(fd.value().bit_rate_switch());
  check(!hal::can::fd_frame::valid_payload_length(10U));

  const hal::can::filter filter{{0x120U, false}, 0x7F0U};
  check(filter.valid());
  check(filter.matches({0x12FU, false}));
  check(!filter.matches({0x12FU, true}));
}

void test_transaction_descriptors() {
  std::array<std::byte, 2U> mutable_data{};
  const std::array<std::byte, 2U> const_data{};

  const auto i2c_write = hal::i2c::operation::write(const_data);
  const auto i2c_read = hal::i2c::operation::read(mutable_data);
  check(i2c_write.valid());
  check(i2c_write.dir() == hal::i2c::direction::write);
  check(i2c_read.valid());
  check(i2c_read.dir() == hal::i2c::direction::read);

  const auto spi_transfer =
      hal::spi::operation8::transfer(const_data, mutable_data);
  const auto spi_delay = hal::spi::operation8::delay_for({50U});
  check(spi_transfer.valid());
  check(spi_delay.valid());
}

void test_crc32() {
  constexpr char text[] = "123456789";
  std::array<std::byte, 9U> bytes{};
  for (std::size_t index = 0U; index < bytes.size(); ++index) {
    bytes[index] =
        static_cast<std::byte>(static_cast<unsigned char>(text[index]));
  }
  check(hal::crc32(bytes) == 0xCBF43926U);
}

}  // namespace

int main() {
  test_span_and_result();
  test_can_values();
  test_transaction_descriptors();
  test_crc32();
  return failures;
}
