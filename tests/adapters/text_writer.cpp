#include <array>
#include <cstddef>
#include <hal/adapters/serial_text_writer.hpp>

namespace {

int failures = 0;

void check(bool condition) {
  if (!condition) {
    ++failures;
  }
}

struct Error {
  [[nodiscard]] constexpr hal::serial::error_kind kind() const noexcept {
    return hal::serial::error_kind::io;
  }
};

struct Transmitter {
  using error_type = Error;

  std::array<std::byte, 16U> bytes{};
  std::size_t size{};
  std::size_t acceptance_limit{16U};
  bool flushed{};

  [[nodiscard]] auto try_write(hal::span<const std::byte> input)
      -> hal::result<std::size_t, error_type> {
    const std::size_t available =
        acceptance_limit < bytes.size() ? acceptance_limit : bytes.size();
    const std::size_t accepted =
        input.size() < available ? input.size() : available;
    for (std::size_t index = 0U; index < accepted; ++index) {
      bytes[index] = input[index];
    }
    size = accepted;
    return hal::result<std::size_t, error_type>::success(accepted);
  }

  [[nodiscard]] auto flush() -> hal::result<void, error_type> {
    flushed = true;
    return hal::result<void, error_type>::success();
  }
};

void test_text_writer() {
  Transmitter transmitter{};
  transmitter.acceptance_limit = 5U;
  hal::serial::TextWriter writer{transmitter};

  auto written = writer.try_write("Hello world");
  check(written.has_value());
  check(written.value() == 5U);
  check(transmitter.size == 5U);
  check(transmitter.bytes[0U] == static_cast<std::byte>('H'));
  check(transmitter.bytes[4U] == static_cast<std::byte>('o'));
  check(writer.flush().has_value());
  check(transmitter.flushed);
}

}  // namespace

int main() {
  test_text_writer();
  return failures;
}
