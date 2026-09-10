#pragma once
#include <hal/serial.hpp>
#include <string_view>

namespace hal::serial {

// Allocation-free, formatting-agnostic text view over a serial transmitter.
// Backpressure is preserved: try_write never retries or waits for the suffix.
template <Tx Transmitter>
class TextWriter {
 public:
  using error_type = typename Transmitter::error_type;

  explicit constexpr TextWriter(Transmitter& transmitter) noexcept
      : transmitter_{transmitter} {}

  [[nodiscard]] auto try_write(std::string_view text)
      -> result<std::size_t, error_type> {
    const auto* data = reinterpret_cast<const std::byte*>(text.data());
    return transmitter_.try_write(span<const std::byte>{data, text.size()});
  }

  template <std::size_t Size>
  [[nodiscard]] auto try_write(const char (&text)[Size])
      -> result<std::size_t, error_type> {
    static_assert(Size > 0U);
    return try_write(std::string_view{text, Size - 1U});
  }

  [[nodiscard]] auto flush() -> result<void, error_type> {
    return transmitter_.flush();
  }

  [[nodiscard]] constexpr Transmitter& underlying() noexcept {
    return transmitter_;
  }

 private:
  Transmitter& transmitter_;
};

template <Tx Transmitter>
TextWriter(Transmitter&) -> TextWriter<Transmitter>;

}  // namespace hal::serial
