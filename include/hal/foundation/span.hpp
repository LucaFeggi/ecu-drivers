#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <hal/foundation/assert.hpp>

namespace hal {

// A non-owning view over contiguous elements. Raw pointer/count construction
// requires a non-empty span to point at that many valid elements.
template <class T>
class span {
 public:
  using element_type = T;
  using pointer = T*;
  using reference = T&;

  constexpr span() noexcept = default;
  constexpr span(pointer data, std::size_t size) noexcept
      : data_{data}, size_{size} {}

  template <std::size_t N>
  constexpr span(T (&array)[N]) noexcept : data_{array}, size_{N} {}

  template <class U, std::size_t N>
    requires std::convertible_to<U (*)[], T (*)[]>
  constexpr span(std::array<U, N>& array) noexcept
      : data_{array.data()}, size_{N} {}

  template <class U, std::size_t N>
    requires std::convertible_to<const U (*)[], T (*)[]>
  constexpr span(const std::array<U, N>& array) noexcept
      : data_{array.data()}, size_{N} {}

  template <class U>
    requires std::convertible_to<U (*)[], T (*)[]>
  constexpr span(const span<U>& other) noexcept
      : data_{other.data()}, size_{other.size()} {}

  [[nodiscard]] constexpr bool valid() const noexcept {
    return size_ == 0U || data_ != nullptr;
  }
  [[nodiscard]] constexpr pointer data() const noexcept { return data_; }
  [[nodiscard]] constexpr std::size_t size() const noexcept { return size_; }
  [[nodiscard]] constexpr bool empty() const noexcept { return size_ == 0U; }

  [[nodiscard]] constexpr reference operator[](
      std::size_t index) const noexcept {
    HAL_CORE_ASSERT(valid() && index < size_);
    return data_[index];
  }

  [[nodiscard]] constexpr pointer begin() const noexcept { return data_; }

  [[nodiscard]] constexpr pointer end() const noexcept {
    return size_ == 0U ? data_ : data_ + size_;
  }

 private:
  pointer data_{};
  std::size_t size_{};
};

}  // namespace hal
