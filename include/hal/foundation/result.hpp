#pragma once

#include <concepts>
#include <hal/foundation/assert.hpp>
#include <new>
#include <type_traits>
#include <utility>

namespace hal {

// Allocation-free expected-like value for the C++20 baseline.
template <class T, class E>
class [[nodiscard]] result {
 public:
  static result success(T value) noexcept(
      std::is_nothrow_move_constructible_v<T>) {
    return result(success_tag{}, std::move(value));
  }

  static result failure(E error) noexcept(
      std::is_nothrow_move_constructible_v<E>) {
    return result(failure_tag{}, std::move(error));
  }

  result(const result& other) noexcept(
      std::is_nothrow_copy_constructible_v<T> &&
      std::is_nothrow_copy_constructible_v<E>)
    requires(std::copy_constructible<T> && std::copy_constructible<E>)
      : has_value_{other.has_value_} {
    if (has_value_) {
      ::new (static_cast<void*>(&storage_.value)) T(other.storage_.value);
    } else {
      ::new (static_cast<void*>(&storage_.error)) E(other.storage_.error);
    }
  }

  result(const result&)
    requires(!(std::copy_constructible<T> && std::copy_constructible<E>))
  = delete;

  result(result&& other) noexcept(std::is_nothrow_move_constructible_v<T> &&
                                  std::is_nothrow_move_constructible_v<E>)
    requires(std::move_constructible<T> && std::move_constructible<E>)
      : has_value_{other.has_value_} {
    if (has_value_) {
      ::new (static_cast<void*>(&storage_.value))
          T(std::move(other.storage_.value));
    } else {
      ::new (static_cast<void*>(&storage_.error))
          E(std::move(other.storage_.error));
    }
  }

  result(result&&)
    requires(!(std::move_constructible<T> && std::move_constructible<E>))
  = delete;

  result& operator=(const result&) = delete;
  result& operator=(result&&) = delete;

  ~result() {
    if (has_value_) {
      storage_.value.~T();
    } else {
      storage_.error.~E();
    }
  }

  [[nodiscard]] constexpr explicit operator bool() const noexcept {
    return has_value_;
  }
  [[nodiscard]] constexpr bool has_value() const noexcept { return has_value_; }

  [[nodiscard]] T& value() & noexcept {
    HAL_CORE_ASSERT(has_value_);
    return storage_.value;
  }

  [[nodiscard]] const T& value() const& noexcept {
    HAL_CORE_ASSERT(has_value_);
    return storage_.value;
  }

  [[nodiscard]] T&& value() && noexcept {
    HAL_CORE_ASSERT(has_value_);
    return std::move(storage_.value);
  }

  [[nodiscard]] E& error() & noexcept {
    HAL_CORE_ASSERT(!has_value_);
    return storage_.error;
  }

  [[nodiscard]] const E& error() const& noexcept {
    HAL_CORE_ASSERT(!has_value_);
    return storage_.error;
  }

  [[nodiscard]] E&& error() && noexcept {
    HAL_CORE_ASSERT(!has_value_);
    return std::move(storage_.error);
  }

 private:
  struct success_tag {};
  struct failure_tag {};

  union storage_t {
    T value;
    E error;

    constexpr storage_t() noexcept {}
    ~storage_t() {}
  } storage_;

  bool has_value_{};

  result(success_tag, T value) noexcept(std::is_nothrow_move_constructible_v<T>)
      : has_value_{true} {
    ::new (static_cast<void*>(&storage_.value)) T(std::move(value));
  }

  result(failure_tag, E error) noexcept(std::is_nothrow_move_constructible_v<E>)
      : has_value_{false} {
    ::new (static_cast<void*>(&storage_.error)) E(std::move(error));
  }
};

template <class E>
class [[nodiscard]] result<void, E> {
 public:
  static result success() noexcept { return result(success_tag{}); }

  static result failure(E error) noexcept(
      std::is_nothrow_move_constructible_v<E>) {
    return result(failure_tag{}, std::move(error));
  }

  result(const result& other) noexcept(std::is_nothrow_copy_constructible_v<E>)
    requires std::copy_constructible<E>
      : has_value_{other.has_value_} {
    if (!has_value_) {
      ::new (static_cast<void*>(&storage_.error)) E(other.storage_.error);
    }
  }

  result(const result&)
    requires(!std::copy_constructible<E>)
  = delete;

  result(result&& other) noexcept(std::is_nothrow_move_constructible_v<E>)
    requires std::move_constructible<E>
      : has_value_{other.has_value_} {
    if (!has_value_) {
      ::new (static_cast<void*>(&storage_.error))
          E(std::move(other.storage_.error));
    }
  }

  result(result&&)
    requires(!std::move_constructible<E>)
  = delete;

  result& operator=(const result&) = delete;
  result& operator=(result&&) = delete;

  ~result() {
    if (!has_value_) {
      storage_.error.~E();
    }
  }

  [[nodiscard]] constexpr explicit operator bool() const noexcept {
    return has_value_;
  }
  [[nodiscard]] constexpr bool has_value() const noexcept { return has_value_; }

  [[nodiscard]] E& error() & noexcept {
    HAL_CORE_ASSERT(!has_value_);
    return storage_.error;
  }

  [[nodiscard]] const E& error() const& noexcept {
    HAL_CORE_ASSERT(!has_value_);
    return storage_.error;
  }

  [[nodiscard]] E&& error() && noexcept {
    HAL_CORE_ASSERT(!has_value_);
    return std::move(storage_.error);
  }

 private:
  struct success_tag {};
  struct failure_tag {};

  union storage_t {
    E error;

    constexpr storage_t() noexcept {}
    ~storage_t() {}
  } storage_;

  bool has_value_{};

  explicit result(success_tag) noexcept : has_value_{true} {}

  result(failure_tag, E error) noexcept(std::is_nothrow_move_constructible_v<E>)
      : has_value_{false} {
    ::new (static_cast<void*>(&storage_.error)) E(std::move(error));
  }
};

}  // namespace hal
