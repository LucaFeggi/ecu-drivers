#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <hal/foundation/result.hpp>
#include <hal/foundation/span.hpp>

namespace hal::nv {

enum class flash_error_kind : std::uint8_t {
  out_of_range,
  not_aligned,
  program,
  erase,
  io,
  other
};

template <class E>
concept FlashError = requires(const E& error) {
  { error.kind() } noexcept -> std::same_as<flash_error_kind>;
};

struct flash_geometry {
  std::size_t capacity{};
  std::size_t read_size{};
  std::size_t program_size{};
  std::size_t erase_size{};
};

template <class T>
concept NorFlash = FlashError<typename T::error_type> &&
                   requires(T& flash, std::size_t offset, std::size_t length,
                            span<std::byte> rx, span<const std::byte> tx) {
                     { T::geometry() } noexcept -> std::same_as<flash_geometry>;
                     {
                       flash.read(offset, rx)
                     } -> std::same_as<result<void, typename T::error_type>>;
                     {
                       flash.program(offset, tx)
                     } -> std::same_as<result<void, typename T::error_type>>;
                     {
                       flash.erase(offset, length)
                     } -> std::same_as<result<void, typename T::error_type>>;
                     {
                       flash.sync()
                     } -> std::same_as<result<void, typename T::error_type>>;
                   };

enum class memory_error_kind : std::uint8_t {
  out_of_range,
  too_large,
  no_space,
  corrupted,
  io,
  other
};

struct memory_properties {
  std::size_t capacity{};
  std::size_t max_atomic_write_size{};
};

template <class E>
concept MemoryError = requires(const E& error) {
  { error.kind() } noexcept -> std::same_as<memory_error_kind>;
};

template <class T>
concept Memory = MemoryError<typename T::error_type> &&
                 requires(T& memory, std::size_t offset, span<std::byte> rx,
                          span<const std::byte> tx) {
                   {
                     memory.properties()
                   } noexcept -> std::same_as<memory_properties>;
                   {
                     memory.read(offset, rx)
                   } -> std::same_as<result<void, typename T::error_type>>;
                   {
                     memory.write(offset, tx)
                   } -> std::same_as<result<void, typename T::error_type>>;
                   {
                     memory.sync()
                   } -> std::same_as<result<void, typename T::error_type>>;
                 };

}  // namespace hal::nv
