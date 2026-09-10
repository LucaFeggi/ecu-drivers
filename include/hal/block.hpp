#pragma once

#include <concepts>
#include <cstdint>
#include <cstddef>
#include <hal/foundation/result.hpp>
#include <hal/foundation/span.hpp>

namespace hal::block {

struct geometry {
  std::uint32_t logical_block_size{};
  std::uint64_t logical_block_count{};
  bool read_only{};
};

enum class error_kind : std::uint8_t {
  no_media,
  write_protected,
  out_of_range,
  not_aligned,
  timeout,
  io,
  other
};

template <class E>
concept Error = requires(const E& error) {
  { error.kind() } noexcept -> std::same_as<error_kind>;
};

template <class T>
concept Device =
    Error<typename T::error_type> &&
    requires(T& device, std::uint64_t lba, span<std::byte> rx, span<const std::byte> tx) {
      {
        device.geometry()
      } -> std::same_as<result<geometry, typename T::error_type>>;
      {
        device.read_blocks(lba, rx)
      } -> std::same_as<result<void, typename T::error_type>>;
      {
        device.write_blocks(lba, tx)
      } -> std::same_as<result<void, typename T::error_type>>;
      { device.sync() } -> std::same_as<result<void, typename T::error_type>>;
    };

template <class T>
concept TrimDevice = Device<T> && requires(T& device, std::uint64_t first_lba,
                                           std::uint64_t count) {
  {
    device.trim(first_lba, count)
  } -> std::same_as<result<void, typename T::error_type>>;
};

}  // namespace hal::block
