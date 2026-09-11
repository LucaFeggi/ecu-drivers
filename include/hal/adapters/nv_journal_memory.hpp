#pragma once
#include <array>
#include <cstddef>
#include <hal/foundation/assert.hpp>
#include <hal/foundation/crc32.hpp>
#include <hal/foundation/result.hpp>
#include <hal/foundation/span.hpp>
#include <hal/contracts/nv.hpp>
#include <limits>
#include <utility>

namespace hal::nv {

enum class journal_error_source : std::uint8_t {
  argument,
  layout,
  flash,
  recovery
};

// A compact error that remains allocation-free while retaining the underlying
// portable flash error category.
class journal_error {
 public:
  [[nodiscard]] static constexpr journal_error argument(
      memory_error_kind kind) noexcept {
    return {kind, journal_error_source::argument, flash_error_kind::other};
  }

  [[nodiscard]] static constexpr journal_error layout(
      memory_error_kind kind) noexcept {
    return {kind, journal_error_source::layout, flash_error_kind::other};
  }

  [[nodiscard]] static constexpr journal_error flash(
      flash_error_kind kind) noexcept {
    return {map_flash(kind), journal_error_source::flash, kind};
  }

  [[nodiscard]] static constexpr journal_error recovery() noexcept {
    return {memory_error_kind::corrupted, journal_error_source::recovery,
            flash_error_kind::other};
  }

  [[nodiscard]] constexpr memory_error_kind kind() const noexcept {
    return kind_;
  }
  [[nodiscard]] constexpr journal_error_source source() const noexcept {
    return source_;
  }
  [[nodiscard]] constexpr flash_error_kind underlying_flash_kind()
      const noexcept {
    return flash_kind_;
  }

 private:
  [[nodiscard]] static constexpr memory_error_kind map_flash(
      flash_error_kind kind) noexcept {
    switch (kind) {
      case flash_error_kind::out_of_range:
        return memory_error_kind::out_of_range;
      case flash_error_kind::not_aligned:
      case flash_error_kind::program:
      case flash_error_kind::erase:
      case flash_error_kind::io:
        return memory_error_kind::io;
      case flash_error_kind::other:
        return memory_error_kind::other;
    }
    return memory_error_kind::other;
  }

  constexpr journal_error(memory_error_kind kind, journal_error_source source,
                          flash_error_kind flash_kind) noexcept
      : kind_{kind}, source_{source}, flash_kind_{flash_kind} {}

  memory_error_kind kind_;
  journal_error_source source_;
  flash_error_kind flash_kind_;
};

namespace detail {

[[nodiscard]] consteval std::size_t journal_gcd(std::size_t left,
                                                std::size_t right) {
  while (right != 0U) {
    const std::size_t remainder = left % right;
    left = right;
    right = remainder;
  }
  return left;
}

[[nodiscard]] consteval std::size_t journal_lcm(std::size_t left,
                                                std::size_t right) {
  return (left / journal_gcd(left, right)) * right;
}

[[nodiscard]] constexpr std::size_t journal_align_up(std::size_t value,
                                                     std::size_t alignment) {
  return ((value + alignment - 1U) / alignment) * alignment;
}

template <std::size_t Size>
constexpr void fill_erased(std::array<std::byte, Size>& data) noexcept {
  for (std::byte& value : data) {
    value = std::byte{0xFF};
  }
}

constexpr void fill_erased(span<std::byte> data) noexcept {
  HAL_CORE_ASSERT(data.valid());
  if (!data.valid()) {
    return;
  }
  for (std::byte& value : data) {
    value = std::byte{0xFF};
  }
}

template <std::size_t Size>
[[nodiscard]] constexpr bool all_erased(
    const std::array<std::byte, Size>& data) noexcept {
  for (const std::byte value : data) {
    if (value != std::byte{0xFF}) {
      return false;
    }
  }
  return true;
}

template <std::size_t Size>
constexpr void put_u16(std::array<std::byte, Size>& destination,
                       std::size_t offset, std::uint16_t value) noexcept {
  destination[offset] = static_cast<std::byte>(value & 0xFFU);
  destination[offset + 1U] = static_cast<std::byte>((value >> 8U) & 0xFFU);
}

template <std::size_t Size>
constexpr void put_u32(std::array<std::byte, Size>& destination,
                       std::size_t offset, std::uint32_t value) noexcept {
  for (std::size_t index = 0U; index < 4U; ++index) {
    destination[offset + index] =
        static_cast<std::byte>((value >> (index * 8U)) & 0xFFU);
  }
}

template <std::size_t Size>
constexpr void put_u64(std::array<std::byte, Size>& destination,
                       std::size_t offset, std::uint64_t value) noexcept {
  for (std::size_t index = 0U; index < 8U; ++index) {
    destination[offset + index] =
        static_cast<std::byte>((value >> (index * 8U)) & 0xFFU);
  }
}

template <std::size_t Size>
[[nodiscard]] constexpr std::uint16_t get_u16(
    const std::array<std::byte, Size>& source, std::size_t offset) noexcept {
  return static_cast<std::uint16_t>(
      static_cast<std::uint16_t>(
          std::to_integer<std::uint8_t>(source[offset])) |
      static_cast<std::uint16_t>(
          static_cast<std::uint16_t>(
              std::to_integer<std::uint8_t>(source[offset + 1U]))
          << 8U));
}

template <std::size_t Size>
[[nodiscard]] constexpr std::uint32_t get_u32(
    const std::array<std::byte, Size>& source, std::size_t offset) noexcept {
  std::uint32_t value = 0U;
  for (std::size_t index = 0U; index < 4U; ++index) {
    value |= static_cast<std::uint32_t>(
                 std::to_integer<std::uint8_t>(source[offset + index]))
             << (index * 8U);
  }
  return value;
}

template <std::size_t Size>
[[nodiscard]] constexpr std::uint64_t get_u64(
    const std::array<std::byte, Size>& source, std::size_t offset) noexcept {
  std::uint64_t value = 0U;
  for (std::size_t index = 0U; index < 8U; ++index) {
    value |= static_cast<std::uint64_t>(
                 std::to_integer<std::uint8_t>(source[offset + index]))
             << (index * 8U);
  }
  return value;
}

}  // namespace detail

// Two-region, append-only, power-loss-recoverable std::byte memory over uniform
// NOR flash. Flash::geometry() must be a constant expression. The in-RAM mirror
// is explicitly caller-supplied and must outlive the mounted object; this keeps
// a potentially large buffer out of return-value temporaries and avoids hidden
// allocation.
//
// Region layout:
//   region header body | region commit | record... | erased tail
//
// A record's commit marker is programmed after its header and payload. During
// compaction, the new region is committed only after its snapshot is durable;
// only then is the old region erased.
template <NorFlash Flash, std::size_t LogicalCapacity,
          std::size_t MaxAtomicWriteSize, std::size_t BaseOffset = 0U,
          std::size_t RegionSize = Flash::geometry().erase_size>
class JournalMemory {
 public:
  using error_type = journal_error;

  static constexpr flash_geometry underlying_geometry = Flash::geometry();
  static constexpr std::size_t logical_capacity = LogicalCapacity;
  static constexpr std::size_t max_atomic_write_size = MaxAtomicWriteSize;

 private:
  static constexpr std::size_t safe_read_size =
      underlying_geometry.read_size == 0U ? 1U : underlying_geometry.read_size;
  static constexpr std::size_t safe_program_size =
      underlying_geometry.program_size == 0U ? 1U
                                             : underlying_geometry.program_size;
  static constexpr std::size_t io_unit =
      detail::journal_lcm(safe_read_size, safe_program_size);

  static constexpr std::size_t region_header_data_size = 36U;
  static constexpr std::size_t record_header_data_size = 40U;
  static constexpr std::size_t commit_data_size = 16U;
  static constexpr std::size_t region_header_size =
      detail::journal_align_up(region_header_data_size, io_unit);
  static constexpr std::size_t record_header_size =
      detail::journal_align_up(record_header_data_size, io_unit);
  static constexpr std::size_t commit_size =
      detail::journal_align_up(commit_data_size, io_unit);
  static constexpr std::size_t record_area_offset =
      region_header_size + commit_size;

  [[nodiscard]] static constexpr std::size_t record_size(
      std::size_t payload_size) {
    return record_header_size +
           detail::journal_align_up(payload_size, io_unit) + commit_size;
  }

  static_assert(underlying_geometry.capacity > 0U,
                "NOR capacity must be non-zero");
  static_assert(underlying_geometry.read_size > 0U,
                "NOR read size must be non-zero");
  static_assert(underlying_geometry.program_size > 0U,
                "NOR program size must be non-zero");
  static_assert(underlying_geometry.erase_size > 0U,
                "NOR erase size must be non-zero");
  static_assert(LogicalCapacity > 0U,
                "logical memory capacity must be non-zero");
  static_assert(MaxAtomicWriteSize > 0U,
                "maximum atomic write must be non-zero");
  static_assert(MaxAtomicWriteSize <= LogicalCapacity,
                "maximum atomic write cannot exceed logical capacity");
  static_assert(BaseOffset % underlying_geometry.erase_size == 0U,
                "journal base must be erase aligned");
  static_assert(BaseOffset % io_unit == 0U,
                "journal base must be aligned to the combined I/O unit");
  static_assert(RegionSize % underlying_geometry.erase_size == 0U,
                "journal region must contain complete erase units");
  static_assert(RegionSize % io_unit == 0U,
                "journal region must be aligned to the combined I/O unit");
  static_assert(BaseOffset <= underlying_geometry.capacity,
                "journal base is outside the underlying flash");
  static_assert(RegionSize <= (underlying_geometry.capacity - BaseOffset) / 2U,
                "two journal regions do not fit in the underlying flash");
  static_assert(record_area_offset + record_size(LogicalCapacity) +
                        record_size(MaxAtomicWriteSize) <=
                    RegionSize,
                "each region must fit one compaction snapshot and one "
                "maximum-sized write");

 public:
  JournalMemory(const JournalMemory&) = delete;
  JournalMemory& operator=(const JournalMemory&) = delete;
  JournalMemory(JournalMemory&&) noexcept = default;
  JournalMemory& operator=(JournalMemory&&) = delete;

  // Mounts existing state. Completely erased storage is initialized; storage
  // containing no valid region but non-erased bytes is reported as corrupted.
  [[nodiscard]] static auto mount(Flash& flash, span<std::byte> mirror)
      -> result<JournalMemory, error_type> {
    HAL_CORE_ASSERT(mirror.valid() && mirror.size() == LogicalCapacity);
    if (!mirror.valid() || mirror.size() != LogicalCapacity) {
      return result<JournalMemory, error_type>::failure(
          error_type::argument(memory_error_kind::other));
    }
    JournalMemory memory{flash, mirror};
    auto mounted = memory.mount_existing();
    if (!mounted) {
      return result<JournalMemory, error_type>::failure(
          std::move(mounted).error());
    }
    return result<JournalMemory, error_type>::success(std::move(memory));
  }

  // Explicit destructive recovery for a new/reprovisioned journal.
  [[nodiscard]] static auto format(Flash& flash, span<std::byte> mirror)
      -> result<JournalMemory, error_type> {
    HAL_CORE_ASSERT(mirror.valid() && mirror.size() == LogicalCapacity);
    if (!mirror.valid() || mirror.size() != LogicalCapacity) {
      return result<JournalMemory, error_type>::failure(
          error_type::argument(memory_error_kind::other));
    }
    JournalMemory memory{flash, mirror};
    auto formatted = memory.format_storage();
    if (!formatted) {
      return result<JournalMemory, error_type>::failure(
          std::move(formatted).error());
    }
    return result<JournalMemory, error_type>::success(std::move(memory));
  }

  [[nodiscard]] constexpr memory_properties properties() const noexcept {
    return {LogicalCapacity, MaxAtomicWriteSize};
  }

  [[nodiscard]] auto read(std::size_t offset, span<std::byte> output) const
      -> result<void, error_type> {
    HAL_CORE_ASSERT(output.valid());
    if (!output.valid()) {
      return fail(error_type::argument(memory_error_kind::other));
    }
    if (!range_valid(offset, output.size())) {
      return fail(error_type::argument(memory_error_kind::out_of_range));
    }

    for (std::size_t index = 0U; index < output.size(); ++index) {
      output[index] = mirror_[offset + index];
    }
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto write(std::size_t offset, span<const std::byte> input)
      -> result<void, error_type> {
    HAL_CORE_ASSERT(input.valid());
    if (!input.valid()) {
      return fail(error_type::argument(memory_error_kind::other));
    }
    if (!range_valid(offset, input.size())) {
      return fail(error_type::argument(memory_error_kind::out_of_range));
    }
    if (input.size() > MaxAtomicWriteSize) {
      return fail(error_type::argument(memory_error_kind::too_large));
    }
    if (input.empty()) {
      return result<void, error_type>::success();
    }

    const std::size_t required = record_size(input.size());
    if (tail_unusable_ ||
        required > region_end(active_region_) - append_offset_) {
      auto compacted = compact();
      if (!compacted) {
        return compacted;
      }
    }

    if (last_sequence_ == std::numeric_limits<std::uint64_t>::max()) {
      return fail(error_type::layout(memory_error_kind::no_space));
    }

    const std::uint64_t sequence = last_sequence_ + 1U;
    auto appended = write_record(append_offset_, sequence, offset, input);
    if (!appended) {
      tail_unusable_ = true;
      return appended;
    }

    for (std::size_t index = 0U; index < input.size(); ++index) {
      mirror_[offset + index] = input[index];
    }
    append_offset_ += required;
    last_sequence_ = sequence;
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto sync() -> result<void, error_type> {
    return map_flash_result(flash_->sync());
  }

  [[nodiscard]] constexpr Flash& underlying() noexcept { return *flash_; }
  [[nodiscard]] constexpr std::uint64_t generation() const noexcept {
    return generation_;
  }

 private:
  static constexpr std::uint32_t region_magic = 0x4A524E4CU;         // JRNL
  static constexpr std::uint32_t region_commit_magic = 0x52434D54U;  // RCMT
  static constexpr std::uint32_t record_magic = 0x52454344U;         // RECD
  static constexpr std::uint32_t record_commit_magic = 0x57434D54U;  // WCMT
  static constexpr std::uint16_t format_version = 1U;

  struct region_info {
    bool valid{};
    bool erased{};
    std::uint64_t generation{};
  };

  explicit constexpr JournalMemory(Flash& flash,
                                   span<std::byte> mirror) noexcept
      : flash_{&flash}, mirror_{mirror} {}

  [[nodiscard]] static auto fail(error_type error) -> result<void, error_type> {
    return result<void, error_type>::failure(error);
  }

  [[nodiscard]] static constexpr bool range_valid(std::size_t offset,
                                                  std::size_t size) noexcept {
    return offset <= LogicalCapacity && size <= LogicalCapacity - offset;
  }

  [[nodiscard]] static constexpr std::size_t region_start(
      std::uint8_t region) noexcept {
    return BaseOffset + static_cast<std::size_t>(region) * RegionSize;
  }

  [[nodiscard]] static constexpr std::size_t region_end(
      std::uint8_t region) noexcept {
    return region_start(region) + RegionSize;
  }

  template <std::size_t Size>
  [[nodiscard]] auto read_array(std::size_t offset,
                                std::array<std::byte, Size>& output)
      -> result<void, error_type> {
    return map_flash_result(flash_->read(offset, span<std::byte>{output}));
  }

  template <std::size_t Size>
  [[nodiscard]] auto program_array(std::size_t offset,
                                   const std::array<std::byte, Size>& input)
      -> result<void, error_type> {
    return map_flash_result(
        flash_->program(offset, span<const std::byte>{input}));
  }

  template <class FlashResult>
  [[nodiscard]] static auto map_flash_result(FlashResult operation)
      -> result<void, error_type> {
    if (!operation) {
      return fail(error_type::flash(operation.error().kind()));
    }
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto erase_region(std::uint8_t region)
      -> result<void, error_type> {
    return map_flash_result(flash_->erase(region_start(region), RegionSize));
  }

  [[nodiscard]] auto mount_existing() -> result<void, error_type> {
    auto first = inspect_region(0U);
    if (!first) {
      return fail(std::move(first).error());
    }
    auto second = inspect_region(1U);
    if (!second) {
      return fail(std::move(second).error());
    }

    const region_info first_info = first.value();
    const region_info second_info = second.value();

    if (!first_info.valid && !second_info.valid) {
      if (first_info.erased && second_info.erased) {
        return format_storage();
      }
      return fail(error_type::recovery());
    }

    if (first_info.valid && second_info.valid &&
        first_info.generation == second_info.generation) {
      return fail(error_type::recovery());
    }

    const bool second_is_newer =
        second_info.valid &&
        (!first_info.valid || second_info.generation > first_info.generation);
    active_region_ = second_is_newer ? 1U : 0U;
    generation_ =
        second_is_newer ? second_info.generation : first_info.generation;

    auto loaded = load_active_region();
    if (loaded || loaded.error().source() != journal_error_source::recovery) {
      return loaded;
    }

    // A valid commit marker with a corrupt/missing compaction snapshot does
    // not supersede the older region. This is the final recovery guard if a
    // storage fault damaged the newly committed generation.
    const bool alternate_valid =
        second_is_newer ? first_info.valid : second_info.valid;
    if (!alternate_valid) {
      return loaded;
    }
    active_region_ = second_is_newer ? 0U : 1U;
    generation_ =
        second_is_newer ? first_info.generation : second_info.generation;
    return load_active_region();
  }

  [[nodiscard]] auto inspect_region(std::uint8_t region)
      -> result<region_info, error_type> {
    std::array<std::byte, region_header_size> header{};
    std::array<std::byte, commit_size> commit{};

    auto header_read = read_array(region_start(region), header);
    if (!header_read) {
      return result<region_info, error_type>::failure(
          std::move(header_read).error());
    }
    auto commit_read =
        read_array(region_start(region) + region_header_size, commit);
    if (!commit_read) {
      return result<region_info, error_type>::failure(
          std::move(commit_read).error());
    }

    const bool body_valid = region_header_valid(header);
    const std::uint64_t generation = detail::get_u64(header, 8U);
    const bool committed =
        body_valid && region_commit_valid(commit, generation);
    if (committed) {
      return result<region_info, error_type>::success(
          {true, false, generation});
    }

    auto erased = region_is_erased(region);
    if (!erased) {
      return result<region_info, error_type>::failure(
          std::move(erased).error());
    }
    return result<region_info, error_type>::success(
        {false, erased.value(), 0U});
  }

  [[nodiscard]] auto region_is_erased(std::uint8_t region)
      -> result<bool, error_type> {
    std::array<std::byte, io_unit> chunk{};
    for (std::size_t offset = region_start(region); offset < region_end(region);
         offset += io_unit) {
      auto read = read_array(offset, chunk);
      if (!read) {
        return result<bool, error_type>::failure(std::move(read).error());
      }
      if (!detail::all_erased(chunk)) {
        return result<bool, error_type>::success(false);
      }
    }
    return result<bool, error_type>::success(true);
  }

  [[nodiscard]] static bool region_header_valid(
      const std::array<std::byte, region_header_size>& header) noexcept {
    if (detail::get_u32(header, 0U) != region_magic ||
        detail::get_u16(header, 4U) != format_version ||
        detail::get_u64(header, 8U) == 0U ||
        detail::get_u64(header, 16U) !=
            static_cast<std::uint64_t>(LogicalCapacity) ||
        detail::get_u64(header, 24U) !=
            static_cast<std::uint64_t>(MaxAtomicWriteSize)) {
      return false;
    }
    const std::uint32_t expected_crc = detail::get_u32(header, 32U);
    return expected_crc ==
           hal::crc32(span<const std::byte>{header.data(), 32U});
  }

  [[nodiscard]] static bool region_commit_valid(
      const std::array<std::byte, commit_size>& commit,
      std::uint64_t generation) noexcept {
    return detail::get_u32(commit, 0U) == region_commit_magic &&
           detail::get_u64(commit, 4U) == generation &&
           detail::get_u32(commit, 12U) ==
               hal::crc32(span<const std::byte>{commit.data(), 12U});
  }

  [[nodiscard]] auto load_active_region() -> result<void, error_type> {
    detail::fill_erased(mirror_);
    last_sequence_ = 0U;
    append_offset_ = region_start(active_region_) + record_area_offset;
    tail_unusable_ = false;
    bool compaction_snapshot_loaded = generation_ == 1U;

    const std::size_t end = region_end(active_region_);
    while (append_offset_ + record_header_size + commit_size <= end) {
      std::array<std::byte, record_header_size> header{};
      auto header_read = read_array(append_offset_, header);
      if (!header_read) {
        return header_read;
      }
      if (detail::all_erased(header)) {
        return finish_recovered_tail(compaction_snapshot_loaded, false);
      }
      if (!record_header_valid(header)) {
        return finish_recovered_tail(compaction_snapshot_loaded, true);
      }

      const std::uint64_t sequence = detail::get_u64(header, 8U);
      const std::uint64_t logical_offset_64 = detail::get_u64(header, 16U);
      const std::uint64_t length_64 = detail::get_u64(header, 24U);
      if (logical_offset_64 > LogicalCapacity || length_64 == 0U ||
          length_64 >
              LogicalCapacity - static_cast<std::size_t>(logical_offset_64) ||
          sequence <= last_sequence_) {
        return finish_recovered_tail(compaction_snapshot_loaded, true);
      }

      const std::size_t logical_offset =
          static_cast<std::size_t>(logical_offset_64);
      const std::size_t length = static_cast<std::size_t>(length_64);
      if (!compaction_snapshot_loaded &&
          (logical_offset != 0U || length != LogicalCapacity)) {
        return fail(error_type::recovery());
      }
      const std::size_t total_size = record_size(length);
      if (total_size > end - append_offset_) {
        return finish_recovered_tail(compaction_snapshot_loaded, true);
      }

      const std::size_t payload_offset = append_offset_ + record_header_size;
      const std::size_t commit_offset =
          payload_offset + detail::journal_align_up(length, io_unit);
      std::array<std::byte, commit_size> commit{};
      auto commit_read = read_array(commit_offset, commit);
      if (!commit_read) {
        return commit_read;
      }
      if (!record_commit_valid(commit, sequence)) {
        return finish_recovered_tail(compaction_snapshot_loaded, true);
      }

      auto payload_crc = crc_payload(payload_offset, length);
      if (!payload_crc) {
        return fail(std::move(payload_crc).error());
      }
      if (payload_crc.value() != detail::get_u32(header, 32U)) {
        return finish_recovered_tail(compaction_snapshot_loaded, true);
      }

      auto applied = apply_payload(payload_offset, logical_offset, length);
      if (!applied) {
        return applied;
      }
      compaction_snapshot_loaded = true;
      last_sequence_ = sequence;
      append_offset_ += total_size;
    }

    return finish_recovered_tail(compaction_snapshot_loaded, true);
  }

  [[nodiscard]] auto finish_recovered_tail(bool prefix_valid, bool unusable)
      -> result<void, error_type> {
    tail_unusable_ = unusable;
    if (!prefix_valid) {
      return fail(error_type::recovery());
    }
    return result<void, error_type>::success();
  }

  [[nodiscard]] static bool record_header_valid(
      const std::array<std::byte, record_header_size>& header) noexcept {
    return detail::get_u32(header, 0U) == record_magic &&
           detail::get_u16(header, 4U) == format_version &&
           detail::get_u32(header, 36U) ==
               hal::crc32(span<const std::byte>{header.data(), 36U});
  }

  [[nodiscard]] static bool record_commit_valid(
      const std::array<std::byte, commit_size>& commit,
      std::uint64_t sequence) noexcept {
    return detail::get_u32(commit, 0U) == record_commit_magic &&
           detail::get_u64(commit, 4U) == sequence &&
           detail::get_u32(commit, 12U) ==
               hal::crc32(span<const std::byte>{commit.data(), 12U});
  }

  [[nodiscard]] auto crc_payload(std::size_t flash_offset, std::size_t length)
      -> result<std::uint32_t, error_type> {
    std::uint32_t state = 0xFFFFFFFFU;
    std::array<std::byte, io_unit> chunk{};
    std::size_t remaining = length;
    std::size_t offset = flash_offset;

    while (remaining > 0U) {
      auto read = read_array(offset, chunk);
      if (!read) {
        return result<std::uint32_t, error_type>::failure(
            std::move(read).error());
      }
      const std::size_t consumed = remaining < io_unit ? remaining : io_unit;
      for (std::size_t index = 0U; index < consumed; ++index) {
        state ^= static_cast<std::uint32_t>(
            std::to_integer<std::uint8_t>(chunk[index]));
        for (std::uint8_t bit = 0U; bit < 8U; ++bit) {
          const std::uint32_t mask = 0U - (state & 1U);
          state = (state >> 1U) ^ (0xEDB88320U & mask);
        }
      }
      offset += io_unit;
      remaining -= consumed;
    }
    return result<std::uint32_t, error_type>::success(~state);
  }

  [[nodiscard]] auto apply_payload(std::size_t flash_offset,
                                   std::size_t logical_offset,
                                   std::size_t length)
      -> result<void, error_type> {
    std::array<std::byte, io_unit> chunk{};
    std::size_t remaining = length;
    std::size_t source_offset = flash_offset;
    std::size_t destination_offset = logical_offset;

    while (remaining > 0U) {
      auto read = read_array(source_offset, chunk);
      if (!read) {
        return read;
      }
      const std::size_t copied = remaining < io_unit ? remaining : io_unit;
      for (std::size_t index = 0U; index < copied; ++index) {
        mirror_[destination_offset + index] = chunk[index];
      }
      source_offset += io_unit;
      destination_offset += copied;
      remaining -= copied;
    }
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto format_storage() -> result<void, error_type> {
    auto first_erased = erase_region(0U);
    if (!first_erased) {
      return first_erased;
    }
    auto second_erased = erase_region(1U);
    if (!second_erased) {
      return second_erased;
    }

    detail::fill_erased(mirror_);
    active_region_ = 0U;
    generation_ = 1U;
    last_sequence_ = 0U;
    append_offset_ = region_start(0U) + record_area_offset;
    tail_unusable_ = false;

    auto header = write_region_header(0U, generation_);
    if (!header) {
      return header;
    }
    auto committed = write_region_commit(0U, generation_);
    if (!committed) {
      return committed;
    }
    return map_flash_result(flash_->sync());
  }

  [[nodiscard]] auto compact() -> result<void, error_type> {
    if (generation_ == std::numeric_limits<std::uint64_t>::max() ||
        last_sequence_ == std::numeric_limits<std::uint64_t>::max()) {
      return fail(error_type::layout(memory_error_kind::no_space));
    }

    const std::uint8_t destination = active_region_ == 0U ? 1U : 0U;
    const std::uint8_t old_region = active_region_;
    const std::uint64_t next_generation = generation_ + 1U;
    const std::uint64_t snapshot_sequence = last_sequence_ + 1U;

    auto erased = erase_region(destination);
    if (!erased) {
      return erased;
    }
    auto header = write_region_header(destination, next_generation);
    if (!header) {
      return header;
    }

    const std::size_t snapshot_offset =
        region_start(destination) + record_area_offset;
    auto snapshot = write_record(snapshot_offset, snapshot_sequence, 0U,
                                 span<const std::byte>{mirror_});
    if (!snapshot) {
      return snapshot;
    }

    // The snapshot must be durable before the destination region becomes
    // eligible for recovery selection.
    auto snapshot_synced = map_flash_result(flash_->sync());
    if (!snapshot_synced) {
      return snapshot_synced;
    }
    auto committed = write_region_commit(destination, next_generation);
    if (!committed) {
      return committed;
    }
    auto region_synced = map_flash_result(flash_->sync());
    if (!region_synced) {
      return region_synced;
    }

    active_region_ = destination;
    generation_ = next_generation;
    last_sequence_ = snapshot_sequence;
    append_offset_ = snapshot_offset + record_size(LogicalCapacity);
    tail_unusable_ = false;

    // Erasing the obsolete region is deliberately last. If it fails, the
    // newly committed region is still the unique newest recovery source.
    return erase_region(old_region);
  }

  [[nodiscard]] auto write_region_header(std::uint8_t region,
                                         std::uint64_t generation)
      -> result<void, error_type> {
    std::array<std::byte, region_header_size> header{};
    detail::fill_erased(header);
    detail::put_u32(header, 0U, region_magic);
    detail::put_u16(header, 4U, format_version);
    detail::put_u16(header, 6U, 0U);
    detail::put_u64(header, 8U, generation);
    detail::put_u64(header, 16U, static_cast<std::uint64_t>(LogicalCapacity));
    detail::put_u64(header, 24U,
                    static_cast<std::uint64_t>(MaxAtomicWriteSize));
    detail::put_u32(header, 32U,
                    hal::crc32(span<const std::byte>{header.data(), 32U}));
    return program_array(region_start(region), header);
  }

  [[nodiscard]] auto write_region_commit(std::uint8_t region,
                                         std::uint64_t generation)
      -> result<void, error_type> {
    std::array<std::byte, commit_size> commit{};
    detail::fill_erased(commit);
    detail::put_u32(commit, 0U, region_commit_magic);
    detail::put_u64(commit, 4U, generation);
    detail::put_u32(commit, 12U,
                    hal::crc32(span<const std::byte>{commit.data(), 12U}));
    return program_array(region_start(region) + region_header_size, commit);
  }

  [[nodiscard]] auto write_record(std::size_t flash_offset,
                                  std::uint64_t sequence,
                                  std::size_t logical_offset,
                                  span<const std::byte> payload)
      -> result<void, error_type> {
    std::array<std::byte, record_header_size> header{};
    detail::fill_erased(header);
    detail::put_u32(header, 0U, record_magic);
    detail::put_u16(header, 4U, format_version);
    detail::put_u16(header, 6U, 0U);
    detail::put_u64(header, 8U, sequence);
    detail::put_u64(header, 16U, static_cast<std::uint64_t>(logical_offset));
    detail::put_u64(header, 24U, static_cast<std::uint64_t>(payload.size()));
    detail::put_u32(header, 32U, hal::crc32(payload));
    detail::put_u32(header, 36U,
                    hal::crc32(span<const std::byte>{header.data(), 36U}));

    auto header_programmed = program_array(flash_offset, header);
    if (!header_programmed) {
      return header_programmed;
    }

    std::array<std::byte, io_unit> chunk{};
    std::size_t remaining = payload.size();
    std::size_t source_offset = 0U;
    std::size_t destination_offset = flash_offset + record_header_size;
    while (remaining > 0U) {
      detail::fill_erased(chunk);
      const std::size_t copied = remaining < io_unit ? remaining : io_unit;
      for (std::size_t index = 0U; index < copied; ++index) {
        chunk[index] = payload[source_offset + index];
      }
      auto payload_programmed = program_array(destination_offset, chunk);
      if (!payload_programmed) {
        return payload_programmed;
      }
      source_offset += copied;
      destination_offset += io_unit;
      remaining -= copied;
    }

    std::array<std::byte, commit_size> commit{};
    detail::fill_erased(commit);
    detail::put_u32(commit, 0U, record_commit_magic);
    detail::put_u64(commit, 4U, sequence);
    detail::put_u32(commit, 12U,
                    hal::crc32(span<const std::byte>{commit.data(), 12U}));
    return program_array(destination_offset, commit);
  }

  Flash* flash_{};
  span<std::byte> mirror_{};
  std::uint64_t generation_{};
  std::uint64_t last_sequence_{};
  std::size_t append_offset_{};
  std::uint8_t active_region_{};
  bool tail_unusable_{};
};

}  // namespace hal::nv
