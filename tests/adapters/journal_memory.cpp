#include <array>
#include <cstddef>
#include <cstdint>
#include <hal/adapters/nv_journal_memory.hpp>
#include <limits>

namespace {

int failures = 0;

void check(bool condition) {
  if (!condition) {
    ++failures;
  }
}

struct FlashError {
  hal::nv::flash_error_kind value{hal::nv::flash_error_kind::other};
  [[nodiscard]] constexpr hal::nv::flash_error_kind kind() const noexcept {
    return value;
  }
};

class TestFlash {
 public:
  using error_type = FlashError;

  TestFlash() noexcept {
    for (std::byte& value : storage_) {
      value = std::byte{0xFF};
    }
  }

  [[nodiscard]] static constexpr hal::nv::flash_geometry geometry() noexcept {
    return {storage_size, 1U, 4U, 256U};
  }

  [[nodiscard]] auto read(std::size_t offset, hal::span<std::byte> output)
      -> hal::result<void, error_type> {
    if (!output.valid() || !range_valid(offset, output.size())) {
      return failure(hal::nv::flash_error_kind::out_of_range);
    }
    for (std::size_t index = 0U; index < output.size(); ++index) {
      output[index] = storage_[offset + index];
    }
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] auto program(std::size_t offset,
                             hal::span<const std::byte> input)
      -> hal::result<void, error_type> {
    ++program_calls_;
    if (program_calls_ == fail_program_call_) {
      return failure(hal::nv::flash_error_kind::program);
    }
    if (!input.valid() || offset % 4U != 0U || input.size() % 4U != 0U) {
      return failure(hal::nv::flash_error_kind::not_aligned);
    }
    if (!range_valid(offset, input.size())) {
      return failure(hal::nv::flash_error_kind::out_of_range);
    }
    for (std::size_t index = 0U; index < input.size(); ++index) {
      const std::uint8_t previous =
          std::to_integer<std::uint8_t>(storage_[offset + index]);
      const std::uint8_t next = std::to_integer<std::uint8_t>(input[index]);
      if ((static_cast<std::uint8_t>(~previous) & next) != 0) {
        return failure(hal::nv::flash_error_kind::program);
      }
    }
    for (std::size_t index = 0U; index < input.size(); ++index) {
      storage_[offset + index] &= input[index];
    }
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] auto erase(std::size_t offset, std::size_t length)
      -> hal::result<void, error_type> {
    if (offset % 256U != 0U || length % 256U != 0U) {
      return failure(hal::nv::flash_error_kind::not_aligned);
    }
    if (!range_valid(offset, length)) {
      return failure(hal::nv::flash_error_kind::out_of_range);
    }
    for (std::size_t index = 0U; index < length; ++index) {
      storage_[offset + index] = std::byte{0xFF};
    }
    return hal::result<void, error_type>::success();
  }

  [[nodiscard]] auto sync() -> hal::result<void, error_type> {
    ++sync_calls_;
    return hal::result<void, error_type>::success();
  }

  void fail_on_program_call(std::size_t call) noexcept {
    fail_program_call_ = call;
  }
  void clear_failure() noexcept {
    fail_program_call_ = std::numeric_limits<std::size_t>::max();
  }
  [[nodiscard]] std::size_t program_calls() const noexcept {
    return program_calls_;
  }
  [[nodiscard]] std::size_t sync_calls() const noexcept { return sync_calls_; }

 private:
  static constexpr std::size_t storage_size = 1024U;

  [[nodiscard]] static constexpr bool range_valid(std::size_t offset,
                                                  std::size_t size) noexcept {
    return offset <= storage_size && size <= storage_size - offset;
  }

  [[nodiscard]] static auto failure(hal::nv::flash_error_kind kind)
      -> hal::result<void, error_type> {
    return hal::result<void, error_type>::failure({kind});
  }

  std::array<std::byte, storage_size> storage_{};
  std::size_t program_calls_{};
  std::size_t sync_calls_{};
  std::size_t fail_program_call_{std::numeric_limits<std::size_t>::max()};
};

using Memory = hal::nv::JournalMemory<TestFlash, 64U, 16U, 0U, 512U>;

static_assert(hal::nv::NorFlash<TestFlash>);
static_assert(hal::nv::Memory<Memory>);

void test_mount_write_recovery_and_compaction() {
  TestFlash flash{};
  std::array<std::byte, 64U> mount_mirror{};
  auto mounted = Memory::mount(flash, mount_mirror);
  check(mounted.has_value());
  if (!mounted) {
    return;
  }

  Memory& memory = mounted.value();
  check(memory.properties().capacity == 64U);
  check(memory.properties().max_atomic_write_size == 16U);

  for (std::uint8_t iteration = 0U; iteration < 10U; ++iteration) {
    const std::array<std::byte, 4U> value{static_cast<std::byte>(iteration),
                                          std::byte{0x22}, std::byte{0x33},
                                          std::byte{0x44}};
    check(memory.write(8U, value).has_value());
  }
  check(memory.sync().has_value());
  check(flash.sync_calls() > 0U);

  std::array<std::byte, 64U> recovery_mirror{};
  auto recovered = Memory::mount(flash, recovery_mirror);
  check(recovered.has_value());
  if (!recovered) {
    return;
  }
  std::array<std::byte, 4U> output{};
  check(recovered.value().read(8U, output).has_value());
  check(output[0U] == std::byte{9U});
  check(output[1U] == std::byte{0x22});

  std::array<std::byte, 17U> too_large{};
  auto rejected = recovered.value().write(0U, too_large);
  check(!rejected);
  check(rejected.error().kind() == hal::nv::memory_error_kind::too_large);
}

void test_torn_record_is_ignored() {
  TestFlash flash{};
  std::array<std::byte, 64U> mount_mirror{};
  auto mounted = Memory::mount(flash, mount_mirror);
  check(mounted.has_value());
  if (!mounted) {
    return;
  }

  const std::array<std::byte, 4U> committed{std::byte{1U}, std::byte{2U},
                                            std::byte{3U}, std::byte{4U}};
  check(mounted.value().write(0U, committed).has_value());
  check(mounted.value().sync().has_value());

  // A record programs its header first and payload second. Failing the
  // payload leaves no commit marker, so recovery must retain the earlier
  // value.
  flash.fail_on_program_call(flash.program_calls() + 2U);
  const std::array<std::byte, 4U> torn{std::byte{9U}, std::byte{9U},
                                       std::byte{9U}, std::byte{9U}};
  auto failed_write = mounted.value().write(0U, torn);
  check(!failed_write);
  flash.clear_failure();

  std::array<std::byte, 64U> recovery_mirror{};
  auto recovered = Memory::mount(flash, recovery_mirror);
  check(recovered.has_value());
  if (!recovered) {
    return;
  }
  std::array<std::byte, 4U> output{};
  check(recovered.value().read(0U, output).has_value());
  check(output == committed);
}

}  // namespace

int main() {
  test_mount_write_recovery_and_compaction();
  test_torn_record_is_ignored();
  return failures;
}
