#include <array>
#include <cstddef>
#include <hal/linux/block.hpp>
#include <hal/nv/journal_memory.hpp>
#include <hal/linux/nv.hpp>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>

namespace {

bool make_path(char* path) {
  const int fd = ::mkstemp(path);
  if (fd < 0) {
    return false;
  }
  (void)::close(fd);
  return true;
}

}  // namespace

int main() {
  char block_path[] = "/tmp/hal-linux-block-XXXXXX";
  char flash_path[] = "/tmp/hal-linux-flash-XXXXXX";
  char journal_path[] = "/tmp/hal-linux-journal-XXXXXX";
  if (!make_path(block_path) || !make_path(flash_path) ||
      !make_path(journal_path)) {
    return 1;
  }

  int result = 0;
  {
    auto block = hal::linux::block::Device::open(
        {block_path, 16U, 256U, false});
    if (!block) {
      result = 1;
    } else {
      std::array<std::byte, 16U> written{};
      for (std::size_t index = 0U; index < written.size(); ++index) {
        written[index] = static_cast<std::byte>(index + 1U);
      }
      std::array<std::byte, 16U> read{};
      if (!block.value().write_blocks(2U, {written.data(), written.size()}) ||
          !block.value().read_blocks(2U, {read.data(), read.size()}) ||
          read != written || !block.value().sync()) {
        result = 1;
      }
      const auto out_of_range =
          block.value().read_blocks(100U, {read.data(), read.size()});
      if (out_of_range ||
          out_of_range.error().kind() != hal::block::error_kind::out_of_range) {
        result = 1;
      }
    }
  }

  {
    auto flash = hal::linux::nv::FileNorFlash<256U, 1U, 64U>::open(
        {flash_path, true});
    if (!flash) {
      result = 1;
    } else {
      const std::array<std::byte, 4U> value{
          std::byte{0xF0}, std::byte{0x0F}, std::byte{0xAA}, std::byte{0x55}};
      const std::array<std::byte, 4U> invalid{
          std::byte{0xFF}, std::byte{0xFF}, std::byte{0xFF}, std::byte{0xFF}};
      std::array<std::byte, 4U> read{};
      if (!flash.value().program(0U, {value.data(), value.size()}) ||
          flash.value().program(0U, {invalid.data(), invalid.size()}) ||
          !flash.value().read(0U, {read.data(), read.size()}) ||
          read != value || !flash.value().erase(0U, 64U) ||
          !flash.value().read(0U, {read.data(), read.size()}) ||
          read != std::array<std::byte, 4U>{std::byte{0xFF}, std::byte{0xFF},
                                             std::byte{0xFF}, std::byte{0xFF}} ||
          !flash.value().sync()) {
        result = 1;
      }
    }
  }

  using Flash = hal::linux::nv::FileNorFlash<4096U, 1U, 256U>;
  using Memory = hal::nv::JournalMemory<Flash, 32U, 4U, 0U, 256U>;
  {
    auto flash = Flash::open({journal_path, true});
    std::array<std::byte, 32U> mirror{};
    if (!flash) {
      result = 1;
    } else {
      auto memory = Memory::mount(
          flash.value(), {mirror.data(), mirror.size()});
      const std::array<std::byte, 4U> value{
          std::byte{0x01}, std::byte{0x02}, std::byte{0x03}, std::byte{0x04}};
      if (!memory || !memory.value().write(8U, {value.data(), value.size()}) ||
          !memory.value().sync()) {
        result = 1;
      }
    }
  }
  {
    auto flash = Flash::open({journal_path, false});
    std::array<std::byte, 32U> mirror{};
    std::array<std::byte, 4U> value{};
    if (!flash) {
      result = 1;
    } else {
      auto memory = Memory::mount(
          flash.value(), {mirror.data(), mirror.size()});
      if (!memory || !memory.value().read(8U, {value.data(), value.size()}) ||
          value != std::array<std::byte, 4U>{std::byte{0x01}, std::byte{0x02},
                                             std::byte{0x03}, std::byte{0x04}}) {
        result = 1;
      }
    }
  }

  (void)::unlink(block_path);
  (void)::unlink(flash_path);
  (void)::unlink(journal_path);
  return result;
}
