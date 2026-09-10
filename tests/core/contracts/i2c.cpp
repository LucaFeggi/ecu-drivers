#include <array>
#include <hal/i2c.hpp>

namespace {

struct Error {
  hal::i2c::error_kind kind() const noexcept;
};

struct Controller {
  using error_type = Error;
  std::array<hal::i2c::direction, 2U> directions{};
  std::size_t count{};
  bool used_ten_bit{};

  [[nodiscard]] auto transaction(
      hal::i2c::address7, hal::span<const hal::i2c::operation> operations)
      -> hal::result<void, error_type> {
    return record(false, operations);
  }
  [[nodiscard]] auto transaction(
      hal::i2c::address10, hal::span<const hal::i2c::operation> operations)
      -> hal::result<void, error_type> {
    return record(true, operations);
  }

 private:
  [[nodiscard]] auto record(bool ten_bit,
                            hal::span<const hal::i2c::operation> operations)
      -> hal::result<void, error_type> {
    used_ten_bit = ten_bit;
    count = operations.size();
    for (std::size_t index = 0U; index < operations.size(); ++index) {
      directions[index] = operations[index].dir();
    }
    return hal::result<void, error_type>::success();
  }
};

static_assert(hal::i2c::Controller7<Controller>);
static_assert(hal::i2c::Controller10<Controller>);
static_assert(hal::i2c::Controller<Controller, hal::i2c::address7>);
static_assert(hal::i2c::Controller<Controller, hal::i2c::address10>);

}  // namespace

int main() {
  Controller controller{};
  const std::array<std::byte, 1U> tx{std::byte{0xA5}};
  std::array<std::byte, 2U> rx{};

  if (!hal::i2c::write(controller, hal::i2c::address7{0x42U}, tx) ||
      controller.used_ten_bit || controller.count != 1U ||
      controller.directions[0U] != hal::i2c::direction::write) {
    return 1;
  }
  if (!hal::i2c::read(controller, hal::i2c::address10{0x2AAU}, rx) ||
      !controller.used_ten_bit || controller.count != 1U ||
      controller.directions[0U] != hal::i2c::direction::read) {
    return 2;
  }
  if (!hal::i2c::write_read(controller, hal::i2c::address7{0x42U}, tx, rx) ||
      controller.used_ten_bit || controller.count != 2U ||
      controller.directions[0U] != hal::i2c::direction::write ||
      controller.directions[1U] != hal::i2c::direction::read) {
    return 3;
  }
  return 0;
}
