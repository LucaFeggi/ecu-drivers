#include <array>
#include <hal/can.hpp>

namespace {

struct Error {
  hal::can::error_kind kind() const noexcept;
};

struct ClassicController {
  using error_type = Error;
  hal::result<bool, error_type> try_send(const hal::can::classic_frame&);
  hal::result<hal::can::polled_frame<hal::can::classic_frame>, error_type>
  try_receive();
  hal::result<void, error_type> set_filters(hal::span<const hal::can::filter>);
};

struct FdController : ClassicController {
  hal::result<bool, error_type> try_send_fd(const hal::can::fd_frame&);
  hal::result<hal::can::polled_frame<hal::can::fd_frame>, error_type>
  try_receive_fd();
};

static_assert(hal::can::ClassicController<ClassicController>);
static_assert(!hal::can::FdController<ClassicController>);
static_assert(hal::can::FilterableController<ClassicController>);
static_assert(hal::can::FdController<FdController>);

}  // namespace

int main() {
  int failures = 0;
  const auto check = [&failures](bool condition) {
    if (!condition) {
      ++failures;
    }
  };

  std::array<std::byte, 2U> payload{std::byte{0x12}, std::byte{0x34}};
  auto made = hal::can::classic_frame::make({0x123U, false}, payload);
  check(made.has_value());
  if (made) {
    auto empty = hal::can::polled_frame<hal::can::classic_frame>::unavailable();
    check(!empty.has_frame());

    auto available =
        hal::can::polled_frame<hal::can::classic_frame>::available(
            std::move(made).value());
    check(available.has_frame());
    check(available.value().frame_id().value == 0x123U);
    check(available.value().data().size() == payload.size());
  }

  const hal::can::filter standard{{0x120U, false}, 0x7F0U};
  check(standard.valid());
  check(standard.matches({0x12FU, false}));
  check(!standard.matches({0x12FU, true}));
  check(!hal::can::filter{{0x120U, false}, 0x800U}.valid());

  std::array<std::byte, 64U> fd_payload{};
  constexpr std::array<std::size_t, 9U> fd_lengths{0U, 8U, 12U, 16U, 20U,
                                                     24U, 32U, 48U, 64U};
  for (const std::size_t length : fd_lengths) {
    auto frame = hal::can::fd_frame::make(
        {0x1ABCDEU, true}, {fd_payload.data(), length});
    check(frame.has_value());
  }
  check(!hal::can::fd_frame::make({0x1ABCDEU, true},
                                  {fd_payload.data(), 10U})
             .has_value());

  return failures;
}
