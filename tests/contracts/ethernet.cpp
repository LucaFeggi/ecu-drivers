#include <cstdint>
#include <hal/contracts/ethernet.hpp>

namespace {

struct Error {
  hal::ethernet::error_kind kind() const noexcept;
};

struct Mdio {
  using error_type = Error;
  hal::result<std::uint16_t, error_type> read_clause22(std::uint8_t,
                                                       std::uint8_t);
  hal::result<void, error_type> write_clause22(std::uint8_t, std::uint8_t,
                                               std::uint16_t);
};

struct Phy {
  using error_type = Error;
  hal::result<hal::ethernet::link_state, error_type> link();
  hal::result<void, error_type> restart_autonegotiation();
};

struct Mac {
  using error_type = Error;
  hal::result<void, error_type> set_address(hal::ethernet::mac_address);
  hal::result<void, error_type> configure_link(hal::ethernet::link_state);
  hal::result<bool, error_type> try_transmit(hal::span<const std::byte>);
  hal::result<hal::ethernet::received_frame, error_type> try_receive(
      hal::span<std::byte>);
};

struct FramePort {
  using error_type = Error;
  hal::ethernet::mac_address address() noexcept;
  hal::result<hal::ethernet::link_state, error_type> link();
  hal::result<bool, error_type> try_transmit(hal::span<const std::byte>);
  hal::result<hal::ethernet::received_frame, error_type> try_receive(
      hal::span<std::byte>);
  hal::result<hal::ethernet::timestamped_frame, error_type>
      try_receive_timestamped(hal::span<std::byte>);
};

static_assert(hal::ethernet::MdioBus<Mdio>);
static_assert(hal::ethernet::Phy<Phy>);
static_assert(hal::ethernet::Mac<Mac>);
static_assert(hal::ethernet::FramePort<FramePort>);
static_assert(hal::ethernet::PtpFramePort<FramePort>);

}  // namespace

int main() {}
