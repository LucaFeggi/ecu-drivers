#include <array>
#include <cstddef>
#include <hal/serial.hpp>
#include <pty.h>
#include <time.h>
#include <unistd.h>

int main() {
  int master = -1;
  int slave = -1;
  if (::openpty(&master, &slave, nullptr, nullptr, nullptr) < 0) {
    return 1;
  }
  char* slave_name = ::ttyname(slave);
  if (slave_name == nullptr) {
    (void)::close(master);
    (void)::close(slave);
    return 1;
  }

  auto port = hal::linux::serial::Port::open({slave_name});
  if (!port) {
    (void)::close(master);
    (void)::close(slave);
    return 1;
  }
  const auto configured = port.value().configure(
      {hal::hertz{115200U}, hal::serial::data_bits::bits8,
       hal::serial::parity::none, hal::serial::stop_bits::one,
       hal::serial::flow_control::none});
  if (!configured) {
    (void)::close(master);
    (void)::close(slave);
    return 1;
  }

  const std::array<std::byte, 4U> tx{
      std::byte{'t'}, std::byte{'e'}, std::byte{'s'}, std::byte{'t'}};
  const auto written = port.value().try_write({tx.data(), tx.size()});
  if (!written || written.value() != tx.size() || !port.value().flush()) {
    (void)::close(master);
    (void)::close(slave);
    return 1;
  }

  std::array<std::byte, 4U> received{};
  const ssize_t count = ::read(master, received.data(), received.size());
  if (count != static_cast<ssize_t>(received.size()) || received != tx) {
    (void)::close(master);
    (void)::close(slave);
    return 1;
  }

  const std::array<std::byte, 4U> reply{
      std::byte{'p'}, std::byte{'o'}, std::byte{'n'}, std::byte{'g'}};
  if (::write(master, reply.data(), reply.size()) !=
      static_cast<ssize_t>(reply.size())) {
    (void)::close(master);
    (void)::close(slave);
    return 1;
  }
  std::array<std::byte, 4U> rx{};
  std::size_t received_count = 0U;
  for (unsigned int attempt = 0U; attempt < 100U && received_count < rx.size();
       ++attempt) {
    const auto read_result = port.value().try_read(
        {rx.data() + received_count, rx.size() - received_count});
    if (!read_result) {
      (void)::close(master);
      (void)::close(slave);
      return 1;
    }
    received_count += read_result.value();
    if (received_count == rx.size()) {
      break;
    }
    ::timespec delay{0, 1'000'000};
    (void)::nanosleep(&delay, nullptr);
  }

  (void)::close(master);
  (void)::close(slave);
  return received_count == rx.size() && rx == reply ? 0 : 1;
}
