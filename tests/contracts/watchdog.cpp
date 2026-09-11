#include <hal/contracts/watchdog.hpp>

namespace {

struct Error {
  hal::watchdog::error_kind kind() const noexcept;
};

struct Feeder {
  using error_type = Error;
  static hal::watchdog::characteristics properties() noexcept;
  hal::result<void, error_type> feed();
};

static_assert(hal::watchdog::Feeder<Feeder>);

}  // namespace

int main() {}
