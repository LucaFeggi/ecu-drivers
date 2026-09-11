#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <hal/contracts/ethernet.hpp>
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <hal/foundation/dma.hpp>
#include "support.hpp"

namespace hal::stm32h7::ethernet {

inline constexpr capability mdio_support{
    implementation_status::available, "direct IEEE 802.3 Clause 22 MDIO"};
inline constexpr capability mac_support{
    implementation_status::available,
    "direct STM32H7 Ethernet MAC/DMA descriptor rings"};

class error {
public:
  explicit constexpr error(hal::ethernet::error_kind kind) noexcept : kind_{kind} {}

  [[nodiscard]] constexpr hal::ethernet::error_kind kind() const noexcept {
    return kind_;
  }

private:
  hal::ethernet::error_kind kind_{};
};

struct mdio_config {
  std::uint32_t peripheral_clock_hz{};
  poll_budget timeout{};
};

template <class Registers> class MdioBus {
public:
  using error_type = error;

  MdioBus(Registers &registers, mdio_config configuration) noexcept
      : registers_{registers}, configuration_{configuration} {}

  MdioBus(const MdioBus &) = delete;
  MdioBus &operator=(const MdioBus &) = delete;

  [[nodiscard]] auto read_clause22(std::uint8_t phy, std::uint8_t reg)
      -> result<std::uint16_t, error_type> {
    if (!valid_configuration() || phy >= 32U || reg >= 32U) {
      return result<std::uint16_t, error_type>::failure(
          error{hal::ethernet::error_kind::io});
    }
    if (!wait_idle()) {
      return result<std::uint16_t, error_type>::failure(
          error{hal::ethernet::error_kind::would_block});
    }
    registers_.MACMDIOAR = mdio_clock_code() | (static_cast<std::uint32_t>(phy) << 21U) |
                            (static_cast<std::uint32_t>(reg) << 16U) |
                            mdio_read | mdio_busy;
    if (!wait_idle()) {
      return result<std::uint16_t, error_type>::failure(
          error{hal::ethernet::error_kind::io});
    }
    return result<std::uint16_t, error_type>::success(
        static_cast<std::uint16_t>(registers_.MACMDIODR & 0xFFFFU));
  }

  [[nodiscard]] auto write_clause22(std::uint8_t phy, std::uint8_t reg,
                                    std::uint16_t value)
      -> result<void, error_type> {
    if (!valid_configuration() || phy >= 32U || reg >= 32U) {
      return result<void, error_type>::failure(
          error{hal::ethernet::error_kind::io});
    }
    if (!wait_idle()) {
      return result<void, error_type>::failure(
          error{hal::ethernet::error_kind::would_block});
    }
    registers_.MACMDIODR = value;
    registers_.MACMDIOAR = mdio_clock_code() | (static_cast<std::uint32_t>(phy) << 21U) |
                            (static_cast<std::uint32_t>(reg) << 16U) |
                            mdio_write | mdio_busy;
    return wait_idle() ? result<void, error_type>::success()
                       : result<void, error_type>::failure(
                             error{hal::ethernet::error_kind::io});
  }

private:
  static constexpr std::uint32_t mdio_busy{1U << 0U};
  static constexpr std::uint32_t mdio_read{3U << 2U};
  static constexpr std::uint32_t mdio_write{1U << 2U};

  [[nodiscard]] bool wait_idle() const noexcept {
    for (std::uint32_t remaining = configuration_.timeout.iterations; remaining > 0U;
         --remaining) {
      if ((registers_.MACMDIOAR & mdio_busy) == 0U) {
        return true;
      }
    }
    return false;
  }

  [[nodiscard]] std::uint32_t mdio_clock_code() const noexcept {
    const std::uint32_t clock = configuration_.peripheral_clock_hz;
    // Select the smallest legal divider whose MDC is no faster than 2.5 MHz.
    constexpr std::uint32_t divisors[] = {42U, 62U, 16U, 26U, 102U, 124U,
                                          4U, 8U, 10U, 12U, 14U, 16U, 18U};
    constexpr std::uint32_t codes[] = {0x000U, 0x100U, 0x200U, 0x300U, 0x400U,
                                       0x500U, 0x800U, 0xA00U, 0xB00U, 0xC00U,
                                       0xD00U, 0xE00U, 0xF00U};
    for (std::size_t index = 0U; index < sizeof(divisors) / sizeof(divisors[0U]);
         ++index) {
      if (clock / divisors[index] <= 2'500'000U) {
        return codes[index];
      }
    }
    return codes[5U]; // DIV124 is the slowest legal H7 MDC divider.
  }

  [[nodiscard]] bool valid_configuration() const noexcept {
    return configuration_.timeout.valid() &&
           configuration_.peripheral_clock_hz > 0U &&
           configuration_.peripheral_clock_hz <= 310'000'000U;
  }

  Registers &registers_;
  mdio_config configuration_{};
};

struct unknown_phy_status_decoder {
  template <class Mdio>
  [[nodiscard]] static hal::ethernet::link_state
  decode(Mdio &, std::uint8_t, bool up) noexcept {
    return {up, hal::ethernet::speed::unknown, hal::ethernet::duplex::full};
  }
};

template <class Mdio, std::uint8_t Address,
          class StatusDecoder = unknown_phy_status_decoder>
class Phy {
public:
  using error_type = typename Mdio::error_type;

  explicit Phy(Mdio &mdio) noexcept : mdio_{mdio} {}

  [[nodiscard]] auto link() -> result<hal::ethernet::link_state, error_type> {
    auto first = mdio_.read_clause22(Address, bmsr);
    if (!first) {
      return result<hal::ethernet::link_state, error_type>::failure(first.error());
    }
    // BMSR link status is latched low; read it twice per Clause 22.
    auto second = mdio_.read_clause22(Address, bmsr);
    if (!second) {
      return result<hal::ethernet::link_state, error_type>::failure(second.error());
    }
    const bool up = (second.value() & (1U << 2U)) != 0U;
    return result<hal::ethernet::link_state, error_type>::success(
        StatusDecoder::decode(mdio_, Address, up));
  }

  [[nodiscard]] auto restart_autonegotiation() -> result<void, error_type> {
    auto control = mdio_.read_clause22(Address, bmcr);
    if (!control) {
      return result<void, error_type>::failure(control.error());
    }
    return mdio_.write_clause22(Address, bmcr,
                                static_cast<std::uint16_t>(control.value() |
                                                           (1U << 9U) | (1U << 12U)));
  }

private:
  static constexpr std::uint8_t bmcr{0U};
  static constexpr std::uint8_t bmsr{1U};
  Mdio &mdio_;
};

// Hardware descriptor lists require 16-byte alignment. A cache-maintenance
// policy may impose the stronger cache-line alignment through valid_region().
struct alignas(16U) descriptor {
  volatile std::uint32_t DESC0{};
  volatile std::uint32_t DESC1{};
  volatile std::uint32_t DESC2{};
  volatile std::uint32_t DESC3{};
};

static_assert(sizeof(descriptor) == 16U);
static_assert(alignof(descriptor) == 16U);

struct mac_config {
  std::uint16_t maximum_frame_size{1536U};
  std::uint8_t tx_dma_burst{8U};
  std::uint8_t rx_dma_burst{8U};
  poll_budget timeout{};
};

template <class Registers, std::size_t TxCount, std::size_t RxCount,
          std::size_t BufferSize = 1536U,
          hal::DmaCoherencyPolicy DmaPolicy = hal::coherent_dma_policy>
class Mac {
  static_assert(TxCount > 0U && RxCount > 0U);
  static_assert(BufferSize >= 64U);

public:
  using error_type = error;

  Mac(Registers &registers, descriptor (&tx_descriptors)[TxCount],
      descriptor (&rx_descriptors)[RxCount],
      std::array<std::byte, TxCount * BufferSize> &tx_buffers,
      std::array<std::byte, RxCount * BufferSize> &rx_buffers,
      mac_config configuration, DmaPolicy dma_policy = {}) noexcept
      : registers_{registers}, tx_descriptors_{tx_descriptors},
        rx_descriptors_{rx_descriptors}, tx_buffers_{tx_buffers},
        rx_buffers_{rx_buffers}, configuration_{configuration},
        dma_policy_{dma_policy} {}

  Mac(const Mac &) = delete;
  Mac &operator=(const Mac &) = delete;

  [[nodiscard]] auto initialize() -> result<void, error_type> {
    if (!configuration_.timeout.valid() ||
        configuration_.maximum_frame_size > BufferSize || BufferSize > 0x3FFFU ||
        (BufferSize % 4U) != 0U || !valid_burst(configuration_.tx_dma_burst) ||
        !valid_burst(configuration_.rx_dma_burst) ||
        (reinterpret_cast<std::uintptr_t>(tx_descriptors_) % 16U) != 0U ||
        (reinterpret_cast<std::uintptr_t>(rx_descriptors_) % 16U) != 0U ||
        (reinterpret_cast<std::uintptr_t>(tx_buffers_.data()) % 4U) != 0U ||
        (reinterpret_cast<std::uintptr_t>(rx_buffers_.data()) % 4U) != 0U ||
        !dma_regions_valid()) {
      return failure(hal::ethernet::error_kind::io);
    }
    registers_.DMAMR = dma_software_reset;
    if (!wait_dma_reset()) {
      return failure(hal::ethernet::error_kind::io);
    }
    registers_.DMACCR = dma_8x_pbl;
    for (std::size_t index = 0U; index < TxCount; ++index) {
      tx_descriptors_[index].DESC0 = pointer_word(tx_buffer(index));
      tx_descriptors_[index].DESC1 = 0U;
      tx_descriptors_[index].DESC2 = 0U;
      tx_descriptors_[index].DESC3 = 0U;
    }
    for (std::size_t index = 0U; index < RxCount; ++index) {
      rx_descriptors_[index].DESC0 = pointer_word(rx_buffer(index));
      rx_descriptors_[index].DESC1 = 0U;
      rx_descriptors_[index].DESC2 = 0U;
      rx_descriptors_[index].DESC3 = descriptor_own | rx_buffer1_valid;
    }
    dma_policy_.prepare_for_device(tx_descriptors_, sizeof(tx_descriptors_));
    dma_policy_.prepare_for_device(rx_descriptors_, sizeof(rx_descriptors_));
    dma_policy_.prepare_for_device(rx_buffers_.data(), rx_buffers_.size());
    registers_.DMACRDLAR = pointer_word(rx_descriptors_);
    registers_.DMACTDLAR = pointer_word(tx_descriptors_);
    registers_.DMACRDRLR = static_cast<std::uint32_t>(RxCount - 1U);
    registers_.DMACTDRLR = static_cast<std::uint32_t>(TxCount - 1U);
    registers_.DMACTDTPR = pointer_word(&tx_descriptors_[0U]);
    registers_.DMACRDTPR = pointer_word(&rx_descriptors_[RxCount - 1U]);
    registers_.DMACTCR = dma_burst_length(configuration_.tx_dma_burst) |
                         transmit_start;
    registers_.DMACRCR = dma_burst_length(configuration_.rx_dma_burst) |
                         (static_cast<std::uint32_t>(BufferSize) << 1U) |
                         receive_start;
    barrier();
    initialized_ = true;
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto set_address(hal::ethernet::mac_address address)
      -> result<void, error_type> {
    address_ = address;
    registers_.MACA0LR = to_word(address.bytes[0U], address.bytes[1U],
                                  address.bytes[2U], address.bytes[3U]);
    registers_.MACA0HR = static_cast<std::uint32_t>(
        std::to_integer<std::uint8_t>(address.bytes[4U])) |
                         (static_cast<std::uint32_t>(
                              std::to_integer<std::uint8_t>(address.bytes[5U]))
                          << 8U);
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto configure_link(hal::ethernet::link_state state)
      -> result<void, error_type> {
    if (!initialized_ || !state.up) {
      return failure(state.up ? hal::ethernet::error_kind::io
                              : hal::ethernet::error_kind::link_down);
    }
    // Strip the Ethernet FCS in hardware so received_frame::size describes
    // the frame delivered to the network stack, not the on-wire CRC bytes.
    std::uint32_t value = mac_receiver_enable | mac_transmitter_enable |
                          mac_automatic_pad_crc_strip | mac_crc_strip_type;
    if (state.link_duplex == hal::ethernet::duplex::full) {
      value |= mac_full_duplex;
    }
    if (state.link_speed == hal::ethernet::speed::mbps100) {
      value |= mac_fast_ethernet;
    } else if (state.link_speed != hal::ethernet::speed::mbps10) {
      return failure(hal::ethernet::error_kind::io);
    }
    registers_.MACCR = value;
    return result<void, error_type>::success();
  }

  [[nodiscard]] auto try_transmit(hal::span<const std::byte> data)
      -> result<bool, error_type> {
    if (!initialized_ || !data.valid()) {
      return result<bool, error_type>::failure(error{hal::ethernet::error_kind::io});
    }
    if (data.size() > configuration_.maximum_frame_size ||
        data.size() > BufferSize) {
      return result<bool, error_type>::failure(
          error{hal::ethernet::error_kind::frame_too_large});
    }
    descriptor &entry = tx_descriptors_[tx_index_];
    dma_policy_.prepare_for_cpu(&entry, sizeof(entry));
    if ((entry.DESC3 & descriptor_own) != 0U) {
      return result<bool, error_type>::success(false);
    }
    std::byte *destination = tx_buffer(tx_index_);
    for (std::size_t index = 0U; index < data.size(); ++index) {
      destination[index] = data[index];
    }
    entry.DESC0 = pointer_word(destination);
    entry.DESC1 = 0U;
    entry.DESC2 = static_cast<std::uint32_t>(data.size()) & tx_buffer1_length_mask;
    entry.DESC3 = tx_first_segment | tx_last_segment | descriptor_own;
    dma_policy_.prepare_for_device(destination, data.size());
    dma_policy_.prepare_for_device(&entry, sizeof(entry));
    barrier();
    tx_index_ = (tx_index_ + 1U) % TxCount;
    barrier();
    registers_.DMACTDTPR = pointer_word(&tx_descriptors_[tx_index_]);
    return result<bool, error_type>::success(true);
  }

  [[nodiscard]] auto try_receive(hal::span<std::byte> output)
      -> result<hal::ethernet::received_frame, error_type> {
    if (!initialized_ || !output.valid()) {
      return result<hal::ethernet::received_frame, error_type>::failure(
          error{hal::ethernet::error_kind::io});
    }
    descriptor &entry = rx_descriptors_[rx_index_];
    dma_policy_.prepare_for_cpu(&entry, sizeof(entry));
    barrier();
    if ((entry.DESC3 & descriptor_own) != 0U) {
      return result<hal::ethernet::received_frame, error_type>::success(
          {false, 0U});
    }
    const std::uint32_t status = entry.DESC3;
    const std::size_t frame_size = static_cast<std::size_t>(status & rx_packet_length_mask);
    if ((status & rx_error_summary) != 0U ||
        (status & (rx_first_segment | rx_last_segment)) !=
            (rx_first_segment | rx_last_segment) ||
        frame_size > BufferSize) {
      recycle_rx(entry);
      return result<hal::ethernet::received_frame, error_type>::failure(
          error{hal::ethernet::error_kind::io});
    }
    if (output.size() < frame_size) {
      // There is no separate discard operation in hal-core. Recycle the
      // descriptor so an undersized caller buffer cannot stall the RX ring.
      recycle_rx(entry);
      return result<hal::ethernet::received_frame, error_type>::failure(
          error{hal::ethernet::error_kind::frame_too_large});
    }
    std::byte *source = rx_buffer(rx_index_);
    dma_policy_.prepare_for_cpu(source, frame_size);
    for (std::size_t index = 0U; index < frame_size; ++index) {
      output[index] = source[index];
    }
    recycle_rx(entry);
    return result<hal::ethernet::received_frame, error_type>::success(
        {true, frame_size});
  }

  [[nodiscard]] hal::ethernet::mac_address address() const noexcept {
    return address_;
  }

private:
  static constexpr std::uint32_t dma_software_reset{1U << 0U};
  static constexpr std::uint32_t dma_8x_pbl{1U << 16U};
  static constexpr std::uint32_t descriptor_own{1U << 31U};
  static constexpr std::uint32_t tx_first_segment{1U << 29U};
  static constexpr std::uint32_t tx_last_segment{1U << 28U};
  static constexpr std::uint32_t rx_first_segment{1U << 29U};
  static constexpr std::uint32_t rx_last_segment{1U << 28U};
  static constexpr std::uint32_t rx_buffer1_valid{1U << 24U};
  static constexpr std::uint32_t rx_error_summary{1U << 15U};
  static constexpr std::uint32_t rx_packet_length_mask{0x7FFFU};
  static constexpr std::uint32_t tx_buffer1_length_mask{0x3FFFU};
  static constexpr std::uint32_t mac_receiver_enable{1U << 0U};
  static constexpr std::uint32_t mac_transmitter_enable{1U << 1U};
  static constexpr std::uint32_t mac_full_duplex{1U << 13U};
  static constexpr std::uint32_t mac_fast_ethernet{1U << 14U};
  static constexpr std::uint32_t mac_automatic_pad_crc_strip{1U << 20U};
  static constexpr std::uint32_t mac_crc_strip_type{1U << 21U};
  static constexpr std::uint32_t receive_start{1U};
  static constexpr std::uint32_t transmit_start{1U};

  [[nodiscard]] static constexpr bool valid_burst(std::uint8_t burst) noexcept {
    return burst == 1U || burst == 2U || burst == 4U || burst == 8U ||
           burst == 16U || burst == 32U;
  }

  [[nodiscard]] static constexpr std::uint32_t
  dma_burst_length(std::uint8_t burst) noexcept {
    return static_cast<std::uint32_t>(burst) << 16U;
  }

  [[nodiscard]] bool wait_dma_reset() const noexcept {
    for (std::uint32_t remaining = configuration_.timeout.iterations;
         remaining > 0U; --remaining) {
      if ((registers_.DMAMR & dma_software_reset) == 0U) {
        return true;
      }
    }
    return false;
  }

  static void barrier() noexcept {
#if defined(__arm__) || defined(__thumb__)
    __asm volatile("dsb 0xF" ::: "memory");
#else
    std::atomic_thread_fence(std::memory_order_seq_cst);
#endif
  }

  [[nodiscard]] static std::uint32_t pointer_word(const volatile void *pointer) noexcept {
    return static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(pointer));
  }

  [[nodiscard]] std::byte *tx_buffer(std::size_t index) noexcept {
    return tx_buffers_.data() + index * BufferSize;
  }
  [[nodiscard]] std::byte *rx_buffer(std::size_t index) noexcept {
    return rx_buffers_.data() + index * BufferSize;
  }

  void recycle_rx(descriptor &entry) noexcept {
    entry.DESC3 = descriptor_own | rx_buffer1_valid;
    dma_policy_.prepare_for_device(&entry, sizeof(entry));
    barrier();
    rx_index_ = (rx_index_ + 1U) % RxCount;
    registers_.DMACRDTPR = pointer_word(
        &rx_descriptors_[(rx_index_ + RxCount - 1U) % RxCount]);
  }

  [[nodiscard]] static std::uint32_t to_word(std::byte a, std::byte b,
                                             std::byte c, std::byte d) noexcept {
    return static_cast<std::uint32_t>(std::to_integer<std::uint8_t>(a)) |
           (static_cast<std::uint32_t>(std::to_integer<std::uint8_t>(b)) << 8U) |
           (static_cast<std::uint32_t>(std::to_integer<std::uint8_t>(c)) << 16U) |
           (static_cast<std::uint32_t>(std::to_integer<std::uint8_t>(d)) << 24U);
  }

  [[nodiscard]] auto failure(hal::ethernet::error_kind kind)
      -> result<void, error_type> {
    return result<void, error_type>::failure(error{kind});
  }

  [[nodiscard]] bool dma_regions_valid() const noexcept {
    return dma_policy_.valid_region(tx_descriptors_, sizeof(tx_descriptors_)) &&
           dma_policy_.valid_region(rx_descriptors_, sizeof(rx_descriptors_)) &&
           dma_policy_.valid_region(tx_buffers_.data(), tx_buffers_.size()) &&
           dma_policy_.valid_region(rx_buffers_.data(), rx_buffers_.size());
  }

  Registers &registers_;
  descriptor (&tx_descriptors_)[TxCount];
  descriptor (&rx_descriptors_)[RxCount];
  std::array<std::byte, TxCount * BufferSize> &tx_buffers_;
  std::array<std::byte, RxCount * BufferSize> &rx_buffers_;
  mac_config configuration_{};
  DmaPolicy dma_policy_{};
  hal::ethernet::mac_address address_{};
  std::size_t tx_index_{};
  std::size_t rx_index_{};
  bool initialized_{};
};

template <class MacType, class PhyType> class FramePort {
public:
  using error_type = typename MacType::error_type;

  FramePort(MacType &mac, PhyType &phy) noexcept : mac_{mac}, phy_{phy} {}

  [[nodiscard]] auto address() const noexcept { return mac_.address(); }

  [[nodiscard]] auto link() -> result<hal::ethernet::link_state, error_type> {
    return phy_.link();
  }

  [[nodiscard]] auto try_transmit(hal::span<const std::byte> data)
      -> result<bool, error_type> {
    return mac_.try_transmit(data);
  }

  [[nodiscard]] auto try_receive(hal::span<std::byte> data)
      -> result<hal::ethernet::received_frame, error_type> {
    return mac_.try_receive(data);
  }

private:
  MacType &mac_;
  PhyType &phy_;
};

} // namespace hal::stm32h7::ethernet
