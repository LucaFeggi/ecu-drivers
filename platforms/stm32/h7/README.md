# hal-target-stm32h7

MCU-family target package for direct-register STM32H7 implementations of the
`ecu-drivers` contracts.

The package owns reusable STM32H7 peripheral code. Exact device and package facts are supplied
separately by the selected chip profile. It does not own PCB pins, oscillators, external
transceivers, application policy, or FreeRTOS integration.

ST's CMSIS device headers v1.10.7 and the matching ARM CMSIS v5.9.0 package are pinned under
`third_party/`. They supply register and Cortex-M declarations only; this target does not use
STM32Cube HAL.

## Status

The target package contains direct-register implementations for GPIO/EXTI, the timer clock and delay,
ADC1/2 timer-triggered circular DMA, polling and DMA USART, polling and DMA SPI, DMA-capable I2C,
FDCAN message-RAM FIFOs, PWM, RTC/alarm, IWDG, internal flash, SDMMC IDMA, and the H7 Ethernet
MAC/MDIO descriptor engine. No STM32Cube HAL or LL code is used.

The host conformance build is passing. Hardware-in-the-loop validation is still required for every
peripheral, especially clock/reset sequencing, DMA request routing, cache/MPU placement, PHY link
negotiation, SD-card behavior, flash execution/cache policy, and exact board wiring. PTP and a generic
PHY speed/duplex decoder are intentionally not claimed as implemented; the BSP supplies a PHY-specific
status decoder when the external PHY requires one.

## Integration

```cmake
add_subdirectory(path/to/ecu-drivers)
target_link_libraries(firmware PRIVATE <selected-stm32h7-device-target>)
```

The BSP constructs target objects with its selected register instance, pin routing, clocks, DMA
storage, interrupt routing, memory attributes, and bounded timeout policy. DMA buffers and Ethernet
descriptor rings are caller-owned so the BSP can place them in memory visible to the relevant DMA and
apply the required cache policy.

The ADC engine accepts register instances and BSP-owned DMA storage. It performs regulator startup,
ADC1/2 linearity calibration, external-trigger and oversampling configuration, DMAMUX/DMA setup,
half/complete interrupt publication, and bounded stop/error handling. Its concrete channel provides
the current `hal::adc::Channel` (`read_raw()`/`read_voltage()`) contract, and the continuous publisher
provides `hal::adc::ContinuousChannel::try_read()` over `raw_sample` values. The BSP must make DMA
storage coherent, normally with a non-cacheable MPU region.

DMA-capable synchronous core contracts use DMA for the data plane and a bounded completion check before
returning. A genuinely interrupt-only completion path would require an asynchronous operation contract
in `hal-core`; the target cannot return `result<void>` before the transfer is complete. Continuous
acquisition and receive rings use DMA half/complete notifications because they have no synchronous
completion boundary.

See [System Architecture](docs/System_Architecture.md) for the driver-level
implementation and ownership model.
