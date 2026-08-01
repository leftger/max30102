# MAX30102 Rust Driver

[![Crates.io](https://img.shields.io/crates/v/max30102.svg)](https://crates.io/crates/max30102)
[![Documentation](https://docs.rs/max30102/badge.svg)](https://docs.rs/max30102)
[![License](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue.svg)](LICENSE-MIT)

A `#![no_std]` Rust driver for the **MAX30102** pulse oximeter and heart rate sensor. Built on top of `embedded-hal` 1.0 and `embedded-hal-async` with type-safe register generation via `device-driver`.

---

## Overview

The MAX30102 is an integrated optical sensor module combining two LEDs (Red 660nm and Infrared 880nm), photodetectors, ambient light cancellation, low-noise electronics, and an 18-bit ADC. It includes an internal 32-sample circular FIFO buffer and an on-chip die temperature sensor.

This driver provides both **blocking** and **asynchronous** interfaces for embedded Rust applications with zero allocation overhead.

### Key Features

* ⚡ **High-Performance Burst Reads**: Performs single-transaction I²C burst reads from the FIFO buffer to minimize bus overhead and power consumption.
* 🔒 **Type-Safe Configuration**: Strongly typed enums for sample rates, pulse widths, ADC resolution, sample averaging, and multi-LED slots generated from a verified YAML register manifest.
* ⚡ **Blocking & Async Support**: First-class support for `embedded-hal` 1.0 (`I2c`) and `embedded-hal-async` (`AsyncI2c`).
* 🎯 **Convenience Utilities**:
  * Direct LED current configuration in milliamps (`0.0 mA` to `51.0 mA`).
  * One-shot temperature measurement with `DelayNs`.
  * Red LED thermal self-heating estimation based on drive current and duty cycle.
  * Structured interrupt flag reading and FIFO overflow counter tracking.
* 📦 **`no_std` Ready**: Compatible with bare-metal microcontrollers (Cortex-M, RISC-V, ESP32, STM32, nRF, RP2040, etc.).
* 📊 **Optional `defmt` Logging**: Integrated `defmt-03` support for debugging on embedded targets.

---

## Installation

Add `max30102` to your `Cargo.toml`:

```toml
[dependencies]
max30102 = "0.1.1"
```

### Feature Flags

| Feature | Description |
| :--- | :--- |
| `async` | Enables `Max30102Async` driver backed by `embedded-hal-async` |
| `defmt-03` | Implements `defmt::Format` for driver types and register field sets |
| `embassy` | Convenience alias enabling both `async` and `defmt-03` |

```toml
# Example for Embassy / Async usage
[dependencies]
max30102 = { version = "0.1.1", features = ["embassy"] }
```

---

## Usage Guide

### 1. SpO₂ Measurement (Blocking)

Standard configuration using Red (660nm) and IR (880nm) LEDs to capture photoplethysmogram (PPG) signals for SpO₂ and heart rate calculations:

```rust
use max30102::{
    Max30102, SlaveAddr, OperatingMode, SampleRate, PulseWidth, AdcRange, SampleAveraging
};
use embedded_hal::i2c::I2c;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 1. Instantiate your I2C peripheral from your HAL
    let i2c = /* i2c peripheral */;

    // 2. Initialize the MAX30102 driver
    let mut sensor = Max30102::new(i2c, SlaveAddr::Default)?;

    // 3. Verify hardware ID (0x15)
    if !sensor.verify_part_id()? {
        // Mismatched sensor identity
        return Err("MAX30102 sensor not found!".into());
    }

    // 4. Soft reset device to power-on state
    sensor.reset()?;

    // 5. Configure acquisition parameters
    sensor.set_sample_rate(SampleRate::Rate100)?;     // 100 Hz sampling
    sensor.set_pulse_width(PulseWidth::Pw411)?;       // 411 µs pulse width (18-bit ADC)
    sensor.set_adc_range(AdcRange::Range4096)?;       // 4096 nA full-scale range
    sensor.set_sample_averaging(SampleAveraging::Avg4)?; // 4-sample moving average

    // 6. Set LED currents in milliamps (0.0 to 51.0 mA)
    sensor.set_led_current_ma(6.2, 6.2)?; // 6.2 mA for Red and IR

    // 7. Configure and clear FIFO
    sensor.set_fifo_rollover(true)?;
    sensor.clear_fifo()?;

    // 8. Start SpO2 Mode (Red + IR active)
    sensor.start_spo2_mode()?;

    // 9. Read samples loop
    loop {
        let available = sensor.get_fifo_samples_available()?;
        if available > 0 {
            // Burst read 18-bit raw ADC counts for (Red, IR)
            let (red, ir) = sensor.read_fifo_sample()?;
            // Process red and ir PPG values...
        }
    }
}
```

---

### 2. Heart Rate Only Mode (Low Power)

Uses only the Red LED to minimize power consumption when blood oxygen saturation is not required:

```rust
sensor.reset()?;
sensor.set_sample_rate(SampleRate::Rate100)?;
sensor.set_pulse_width(PulseWidth::Pw215)?; // 17-bit resolution
sensor.set_led_current_ma(6.2, 0.0)?;         // Red LED 6.2 mA, IR off
sensor.clear_fifo()?;
sensor.start_heart_rate_mode()?;
```

---

### 3. Multi-LED Mode with Slot Assignment

Multi-LED mode allows customized time-slot sequencing for advanced optical algorithms or proximity detection:

```rust
use max30102::Slot;

sensor.reset()?;
sensor.start_multi_led_mode()?;

// Configure time slots: Slot 1 = Red LED, Slot 2 = IR LED, Slot 3 & 4 = Disabled
sensor.set_multi_led_slots(Slot::Led1, Slot::Led2, Slot::None, Slot::None)?;
```

---

### 4. Die Temperature Measurement

The MAX30102 on-chip temperature sensor measures die temperature in °C with 0.0625°C resolution.

#### Option A: One-Shot Helper (with `DelayNs`)
```rust
// Triggers conversion, waits 30ms, and returns temperature in °C
let temp_c = sensor.measure_temperature(&mut delay)?;
```

#### Option B: Non-Blocking Conversion
```rust
// 1. Initiate temperature conversion
sensor.start_temperature_conversion()?;

// 2. Poll status (or wait for DIE_TEMP_RDY interrupt)
while !sensor.is_temperature_ready()? {
    // Perform other work...
}

// 3. Read measured temperature
let temp_c = sensor.read_temperature()?;
```

#### Thermal Self-Heating Estimation
Calculate Red LED thermal self-heating shift based on LED drive current and duty cycle:

```rust
use max30102::estimate_red_led_temperature;

let die_temp = sensor.read_temperature()?;
// Red LED current 25.4 mA at 16% duty cycle
let red_led_temp = estimate_red_led_temperature(die_temp, 25.4, 16.0);
```

---

### 5. Interrupt & Overflow Handling

```rust
// Set FIFO almost full threshold (triggers when 4 empty slots remain)
sensor.set_fifo_almost_full(4)?;
sensor.enable_fifo_almost_full_interrupt()?;
sensor.enable_alc_overflow_interrupt()?;

// In your interrupt service routine or polling loop:
let flags = sensor.read_interrupt_flags()?;

if flags.almost_full {
    // Read lost sample count if overflow occurred
    let overflow_samples = sensor.read_overflow_counter()?;
    
    // Read up to 28 available samples into buffer in 1 burst transaction
    let mut buffer = [0u8; 168]; // 28 samples * 6 bytes
    let bytes_read = sensor.read_fifo(&mut buffer)?;
}

if flags.alc_overflow {
    // Ambient Light Cancellation reached maximum limit
}
```

---

### 6. Asynchronous Driver (`embedded-hal-async` / Embassy)

Enable the `async` feature to use `Max30102Async`:

```rust
use max30102::{Max30102Async, SlaveAddr, SampleRate, PulseWidth};
use embassy_time::Timer;

#[embassy_executor::task]
async fn ppg_task(i2c: I2cDevice) {
    let mut sensor = Max30102Async::new(i2c, SlaveAddr::Default);

    if !sensor.verify_part_id().await.unwrap() {
        return;
    }

    sensor.reset().await.unwrap();
    sensor.set_sample_rate(SampleRate::Rate100).await.unwrap();
    sensor.set_pulse_width(PulseWidth::Pw411).await.unwrap();
    sensor.start_spo2_mode().await.unwrap();
    sensor.clear_fifo().await.unwrap();

    loop {
        if sensor.get_fifo_samples_available().await.unwrap() > 0 {
            let (red, ir) = sensor.read_fifo_sample().await.unwrap();
            // Process sample...
        }
        Timer::after_millis(10).await;
    }
}
```

---

## Hardware Reference & Wiring

| MAX30102 Pin | Name | Description | MCU Connection |
| :--- | :--- | :--- | :--- |
| 2 | **SCL** | I²C Clock Input | I²C SCL (with 4.7kΩ pull-up to VDD) |
| 3 | **SDA** | I²C Bidirectional Data | I²C SDA (with 4.7kΩ pull-up to VDD) |
| 4 | **PGND** | Power Ground (LED Driver) | Ground |
| 9, 10 | **VLED+** | LED Power Supply Anode | +3.3V to +5.0V (1µF bypass cap to PGND) |
| 11 | **VDD** | Analog Power Supply Input | +1.8V (1µF bypass cap to GND) |
| 12 | **GND** | Analog Ground | Ground |
| 13 | **INT** | Active-Low Open-Drain Interrupt | GPIO Input (4.7kΩ pull-up to VDD) |

* **Default Slave I²C Address**: `0x57` (7-bit address `0b1010111`)

---

## Datasheet Configuration Matrix

### Pulse Width vs. ADC Resolution & Max Sample Rate

| Pulse Width (`PulseWidth`) | Integration Time | ADC Resolution | Max Sample Rate (`SpO2` Mode) |
| :--- | :--- | :--- | :--- |
| `Pw69` | 69 µs | 15-bit | 3200 sps |
| `Pw118` | 118 µs | 16-bit | 1600 sps |
| `Pw215` | 215 µs | 17-bit | 800 sps |
| `Pw411` | 411 µs | 18-bit | 400 sps |

---

## License

Dual-licensed under either:

* **Apache License, Version 2.0** ([`LICENSE-APACHE`](LICENSE-APACHE) or [http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0))
* **MIT License** ([`LICENSE-MIT`](LICENSE-MIT) or [http://opensource.org/licenses/MIT](http://opensource.org/licenses/MIT))

at your option.
