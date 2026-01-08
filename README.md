# MAX30102 Rust Driver

A `no_std` Rust driver for the MAX30102 pulse oximeter and heart rate sensor with support for both blocking and async I2C operations.

## Features

- **Blocking and Async I2C**: Full support for both synchronous and asynchronous operations via `embedded-hal` and `embedded-hal-async`
- **Multiple Operating Modes**: Heart Rate Only, SpO2, and Multi-LED modes
- **FIFO Management**: Read samples from the 32-sample FIFO buffer with configurable averaging
- **Flexible Configuration**: Configurable sample rates (50-3200 Hz), LED pulse widths, ADC ranges, and LED pulse amplitudes
- **Temperature Measurement**: On-chip die temperature sensor
- **Interrupt Support**: Configure and handle FIFO, temperature, and overflow interrupts
- **Register-Level Access**: Generated from YAML manifest using `device-driver` for type-safe register access
- **Zero Cost Abstractions**: Minimal overhead with strict Rust linting

## What is the MAX30102?

The MAX30102 is an integrated pulse oximetry and heart rate monitor sensor solution. It combines:
- Red LED (660nm) for heart rate detection
- IR LED (880nm) for SpO2 measurement
- Photodetector with ambient light cancellation
- 18-bit ADC with programmable sample rate and pulse width
- 32-sample FIFO buffer
- On-chip temperature sensor
- I2C digital interface

## Installation

Add this to your `Cargo.toml`:

```toml
[dependencies]
max30102 = "0.1.0"
```

For async support:

```toml
[dependencies]
max30102 = { version = "0.1.0", features = ["async"] }
```

For Embassy framework support:

```toml
[dependencies]
max30102 = { version = "0.1.0", features = ["embassy"] }
```

## Usage

### Basic Example (Blocking)

```rust
use max30102::{Max30102, SampleRate, PulseWidth, AdcRange};
use embedded_hal::i2c::I2c;

fn main() -> Result<(), Error> {
    // Initialize your I2C peripheral
    let i2c = /* your I2C instance */;

    // Create the driver
    let mut sensor = Max30102::new(i2c);

    // Verify the device
    sensor.verify_part_id()?;

    // Reset and configure
    sensor.reset()?;
    sensor.set_sample_rate(SampleRate::Rate100)?;
    sensor.set_pulse_width(PulseWidth::Pw411)?;
    sensor.set_adc_range(AdcRange::Range4096)?;
    sensor.set_led_pulse_amplitude(0x1F, 0x1F)?;
    sensor.set_fifo_rollover(true)?;
    sensor.clear_fifo()?;

    // Start SpO2 mode (Red + IR LEDs)
    sensor.start_spo2_mode()?;

    // Read samples
    loop {
        if sensor.get_fifo_samples_available()? > 0 {
            let (red, ir) = sensor.read_fifo_sample()?;
            // Process red and ir values...
        }
    }
}
```

### Reading Temperature

```rust
// Start temperature conversion
sensor.start_temperature_conversion()?;

// Wait for conversion to complete
while !sensor.is_temperature_ready()? {
    // Wait or do other work
}

// Read temperature in Celsius
let temp = sensor.read_temperature()?;
```

### Using Interrupts

```rust
// Configure FIFO almost full interrupt (4 samples remaining)
sensor.set_fifo_almost_full(4)?;
sensor.enable_fifo_almost_full_interrupt()?;

// In your interrupt handler:
let status = sensor.read_interrupt_status_1()?;
if (status & 0x80) != 0 {
    // FIFO almost full
    let mut buffer = [0u8; 24]; // 4 samples * 6 bytes
    sensor.read_fifo(&mut buffer)?;
}
```

### Async Example (Embassy)

```rust
use max30102::Max30102;
use embassy_time::Timer;

#[embassy_executor::task]
async fn sensor_task(i2c: I2cDevice) {
    let mut sensor = Max30102::new(i2c);

    sensor.verify_part_id_async().await.unwrap();
    sensor.reset_async().await.unwrap();
    sensor.start_spo2_mode().unwrap();

    loop {
        if sensor.get_fifo_samples_available().unwrap() > 0 {
            let mut buffer = [0u8; 6];
            sensor.read_fifo_async(&mut buffer).await.unwrap();
            // Process samples...
        }
        Timer::after_millis(10).await;
    }
}
```

## Operating Modes

### Heart Rate Mode
Uses only the red LED to detect heart rate.

```rust
sensor.start_heart_rate_mode()?;
```

### SpO2 Mode
Uses both red and IR LEDs for SpO2 and heart rate measurement (most common mode).

```rust
sensor.start_spo2_mode()?;
```

### Multi-LED Mode
Advanced mode allowing custom LED time-slotting patterns.

```rust
sensor.start_multi_led_mode()?;
```

## Configuration Options

### Sample Rate
Available rates: 50, 100, 200, 400, 800, 1000, 1600, 3200 samples per second

```rust
sensor.set_sample_rate(SampleRate::Rate100)?;
```

### LED Pulse Width
- `Pw69`: 69 μs (15-bit ADC resolution)
- `Pw118`: 118 μs (16-bit ADC resolution)
- `Pw215`: 215 μs (17-bit ADC resolution)
- `Pw411`: 411 μs (18-bit ADC resolution)

```rust
sensor.set_pulse_width(PulseWidth::Pw411)?;
```

### ADC Range
- `Range2048`: 2048 nA full scale
- `Range4096`: 4096 nA full scale (typical)
- `Range8192`: 8192 nA full scale
- `Range16384`: 16384 nA full scale

```rust
sensor.set_adc_range(AdcRange::Range4096)?;
```

### LED Pulse Amplitude
Values from 0x00 to 0xFF control LED brightness (0x1F is typical).

```rust
sensor.set_led_pulse_amplitude(0x1F, 0x1F)?; // Red, IR
```

### Sample Averaging
Average multiple samples to reduce noise: 1, 2, 4, 8, 16, or 32 samples.

```rust
sensor.set_sample_averaging(SampleAveraging::Avg4)?;
```

## FIFO Management

The MAX30102 has a 32-sample FIFO buffer that can be configured to:
- Stop when full, or
- Roll over (circular buffer mode)

```rust
// Enable rollover mode
sensor.set_fifo_rollover(true)?;

// Clear FIFO
sensor.clear_fifo()?;

// Check available samples
let count = sensor.get_fifo_samples_available()?;

// Read samples
let (red, ir) = sensor.read_fifo_sample()?;
```

## Power Management

```rust
// Enter low-power shutdown mode
sensor.shutdown()?;

// Wake up from shutdown
sensor.wakeup()?;

// Software reset
sensor.reset()?;
```

## Features

- `async`: Enable async I2C support via `embedded-hal-async`
- `defmt-03`: Enable defmt logging support
- `embassy`: Enable both async and defmt support for Embassy framework

## Hardware Connections

The MAX30102 uses I2C communication:

| MAX30102 Pin | Function | MCU Connection |
|--------------|----------|----------------|
| VCC | Power Supply | 1.8V or 3.3V |
| GND | Ground | GND |
| SDA | I2C Data | I2C SDA |
| SCL | I2C Clock | I2C SCL |
| INT | Interrupt (optional) | GPIO with interrupt |

Default I2C address: `0x57`

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT License ([LICENSE-MIT](LICENSE-MIT))

at your option.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Resources

- [MAX30102 Datasheet](https://datasheets.maximintegrated.com/en/ds/MAX30102.pdf)
- [Application Note 6409: Digital Filter for Pulse Oximetry](https://www.maximintegrated.com/en/design/technical-documents/app-notes/6/6409.html)
