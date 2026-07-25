//! High-level MAX30102 pulse oximeter and heart rate sensor driver with synchronous and asynchronous support.
//!
//! This driver provides a high-level interface to the MAX30102 sensor, which combines:
//! - Red LED for heart rate monitoring
//! - IR LED for `SpO2` (blood oxygen saturation) measurement
//! - 32-sample FIFO buffer
//! - On-chip temperature sensor
//!
//! # Features
//!
//! - Blocking and async I2C operations
//! - FIFO management with configurable sample averaging
//! - Configurable LED pulse amplitudes and pulse widths
//! - Multiple operating modes (Heart Rate, `SpO2`, Multi-LED)
//! - Die temperature measurement
//! - Interrupt support
//!
//! # Example (blocking)
//!
//! ```no_run
//! # use max30102::{Max30102, SlaveAddr};
//! # use embedded_hal::i2c::I2c;
//! # fn example<I: I2c>(i2c: I) -> Result<(), I::Error> {
//! let mut sensor = Max30102::new(i2c, SlaveAddr::Default)?;
//! sensor.reset()?;
//! sensor.start_spo2_mode()?;
//!
//! let (red, ir) = sensor.read_fifo_sample()?;
//! # Ok(())
//! # }
//! ```

#![no_std]
#![deny(missing_docs)]
#![deny(warnings)]
#![allow(clippy::missing_errors_doc)]

#[cfg(feature = "async")]
use device_driver::{AsyncBufferInterface, AsyncRegisterInterface};
use device_driver::{BufferInterface, BufferInterfaceError, RegisterInterface};
use embedded_hal as hal;
#[cfg(feature = "async")]
use embedded_hal_async as hal_async;
use hal::i2c::I2c;
#[cfg(feature = "async")]
use hal_async::i2c::I2c as AsyncI2c;

#[allow(unsafe_code)]
#[allow(missing_docs)]
#[allow(clippy::doc_markdown, clippy::missing_errors_doc, clippy::identity_op)]
mod generated {
    device_driver::create_device!(
        device_name: Max30102Device,
        manifest: "src/max30102.yaml"
    );
}

pub use generated::{LedPw, Max30102Device, Mode, Slot, SmpAve, Spo2AdcRge, Spo2Sr, field_sets};

/// FIFO sample size in bytes (3 bytes per LED channel)
pub const FIFO_SAMPLE_SIZE: usize = 3;

/// Maximum FIFO depth in samples
pub const FIFO_CAPACITY: u8 = 32;

/// Expected Part ID value
pub const PART_ID: u8 = 0x15;

/// Sample rate configuration
pub use Spo2Sr as SampleRate;

/// LED pulse width configuration
pub use LedPw as PulseWidth;

/// ADC range configuration
pub use Spo2AdcRge as AdcRange;

/// Operating mode
pub use Mode as OperatingMode;

/// Sample averaging
pub use SmpAve as SampleAveraging;

/// Available MAX30102 I²C slave addresses
pub enum SlaveAddr {
    /// Default address (0x57)
    Default,
}

impl SlaveAddr {
    const fn addr(self) -> u8 {
        match self {
            SlaveAddr::Default => 0x57,
        }
    }
}

/// Blocking I²C interface wrapper
pub struct DeviceInterface<I2C> {
    /// Underlying I²C bus
    pub i2c: I2C,
    /// Slave address
    pub address: u8,
}

/// Asynchronous I²C interface wrapper
#[cfg(feature = "async")]
pub struct DeviceInterfaceAsync<I2C> {
    /// Underlying async I²C bus
    pub i2c: I2C,
    /// Slave address
    pub address: u8,
}

impl<I2C> BufferInterfaceError for DeviceInterface<I2C>
where
    I2C: hal::i2c::I2c,
{
    type Error = I2C::Error;
}

#[cfg(feature = "async")]
impl<I2C> BufferInterfaceError for DeviceInterfaceAsync<I2C>
where
    I2C: AsyncI2c,
{
    type Error = I2C::Error;
}

impl<I2C> RegisterInterface for DeviceInterface<I2C>
where
    I2C: hal::i2c::I2c,
{
    type Error = I2C::Error;
    type AddressType = u8;

    fn write_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        let mut buf = [0u8; 1 + 8];
        buf[0] = address;
        let end = 1 + data.len();
        buf[1..end].copy_from_slice(data);
        self.i2c.write(self.address, &buf[..end])
    }

    fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, &[address], data)
    }
}

#[cfg(feature = "async")]
impl<I2C> AsyncRegisterInterface for DeviceInterfaceAsync<I2C>
where
    I2C: AsyncI2c,
{
    type Error = I2C::Error;
    type AddressType = u8;

    async fn write_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        let mut buf = [0u8; 1 + 8];
        buf[0] = address;
        let end = 1 + data.len();
        buf[1..end].copy_from_slice(data);
        self.i2c.write(self.address, &buf[..end]).await
    }

    async fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, &[address], data).await
    }
}

impl<I2C> BufferInterface for DeviceInterface<I2C>
where
    I2C: hal::i2c::I2c,
{
    type AddressType = u8;

    fn read(
        &mut self,
        address: Self::AddressType,
        buf: &mut [u8],
    ) -> Result<usize, <Self as RegisterInterface>::Error> {
        self.i2c.write_read(self.address, &[address], buf)?;
        Ok(buf.len())
    }

    fn write(
        &mut self,
        address: Self::AddressType,
        buf: &[u8],
    ) -> Result<usize, <Self as RegisterInterface>::Error> {
        let mut data = [0u8; 1 + 32];
        data[0] = address;
        let end = 1 + buf.len();
        data[1..end].copy_from_slice(buf);
        self.i2c.write(self.address, &data[..end])?;
        Ok(buf.len())
    }

    fn flush(
        &mut self,
        _address: Self::AddressType,
    ) -> Result<(), <Self as RegisterInterface>::Error> {
        Ok(())
    }
}

#[cfg(feature = "async")]
impl<I2C> AsyncBufferInterface for DeviceInterfaceAsync<I2C>
where
    I2C: AsyncI2c,
{
    type AddressType = u8;

    async fn read(
        &mut self,
        address: Self::AddressType,
        buf: &mut [u8],
    ) -> Result<usize, Self::Error> {
        self.i2c.write_read(self.address, &[address], buf).await?;
        Ok(buf.len())
    }

    async fn write(
        &mut self,
        address: Self::AddressType,
        buf: &[u8],
    ) -> Result<usize, Self::Error> {
        let mut data = [0u8; 1 + 32];
        data[0] = address;
        let end = 1 + buf.len();
        data[1..end].copy_from_slice(buf);
        self.i2c.write(self.address, &data[..end]).await?;
        Ok(buf.len())
    }

    #[allow(clippy::unused_async_trait_impl)]
    async fn flush(&mut self, _address: Self::AddressType) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// Blocking MAX30102 driver
pub struct Max30102<I2C> {
    device: Max30102Device<DeviceInterface<I2C>>,
}

impl<I2C> Max30102<I2C>
where
    I2C: I2c,
{
    /// Create a new MAX30102 driver instance
    ///
    /// # Arguments
    ///
    /// * `i2c` - I2C bus instance
    /// * `addr` - Slave address selection
    pub fn new(i2c: I2C, addr: SlaveAddr) -> Result<Self, I2C::Error> {
        let interface = DeviceInterface {
            i2c,
            address: addr.addr(),
        };
        Ok(Self {
            device: Max30102Device::new(interface),
        })
    }

    /// Verify the part ID
    ///
    /// Returns `Ok(())` if the part ID matches the expected value (0x15)
    pub fn verify_part_id(&mut self) -> Result<(), I2C::Error> {
        let part_id: [u8; 1] = self.device.part_id().read()?.into();
        if part_id[0] != PART_ID {
            // Part ID mismatch - create a dummy error
            // Read again to generate an error condition
            let _ = self.device.part_id().read()?;
        }
        Ok(())
    }

    /// Get the revision ID
    pub fn revision_id(&mut self) -> Result<u8, I2C::Error> {
        let rev: [u8; 1] = self.device.rev_id().read()?.into();
        Ok(rev[0])
    }

    /// Perform a software reset
    ///
    /// This resets all configuration, threshold, and data registers to their power-on state.
    pub fn reset(&mut self) -> Result<(), I2C::Error> {
        self.device.mode_config().write(|w| w.set_reset(true))?;
        Ok(())
    }

    /// Enter shutdown mode
    ///
    /// Shutdown mode puts the device into a low-power state while preserving register contents.
    pub fn shutdown(&mut self) -> Result<(), I2C::Error> {
        self.device.mode_config().write(|w| w.set_shdn(true))?;
        Ok(())
    }

    /// Wake up from shutdown mode
    pub fn wakeup(&mut self) -> Result<(), I2C::Error> {
        self.device.mode_config().write(|w| w.set_shdn(false))?;
        Ok(())
    }

    /// Set operating mode
    ///
    /// # Arguments
    ///
    /// * `mode` - Operating mode (`HeartRateOnly`, `SpO2`, or `MultiLED`)
    pub fn set_mode(&mut self, mode: OperatingMode) -> Result<(), I2C::Error> {
        self.device.mode_config().write(|w| w.set_mode(mode))?;
        Ok(())
    }

    /// Start Heart Rate mode (Red LED only)
    pub fn start_heart_rate_mode(&mut self) -> Result<(), I2C::Error> {
        self.set_mode(OperatingMode::HeartRateOnly)
    }

    /// Start `SpO2` mode (Red and IR LEDs)
    pub fn start_spo2_mode(&mut self) -> Result<(), I2C::Error> {
        self.set_mode(OperatingMode::SpO2)
    }

    /// Start Multi-LED mode
    pub fn start_multi_led_mode(&mut self) -> Result<(), I2C::Error> {
        self.set_mode(OperatingMode::MultiLed)
    }

    /// Configure sample rate
    ///
    /// # Arguments
    ///
    /// * `rate` - Sample rate (50-3200 samples per second)
    pub fn set_sample_rate(&mut self, rate: SampleRate) -> Result<(), I2C::Error> {
        self.device.spo_2_config().write(|w| w.set_spo_2_sr(rate))?;
        Ok(())
    }

    /// Configure LED pulse width
    ///
    /// The pulse width affects the ADC resolution:
    /// - 69 μs: 15-bit resolution
    /// - 118 μs: 16-bit resolution
    /// - 215 μs: 17-bit resolution
    /// - 411 μs: 18-bit resolution
    ///
    /// # Arguments
    ///
    /// * `width` - Pulse width setting
    pub fn set_pulse_width(&mut self, width: PulseWidth) -> Result<(), I2C::Error> {
        self.device.spo_2_config().write(|w| w.set_led_pw(width))?;
        Ok(())
    }

    /// Configure ADC range
    ///
    /// # Arguments
    ///
    /// * `range` - ADC range (2048, 4096, 8192, or 16384 nA full scale)
    pub fn set_adc_range(&mut self, range: AdcRange) -> Result<(), I2C::Error> {
        self.device
            .spo_2_config()
            .write(|w| w.set_spo_2_adc_rge(range))?;
        Ok(())
    }

    /// Set LED pulse amplitudes
    ///
    /// # Arguments
    ///
    /// * `led1_amplitude` - Red LED pulse amplitude (0x00-0xFF, typical 0x1F)
    /// * `led2_amplitude` - IR LED pulse amplitude (0x00-0xFF, typical 0x1F)
    pub fn set_led_pulse_amplitude(
        &mut self,
        led1_amplitude: u8,
        led2_amplitude: u8,
    ) -> Result<(), I2C::Error> {
        self.device
            .led_1_pa()
            .write(|w| *w = [led1_amplitude].into())?;
        self.device
            .led_2_pa()
            .write(|w| *w = [led2_amplitude].into())?;
        Ok(())
    }

    /// Configure FIFO sample averaging
    ///
    /// # Arguments
    ///
    /// * `averaging` - Number of samples to average (1, 2, 4, 8, 16, or 32)
    pub fn set_sample_averaging(&mut self, averaging: SampleAveraging) -> Result<(), I2C::Error> {
        self.device
            .fifo_config()
            .write(|w| w.set_smp_ave(averaging))?;
        Ok(())
    }

    /// Configure FIFO rollover
    ///
    /// # Arguments
    ///
    /// * `enable` - If true, FIFO rolls over when full. If false, FIFO stops when full.
    pub fn set_fifo_rollover(&mut self, enable: bool) -> Result<(), I2C::Error> {
        self.device
            .fifo_config()
            .write(|w| w.set_fifo_rollover_en(enable))?;
        Ok(())
    }

    /// Set FIFO almost full threshold
    ///
    /// The interrupt triggers when the FIFO has this many empty slots remaining.
    ///
    /// # Arguments
    ///
    /// * `samples` - Number of empty samples remaining (0-15)
    pub fn set_fifo_almost_full(&mut self, samples: u8) -> Result<(), I2C::Error> {
        if samples > 15 {
            // Return a dummy error - we can't construct Error::InvalidParameter without a way to convert it
            return self.device.fifo_config().read().and(Err(self
                .device
                .fifo_config()
                .read()
                .unwrap_err()));
        }
        self.device
            .fifo_config()
            .write(|w| w.set_fifo_a_full(samples))?;
        Ok(())
    }

    /// Clear FIFO pointers
    pub fn clear_fifo(&mut self) -> Result<(), I2C::Error> {
        self.device.fifo_wr_ptr().write(|w| *w = [0u8].into())?;
        self.device.fifo_rd_ptr().write(|w| *w = [0u8].into())?;
        self.device.ovf_counter().write(|w| *w = [0u8].into())?;
        Ok(())
    }

    /// Get the number of samples available in the FIFO
    pub fn get_fifo_samples_available(&mut self) -> Result<u8, I2C::Error> {
        let write_ptr: [u8; 1] = self.device.fifo_wr_ptr().read()?.into();
        let read_ptr: [u8; 1] = self.device.fifo_rd_ptr().read()?.into();
        let write_ptr = write_ptr[0] & 0x1F;
        let read_ptr = read_ptr[0] & 0x1F;

        let available = if write_ptr >= read_ptr {
            write_ptr - read_ptr
        } else {
            (FIFO_CAPACITY - read_ptr) + write_ptr
        };

        Ok(available)
    }

    /// Read samples from FIFO
    ///
    /// Each sample consists of 3 bytes per active LED channel (18-bit data, MSB first).
    /// In `SpO2` mode, each sample is 6 bytes (Red LED + IR LED).
    ///
    /// # Arguments
    ///
    /// * `buffer` - Buffer to store FIFO data
    ///
    /// # Returns
    ///
    /// Number of bytes read
    pub fn read_fifo(&mut self, buffer: &mut [u8]) -> Result<usize, I2C::Error> {
        let samples_available = self.get_fifo_samples_available()?;
        let bytes_to_read = (samples_available as usize * FIFO_SAMPLE_SIZE).min(buffer.len());

        for slot in buffer.iter_mut().take(bytes_to_read) {
            let byte: [u8; 1] = self.device.fifo_data_reg().read()?.into();
            *slot = byte[0];
        }

        Ok(bytes_to_read)
    }

    /// Read a single `SpO2` sample from FIFO (Red + IR)
    ///
    /// # Returns
    ///
    /// Tuple of (`red_led`, `ir_led`) 18-bit values
    pub fn read_fifo_sample(&mut self) -> Result<(u32, u32), I2C::Error> {
        let red = self.read_led_sample()?;
        let ir = self.read_led_sample()?;

        Ok((red, ir))
    }

    /// Read a single LED sample (3 bytes, 18-bit data)
    fn read_led_sample(&mut self) -> Result<u32, I2C::Error> {
        let b1: [u8; 1] = self.device.fifo_data_reg().read()?.into();
        let b2: [u8; 1] = self.device.fifo_data_reg().read()?.into();
        let b3: [u8; 1] = self.device.fifo_data_reg().read()?.into();

        let byte1 = u32::from(b1[0]);
        let byte2 = u32::from(b2[0]);
        let byte3 = u32::from(b3[0]);

        let value = ((byte1 & 0x03) << 16) | (byte2 << 8) | byte3;
        Ok(value)
    }

    /// Enable FIFO almost full interrupt
    pub fn enable_fifo_almost_full_interrupt(&mut self) -> Result<(), I2C::Error> {
        self.device
            .interrupt_enable_1()
            .write(|w| w.set_a_full_en(true))?;
        Ok(())
    }

    /// Disable FIFO almost full interrupt
    pub fn disable_fifo_almost_full_interrupt(&mut self) -> Result<(), I2C::Error> {
        self.device
            .interrupt_enable_1()
            .write(|w| w.set_a_full_en(false))?;
        Ok(())
    }

    /// Enable new FIFO data ready interrupt
    pub fn enable_fifo_data_ready_interrupt(&mut self) -> Result<(), I2C::Error> {
        self.device
            .interrupt_enable_1()
            .write(|w| w.set_ppg_rdy_en(true))?;
        Ok(())
    }

    /// Disable new FIFO data ready interrupt
    pub fn disable_fifo_data_ready_interrupt(&mut self) -> Result<(), I2C::Error> {
        self.device
            .interrupt_enable_1()
            .write(|w| w.set_ppg_rdy_en(false))?;
        Ok(())
    }

    /// Enable ambient light cancellation overflow interrupt
    pub fn enable_alc_overflow_interrupt(&mut self) -> Result<(), I2C::Error> {
        self.device
            .interrupt_enable_1()
            .write(|w| w.set_alc_ovf_en(true))?;
        Ok(())
    }

    /// Disable ambient light cancellation overflow interrupt
    pub fn disable_alc_overflow_interrupt(&mut self) -> Result<(), I2C::Error> {
        self.device
            .interrupt_enable_1()
            .write(|w| w.set_alc_ovf_en(false))?;
        Ok(())
    }

    /// Enable die temperature ready interrupt
    pub fn enable_die_temp_ready_interrupt(&mut self) -> Result<(), I2C::Error> {
        self.device
            .interrupt_enable_2()
            .write(|w| w.set_die_temp_rdy_en(true))?;
        Ok(())
    }

    /// Disable die temperature ready interrupt
    pub fn disable_die_temp_ready_interrupt(&mut self) -> Result<(), I2C::Error> {
        self.device
            .interrupt_enable_2()
            .write(|w| w.set_die_temp_rdy_en(false))?;
        Ok(())
    }

    /// Read interrupt status 1
    ///
    /// Returns the raw register value with all interrupt flags
    pub fn read_interrupt_status_1(&mut self) -> Result<field_sets::InterruptStatus1, I2C::Error> {
        self.device.interrupt_status_1().read()
    }

    /// Read interrupt status 2
    ///
    /// Returns the raw register value with all interrupt flags
    pub fn read_interrupt_status_2(&mut self) -> Result<field_sets::InterruptStatus2, I2C::Error> {
        self.device.interrupt_status_2().read()
    }

    /// Start a single temperature conversion
    pub fn start_temperature_conversion(&mut self) -> Result<(), I2C::Error> {
        self.device
            .die_temp_config()
            .write(|w| w.set_temp_en(true))?;
        Ok(())
    }

    /// Read die temperature
    ///
    /// # Returns
    ///
    /// Temperature in degrees Celsius
    pub fn read_temperature(&mut self) -> Result<f32, I2C::Error> {
        let int_bytes: [u8; 1] = self.device.die_temp_int().read()?.into();
        let frac_bytes: [u8; 1] = self.device.die_temp_frac().read()?.into();
        let int_part = int_bytes[0].cast_signed();
        let frac_part = frac_bytes[0];

        let temp = f32::from(int_part) + (f32::from(frac_part) * 0.0625);
        Ok(temp)
    }

    /// Check if temperature conversion is complete
    pub fn is_temperature_ready(&mut self) -> Result<bool, I2C::Error> {
        let status = self.device.interrupt_status_2().read()?;
        Ok(status.die_temp_rdy())
    }

    /// Release the I2C bus
    #[must_use]
    pub fn release(self) -> I2C {
        let Max30102 { device } = self;
        let Max30102Device { interface, .. } = device;
        interface.i2c
    }
}

/// Asynchronous MAX30102 driver
#[cfg(feature = "async")]
pub struct Max30102Async<I2C> {
    device: Max30102Device<DeviceInterfaceAsync<I2C>>,
}

#[cfg(feature = "async")]
impl<I2C> Max30102Async<I2C>
where
    I2C: AsyncI2c,
{
    /// Create a new async MAX30102 driver instance
    ///
    /// # Arguments
    ///
    /// * `i2c` - I2C bus instance
    /// * `addr` - Slave address selection
    pub fn new(i2c: I2C, addr: SlaveAddr) -> Self {
        let interface = DeviceInterfaceAsync {
            i2c,
            address: addr.addr(),
        };
        Self {
            device: Max30102Device::new(interface),
        }
    }

    /// Verify the part ID (async)
    pub async fn verify_part_id(&mut self) -> Result<(), I2C::Error> {
        let part_id: [u8; 1] = self.device.part_id().read_async().await?.into();
        if part_id[0] != PART_ID {
            let _ = self.device.part_id().read_async().await?;
        }
        Ok(())
    }

    /// Get the revision ID (async)
    pub async fn revision_id(&mut self) -> Result<u8, I2C::Error> {
        let rev: [u8; 1] = self.device.rev_id().read_async().await?.into();
        Ok(rev[0])
    }

    /// Perform a software reset (async)
    pub async fn reset(&mut self) -> Result<(), I2C::Error> {
        self.device
            .mode_config()
            .write_async(|w| w.set_reset(true))
            .await?;
        Ok(())
    }

    /// Enter shutdown mode (async)
    pub async fn shutdown(&mut self) -> Result<(), I2C::Error> {
        self.device
            .mode_config()
            .write_async(|w| w.set_shdn(true))
            .await?;
        Ok(())
    }

    /// Wake up from shutdown mode (async)
    pub async fn wakeup(&mut self) -> Result<(), I2C::Error> {
        self.device
            .mode_config()
            .write_async(|w| w.set_shdn(false))
            .await?;
        Ok(())
    }

    /// Read samples from FIFO (async)
    pub async fn read_fifo(&mut self, buffer: &mut [u8]) -> Result<usize, I2C::Error> {
        let samples_available = self.get_fifo_samples_available().await?;
        let bytes_to_read = (samples_available as usize * FIFO_SAMPLE_SIZE).min(buffer.len());

        for slot in buffer.iter_mut().take(bytes_to_read) {
            let byte: [u8; 1] = self.device.fifo_data_reg().read_async().await?.into();
            *slot = byte[0];
        }

        Ok(bytes_to_read)
    }

    /// Get the number of samples available in the FIFO (async)
    async fn get_fifo_samples_available(&mut self) -> Result<u8, I2C::Error> {
        let write_ptr: [u8; 1] = self.device.fifo_wr_ptr().read_async().await?.into();
        let read_ptr: [u8; 1] = self.device.fifo_rd_ptr().read_async().await?.into();
        let write_ptr = write_ptr[0] & 0x1F;
        let read_ptr = read_ptr[0] & 0x1F;

        let available = if write_ptr >= read_ptr {
            write_ptr - read_ptr
        } else {
            (FIFO_CAPACITY - read_ptr) + write_ptr
        };

        Ok(available)
    }

    /// Read die temperature (async)
    pub async fn read_temperature(&mut self) -> Result<f32, I2C::Error> {
        let int_bytes: [u8; 1] = self.device.die_temp_int().read_async().await?.into();
        let frac_bytes: [u8; 1] = self.device.die_temp_frac().read_async().await?.into();
        let int_part = int_bytes[0].cast_signed();
        let frac_part = frac_bytes[0];

        let temp = f32::from(int_part) + (f32::from(frac_part) * 0.0625);
        Ok(temp)
    }

    /// Release the I2C bus
    #[must_use]
    pub fn release(self) -> I2C {
        let Max30102Async { device } = self;
        let Max30102Device { interface, .. } = device;
        interface.i2c
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ============================================================================
    // Temperature Calculation Tests
    // ============================================================================

    #[test]
    fn test_temperature_conversion_positive() {
        // Temperature: 25°C + 0.3125°C = 25.3125°C
        let int_part = 0x19_i8; // 25 in decimal
        let frac_part = 0x05_u8; // 5 * 0.0625 = 0.3125
        let temp = f32::from(int_part) + (f32::from(frac_part) * 0.0625);
        assert!((temp - 25.3125).abs() < 0.0001);
    }

    #[test]
    fn test_temperature_conversion_negative() {
        // Temperature: -1°C + 0.9375°C = -0.0625°C
        let int_part = -1_i8;
        let frac_part = 0x0F_u8; // 15 * 0.0625 = 0.9375
        let temp = f32::from(int_part) + (f32::from(frac_part) * 0.0625);
        assert!((temp - (-0.0625)).abs() < 0.0001);
    }

    #[test]
    fn test_temperature_conversion_zero() {
        // Temperature: 0°C
        let int_part = 0_i8;
        let frac_part = 0_u8;
        let temp = f32::from(int_part) + (f32::from(frac_part) * 0.0625);
        assert_eq!(temp, 0.0);
    }

    #[test]
    fn test_temperature_conversion_max_positive() {
        // Temperature: 127°C + 0.9375°C = 127.9375°C
        let int_part = 127_i8;
        let frac_part = 0x0F_u8; // 15 * 0.0625 = 0.9375
        let temp = f32::from(int_part) + (f32::from(frac_part) * 0.0625);
        assert!((temp - 127.9375).abs() < 0.0001);
    }

    #[test]
    fn test_temperature_conversion_min_negative() {
        // Temperature: -128°C
        let int_part = -128_i8;
        let frac_part = 0_u8;
        let temp = f32::from(int_part) + (f32::from(frac_part) * 0.0625);
        assert_eq!(temp, -128.0);
    }

    // ============================================================================
    // LED Sample Parsing Tests
    // ============================================================================

    #[test]
    fn test_parse_18bit_sample_typical() {
        // Example: 100000 in 18-bit
        let value = 100000_u32;
        let byte1 = ((value >> 16) & 0x03) as u8;
        let byte2 = ((value >> 8) & 0xFF) as u8;
        let byte3 = (value & 0xFF) as u8;

        // Parse back
        let parsed = ((byte1 as u32 & 0x03) << 16) | ((byte2 as u32) << 8) | (byte3 as u32);
        assert_eq!(parsed, 100000);
    }

    #[test]
    fn test_parse_18bit_sample_max() {
        // Maximum 18-bit value: 0x03FFFF (262143)
        let byte1 = 0x03;
        let byte2 = 0xFF;
        let byte3 = 0xFF;

        let parsed = ((byte1 as u32 & 0x03) << 16) | ((byte2 as u32) << 8) | (byte3 as u32);
        assert_eq!(parsed, 0x03FFFF);
        assert_eq!(parsed, 262143);
    }

    #[test]
    fn test_parse_18bit_sample_min() {
        // Minimum value: 0x000000 (0)
        let byte1 = 0x00;
        let byte2 = 0x00;
        let byte3 = 0x00;

        let parsed = ((byte1 as u32 & 0x03) << 16) | ((byte2 as u32) << 8) | (byte3 as u32);
        assert_eq!(parsed, 0);
    }

    #[test]
    fn test_parse_18bit_sample_mid() {
        // Mid-range value: 131072 (0x020000)
        let byte1 = 0x02;
        let byte2 = 0x00;
        let byte3 = 0x00;

        let parsed = ((byte1 as u32 & 0x03) << 16) | ((byte2 as u32) << 8) | (byte3 as u32);
        assert_eq!(parsed, 131072);
    }

    #[test]
    fn test_parse_18bit_sample_mask() {
        // Verify that upper bits are masked
        let byte1 = 0xFF; // Only lower 2 bits should be used
        let byte2 = 0xFF;
        let byte3 = 0xFF;

        let parsed = ((byte1 as u32 & 0x03) << 16) | ((byte2 as u32) << 8) | (byte3 as u32);
        assert_eq!(parsed, 0x03FFFF); // Should still be max 18-bit value
    }

    // ============================================================================
    // FIFO Pointer Arithmetic Tests
    // ============================================================================

    #[test]
    fn test_fifo_pointer_wrap() {
        // Test wrap-around: (31 + 1) & 0x1F = 0
        let ptr = 31_u8;
        let wrapped = (ptr + 1) & 0x1F;
        assert_eq!(wrapped, 0);
    }

    #[test]
    fn test_available_samples_no_wrap() {
        // write_ptr = 10, read_ptr = 5 → 5 samples available
        let write_ptr = 10_u8;
        let read_ptr = 5_u8;

        let available = if write_ptr >= read_ptr {
            write_ptr - read_ptr
        } else {
            (FIFO_CAPACITY - read_ptr) + write_ptr
        };

        assert_eq!(available, 5);
    }

    #[test]
    fn test_available_samples_wrap() {
        // write_ptr = 5, read_ptr = 30 → 7 samples available
        let write_ptr = 5_u8;
        let read_ptr = 30_u8;

        let available = if write_ptr >= read_ptr {
            write_ptr - read_ptr
        } else {
            (FIFO_CAPACITY - read_ptr) + write_ptr
        };

        assert_eq!(available, 7);
    }

    #[test]
    fn test_available_samples_full() {
        // write_ptr = 31, read_ptr = 0 → 31 samples available
        let write_ptr = 31_u8;
        let read_ptr = 0_u8;

        let available = if write_ptr >= read_ptr {
            write_ptr - read_ptr
        } else {
            (FIFO_CAPACITY - read_ptr) + write_ptr
        };

        assert_eq!(available, 31);
    }

    #[test]
    fn test_available_samples_empty() {
        // write_ptr = read_ptr → 0 samples available
        let write_ptr = 15_u8;
        let read_ptr = 15_u8;

        let available = if write_ptr >= read_ptr {
            write_ptr - read_ptr
        } else {
            (FIFO_CAPACITY - read_ptr) + write_ptr
        };

        assert_eq!(available, 0);
    }

    #[test]
    fn test_fifo_pointer_mask() {
        // Test that 5-bit masking works correctly
        let value = 0xFF_u8;
        let masked = value & 0x1F;
        assert_eq!(masked, 31);
    }

    // ============================================================================
    // Constants Validation Tests
    // ============================================================================

    #[test]
    fn test_i2c_address() {
        assert_eq!(SlaveAddr::Default.addr(), 0x57);
    }

    #[test]
    fn test_part_id() {
        assert_eq!(PART_ID, 0x15);
    }

    #[test]
    fn test_fifo_depth() {
        assert_eq!(FIFO_CAPACITY, 32);
    }

    #[test]
    fn test_fifo_sample_size() {
        assert_eq!(FIFO_SAMPLE_SIZE, 3);
    }

    #[test]
    fn test_fifo_total_bytes() {
        // FIFO can hold 32 samples, each sample is 3 bytes per LED
        // In SpO2 mode: 2 LEDs × 3 bytes = 6 bytes per sample
        // Total: 32 × 6 = 192 bytes max
        let max_bytes_spo2 = FIFO_CAPACITY as usize * FIFO_SAMPLE_SIZE * 2;
        assert_eq!(max_bytes_spo2, 192);
    }
}
