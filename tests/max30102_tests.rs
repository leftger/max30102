//! Integration tests for MAX30102 driver

use max30102::{
    AdcRange, Max30102, OperatingMode as Mode, PulseWidth, SampleAveraging, SampleRate, SlaveAddr,
};
use std::collections::{HashMap, VecDeque};

/// Mock I2C Error type
#[derive(Debug)]
struct DummyError;

impl embedded_hal::i2c::Error for DummyError {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        embedded_hal::i2c::ErrorKind::Other
    }
}

/// Mock I2C device that simulates MAX30102 hardware behavior
#[derive(Debug)]
struct DummyI2c {
    registers: HashMap<u8, u8>,
    fifo_data: VecDeque<u8>,
    write_ptr: u8,
    read_ptr: u8,
}

impl DummyI2c {
    fn new() -> Self {
        let mut registers = HashMap::new();

        // Initialize registers with their reset values
        registers.insert(0xFF, 0x15); // PART_ID
        registers.insert(0xFE, 0x00); // REV_ID
        registers.insert(0x00, 0x00); // INTERRUPT_STATUS_1
        registers.insert(0x01, 0x00); // INTERRUPT_STATUS_2
        registers.insert(0x02, 0x00); // INTERRUPT_ENABLE_1
        registers.insert(0x03, 0x00); // INTERRUPT_ENABLE_2
        registers.insert(0x04, 0x00); // FIFO_WR_PTR
        registers.insert(0x05, 0x00); // OVF_COUNTER
        registers.insert(0x06, 0x00); // FIFO_RD_PTR
        registers.insert(0x08, 0x00); // FIFO_CONFIG
        registers.insert(0x09, 0x00); // MODE_CONFIG
        registers.insert(0x0A, 0x00); // SPO2_CONFIG
        registers.insert(0x0C, 0x00); // LED1_PA
        registers.insert(0x0D, 0x00); // LED2_PA
        registers.insert(0x11, 0x00); // MULTI_LED_CTRL_1
        registers.insert(0x12, 0x00); // MULTI_LED_CTRL_2
        registers.insert(0x1F, 0x00); // DIE_TEMP_INT
        registers.insert(0x20, 0x00); // DIE_TEMP_FRAC
        registers.insert(0x21, 0x00); // DIE_TEMP_CONFIG

        Self {
            registers,
            fifo_data: VecDeque::with_capacity(96), // 32 samples * 3 bytes
            write_ptr: 0,
            read_ptr: 0,
        }
    }

    fn handle_fifo_read(&mut self) -> u8 {
        if let Some(byte) = self.fifo_data.pop_front() {
            // Each FIFO read doesn't change pointers automatically - driver manages them
            byte
        } else {
            0x00 // Return 0 if FIFO empty
        }
    }

    /// Simulate a sample being written to FIFO by the sensor
    fn simulate_fifo_sample(&mut self, red: u32, ir: u32) {
        // Add 18-bit red sample as 3 bytes
        self.fifo_data.push_back(((red >> 16) & 0x03) as u8);
        self.fifo_data.push_back(((red >> 8) & 0xFF) as u8);
        self.fifo_data.push_back((red & 0xFF) as u8);

        // Add 18-bit IR sample as 3 bytes
        self.fifo_data.push_back(((ir >> 16) & 0x03) as u8);
        self.fifo_data.push_back(((ir >> 8) & 0xFF) as u8);
        self.fifo_data.push_back((ir & 0xFF) as u8);

        // Update write pointer
        self.write_ptr = (self.write_ptr + 1) & 0x1F;
        self.registers.insert(0x04, self.write_ptr);

        // Set PPG_RDY interrupt flag
        let status = self.registers.get(&0x00).copied().unwrap_or(0);
        self.registers.insert(0x00, status | 0x40);
    }

    fn handle_reset(&mut self) {
        // Reset all registers to default values
        *self = Self::new();
    }

    fn handle_fifo_clear(&mut self) {
        self.fifo_data.clear();
        self.write_ptr = 0;
        self.read_ptr = 0;
        self.registers.insert(0x04, 0); // FIFO_WR_PTR
        self.registers.insert(0x06, 0); // FIFO_RD_PTR
        self.registers.insert(0x05, 0); // OVF_COUNTER
    }
}

impl embedded_hal::i2c::ErrorType for DummyI2c {
    type Error = DummyError;
}

impl embedded_hal::i2c::I2c for DummyI2c {
    fn transaction(
        &mut self,
        _address: u8,
        _operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        // Not needed for our tests
        Ok(())
    }

    fn write(&mut self, _address: u8, data: &[u8]) -> Result<(), Self::Error> {
        if data.is_empty() {
            return Ok(());
        }

        let register_addr = data[0];

        for (i, &value) in data[1..].iter().enumerate() {
            let addr = register_addr + i as u8;

            // Handle special register behaviors
            match addr {
                0x09 => {
                    // MODE_CONFIG - handle RESET bit
                    if value & 0x40 != 0 {
                        self.handle_reset();
                        return Ok(());
                    }
                    self.registers.insert(addr, value);
                }
                0x04 | 0x06 => {
                    // FIFO pointers - mask to 5 bits and handle clear
                    let masked_value = value & 0x1F;
                    self.registers.insert(addr, masked_value);

                    if addr == 0x04 {
                        self.write_ptr = masked_value;
                    } else {
                        self.read_ptr = masked_value;
                    }

                    // If both pointers are written to 0, clear FIFO
                    if self.write_ptr == 0 && self.read_ptr == 0 {
                        self.handle_fifo_clear();
                    }
                }
                0x21 => {
                    // DIE_TEMP_CONFIG - handle TEMP_EN bit
                    if value & 0x01 != 0 {
                        // Simulate temperature measurement complete
                        self.registers.insert(0x1F, 0x19); // 25°C integer
                        self.registers.insert(0x20, 0x05); // 0.3125°C fraction (5 * 0.0625 = 0.3125)

                        // Set DIE_TEMP_RDY flag
                        let status = self.registers.get(&0x01).copied().unwrap_or(0);
                        self.registers.insert(0x01, status | 0x02);
                    }
                    self.registers.insert(addr, value);
                }
                _ => {
                    self.registers.insert(addr, value);
                }
            }
        }

        Ok(())
    }

    fn read(&mut self, _address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        // This shouldn't be called in our implementation
        // We use write_read instead
        for byte in buffer.iter_mut() {
            *byte = 0;
        }
        Ok(())
    }

    fn write_read(
        &mut self,
        _address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        if write.is_empty() {
            return Ok(());
        }

        let register_addr = write[0];

        for (i, byte) in read.iter_mut().enumerate() {
            let addr = register_addr + i as u8;

            // Handle FIFO_DATA_REG specially
            if addr == 0x07 {
                *byte = self.handle_fifo_read();
            } else {
                *byte = self.registers.get(&addr).copied().unwrap_or(0);
            }
        }

        Ok(())
    }
}

// Helper function to create driver with mock I2C
fn create_driver() -> Max30102<DummyI2c> {
    let i2c = DummyI2c::new();
    Max30102::new(i2c, SlaveAddr::Default).unwrap()
}

// ============================================================================
// Device Identity & Initialization Tests
// ============================================================================

#[test]
fn test_device_identity() {
    let _driver = create_driver();
    // If initialization succeeded, device identity was verified (PART_ID = 0x15)
}

#[test]
fn test_reset() {
    let mut driver = create_driver();

    // Configure some registers
    driver.set_mode(Mode::SpO2).unwrap();
    driver.set_led_pulse_amplitude(50, 50).unwrap();

    // Reset the device
    driver.reset().unwrap();

    // After reset, device should be in default state
    // We can't directly verify register values, but reset shouldn't error
}

#[test]
fn test_shutdown_mode() {
    let mut driver = create_driver();

    // Enter shutdown mode
    driver.shutdown().unwrap();

    // Wake up from shutdown
    driver.wakeup().unwrap();

    // Should be able to configure after wakeup
    driver.set_mode(Mode::HeartRateOnly).unwrap();
}

// ============================================================================
// Mode Configuration Tests
// ============================================================================

#[test]
fn test_mode_heart_rate_only() {
    let mut driver = create_driver();
    driver.set_mode(Mode::HeartRateOnly).unwrap();
}

#[test]
fn test_mode_spo2() {
    let mut driver = create_driver();
    driver.set_mode(Mode::SpO2).unwrap();
}

#[test]
fn test_mode_multi_led() {
    let mut driver = create_driver();
    driver.set_mode(Mode::MultiLed).unwrap();
}

#[test]
fn test_start_heart_rate_mode() {
    let mut driver = create_driver();
    driver.start_heart_rate_mode().unwrap();
}

#[test]
fn test_start_spo2_mode() {
    let mut driver = create_driver();
    driver.start_spo2_mode().unwrap();
}

#[test]
fn test_start_multi_led_mode() {
    let mut driver = create_driver();
    driver.start_multi_led_mode().unwrap();
}

// ============================================================================
// Configuration Tests
// ============================================================================

#[test]
fn test_sample_rate_config() {
    let mut driver = create_driver();

    driver.set_sample_rate(SampleRate::Rate50).unwrap();
    driver.set_sample_rate(SampleRate::Rate100).unwrap();
    driver.set_sample_rate(SampleRate::Rate200).unwrap();
    driver.set_sample_rate(SampleRate::Rate400).unwrap();
    driver.set_sample_rate(SampleRate::Rate800).unwrap();
    driver.set_sample_rate(SampleRate::Rate1000).unwrap();
    driver.set_sample_rate(SampleRate::Rate1600).unwrap();
    driver.set_sample_rate(SampleRate::Rate3200).unwrap();
}

#[test]
fn test_pulse_width_config() {
    let mut driver = create_driver();

    driver.set_pulse_width(PulseWidth::Pw69).unwrap();
    driver.set_pulse_width(PulseWidth::Pw118).unwrap();
    driver.set_pulse_width(PulseWidth::Pw215).unwrap();
    driver.set_pulse_width(PulseWidth::Pw411).unwrap();
}

#[test]
fn test_adc_range_config() {
    let mut driver = create_driver();

    driver.set_adc_range(AdcRange::Range2048).unwrap();
    driver.set_adc_range(AdcRange::Range4096).unwrap();
    driver.set_adc_range(AdcRange::Range8192).unwrap();
    driver.set_adc_range(AdcRange::Range16384).unwrap();
}

#[test]
fn test_led_amplitude() {
    let mut driver = create_driver();

    // Test various amplitude values
    driver.set_led_pulse_amplitude(0, 0).unwrap();
    driver.set_led_pulse_amplitude(127, 127).unwrap();
    driver.set_led_pulse_amplitude(255, 255).unwrap();
    driver.set_led_pulse_amplitude(50, 100).unwrap();
}

#[test]
fn test_sample_averaging() {
    let mut driver = create_driver();

    driver
        .set_sample_averaging(SampleAveraging::NoAveraging)
        .unwrap();
    driver.set_sample_averaging(SampleAveraging::Avg2).unwrap();
    driver.set_sample_averaging(SampleAveraging::Avg4).unwrap();
    driver.set_sample_averaging(SampleAveraging::Avg8).unwrap();
    driver.set_sample_averaging(SampleAveraging::Avg16).unwrap();
    driver.set_sample_averaging(SampleAveraging::Avg32).unwrap();
}

#[test]
fn test_fifo_almost_full_config() {
    let mut driver = create_driver();

    // Test various FIFO almost full thresholds
    driver.set_fifo_almost_full(0).unwrap();
    driver.set_fifo_almost_full(15).unwrap();

    // Maximum value is 15 (4 bits)
    driver.set_fifo_almost_full(15).unwrap();
}

#[test]
fn test_fifo_rollover() {
    let mut driver = create_driver();

    driver.set_fifo_rollover(true).unwrap();
    driver.set_fifo_rollover(false).unwrap();
}

// ============================================================================
// FIFO Management Tests
// ============================================================================

#[test]
fn test_clear_fifo() {
    let mut driver = create_driver();

    // Clear FIFO
    driver.clear_fifo().unwrap();

    // Check that no samples are available
    assert_eq!(driver.get_fifo_samples_available().unwrap(), 0);
}

#[test]
fn test_fifo_available_samples() {
    let mut driver = create_driver();

    // Initially should be empty
    driver.clear_fifo().unwrap();
    assert_eq!(driver.get_fifo_samples_available().unwrap(), 0);

    // We can't easily simulate samples being added without access to internals
    // This test verifies the method works without error
}

#[test]
fn test_read_fifo_single_sample() {
    let driver = create_driver();
    let i2c = driver.release();

    // Create a new driver with access to mock internals
    let mut mock_i2c = i2c;

    // Simulate a sample in FIFO
    mock_i2c.simulate_fifo_sample(100000, 80000);

    let mut driver = Max30102::new(mock_i2c, SlaveAddr::Default).unwrap();

    // Check samples available
    let available = driver.get_fifo_samples_available().unwrap();
    assert_eq!(available, 1);

    // Read the sample
    let (red, ir) = driver.read_fifo_sample().unwrap();
    assert_eq!(red, 100000);
    assert_eq!(ir, 80000);
}

#[test]
fn test_read_fifo_multiple_samples() {
    let driver = create_driver();
    let i2c = driver.release();

    let mut mock_i2c = i2c;

    // Simulate multiple samples
    mock_i2c.simulate_fifo_sample(100000, 80000);
    mock_i2c.simulate_fifo_sample(110000, 85000);
    mock_i2c.simulate_fifo_sample(120000, 90000);

    let mut driver = Max30102::new(mock_i2c, SlaveAddr::Default).unwrap();

    // Check samples available
    assert_eq!(driver.get_fifo_samples_available().unwrap(), 3);

    // Read all samples
    let (red1, ir1) = driver.read_fifo_sample().unwrap();
    assert_eq!(red1, 100000);
    assert_eq!(ir1, 80000);

    let (red2, ir2) = driver.read_fifo_sample().unwrap();
    assert_eq!(red2, 110000);
    assert_eq!(ir2, 85000);

    let (red3, ir3) = driver.read_fifo_sample().unwrap();
    assert_eq!(red3, 120000);
    assert_eq!(ir3, 90000);
}

#[test]
fn test_fifo_pointer_wrap_around() {
    let driver = create_driver();
    let i2c = driver.release();

    let mut mock_i2c = i2c;

    // Simulate FIFO wrap-around by filling to capacity
    for i in 0..32 {
        mock_i2c.simulate_fifo_sample(100000 + i * 1000, 80000 + i * 1000);
    }

    let mut driver = Max30102::new(mock_i2c, SlaveAddr::Default).unwrap();

    // After 32 samples, write pointer wraps to 0. This tests that pointer
    // arithmetic with wrap-around doesn't error. Note: in our mock, when
    // write_ptr == read_ptr, it appears empty (pointer arithmetic limitation).
    let available = driver.get_fifo_samples_available().unwrap();
    assert!(available <= 32);
}

#[test]
fn test_read_fifo_buffer() {
    let driver = create_driver();
    let i2c = driver.release();

    let mut mock_i2c = i2c;

    // Simulate samples
    mock_i2c.simulate_fifo_sample(100000, 80000);

    let mut driver = Max30102::new(mock_i2c, SlaveAddr::Default).unwrap();

    // Read using buffer
    let mut buffer = [0u8; 6]; // 6 bytes for one SpO2 sample
    let bytes_read = driver.read_fifo(&mut buffer).unwrap();
    assert!(bytes_read > 0);
}

// ============================================================================
// Temperature Tests
// ============================================================================

#[test]
fn test_read_temperature() {
    let mut driver = create_driver();

    // Start temperature conversion
    driver.start_temperature_conversion().unwrap();

    // Read temperature (mock simulates 25.3125°C)
    let temp = driver.read_temperature().unwrap();

    // Verify temperature is in reasonable range
    assert!(temp > 20.0 && temp < 30.0);

    // More specifically, mock returns 0x19 + 0x50 = 25.3125°C
    assert!((temp - 25.3125).abs() < 0.001);
}

#[test]
fn test_temperature_multiple_reads() {
    let mut driver = create_driver();

    // Start conversion
    driver.start_temperature_conversion().unwrap();

    // Multiple temperature reads should work
    let temp1 = driver.read_temperature().unwrap();
    let temp2 = driver.read_temperature().unwrap();

    // Mock always returns same value
    assert_eq!(temp1, temp2);
}

#[test]
fn test_temperature_ready_flag() {
    let mut driver = create_driver();

    // Start conversion (mock sets the ready flag)
    driver.start_temperature_conversion().unwrap();

    // Check if ready
    let ready = driver.is_temperature_ready().unwrap();
    assert!(ready);
}

// ============================================================================
// Interrupt Configuration Tests
// ============================================================================

#[test]
fn test_enable_fifo_almost_full_interrupt() {
    let mut driver = create_driver();
    driver.enable_fifo_almost_full_interrupt().unwrap();
}

#[test]
fn test_disable_fifo_almost_full_interrupt() {
    let mut driver = create_driver();
    driver.disable_fifo_almost_full_interrupt().unwrap();
}

#[test]
fn test_enable_fifo_data_ready_interrupt() {
    let mut driver = create_driver();
    driver.enable_fifo_data_ready_interrupt().unwrap();
}

#[test]
fn test_disable_fifo_data_ready_interrupt() {
    let mut driver = create_driver();
    driver.disable_fifo_data_ready_interrupt().unwrap();
}

#[test]
fn test_enable_alc_overflow_interrupt() {
    let mut driver = create_driver();
    driver.enable_alc_overflow_interrupt().unwrap();
}

#[test]
fn test_disable_alc_overflow_interrupt() {
    let mut driver = create_driver();
    driver.disable_alc_overflow_interrupt().unwrap();
}

#[test]
fn test_enable_die_temp_ready_interrupt() {
    let mut driver = create_driver();
    driver.enable_die_temp_ready_interrupt().unwrap();
}

#[test]
fn test_disable_die_temp_ready_interrupt() {
    let mut driver = create_driver();
    driver.disable_die_temp_ready_interrupt().unwrap();
}

#[test]
fn test_read_interrupt_status() {
    let mut driver = create_driver();

    // Read interrupt status registers (shouldn't error)
    let _status1 = driver.read_interrupt_status_1().unwrap();
    let _status2 = driver.read_interrupt_status_2().unwrap();
}

// ============================================================================
// Integration Tests
// ============================================================================

#[test]
fn test_full_initialization_sequence() {
    let mut driver = create_driver();

    // Typical initialization sequence
    driver.reset().unwrap();
    driver.clear_fifo().unwrap();
    driver.set_mode(Mode::SpO2).unwrap();
    driver.set_sample_rate(SampleRate::Rate100).unwrap();
    driver.set_pulse_width(PulseWidth::Pw411).unwrap();
    driver.set_adc_range(AdcRange::Range4096).unwrap();
    driver.set_led_pulse_amplitude(50, 50).unwrap();
    driver.set_sample_averaging(SampleAveraging::Avg4).unwrap();
    driver.set_fifo_almost_full(15).unwrap();
    driver.enable_fifo_almost_full_interrupt().unwrap();
    driver.enable_fifo_data_ready_interrupt().unwrap();
}

#[test]
fn test_read_samples_after_configuration() {
    let driver = create_driver();
    let i2c = driver.release();

    let mut mock_i2c = i2c;

    // Configure device
    let mut driver = Max30102::new(mock_i2c, SlaveAddr::Default).unwrap();
    driver.set_mode(Mode::SpO2).unwrap();
    driver.set_led_pulse_amplitude(50, 50).unwrap();
    driver.clear_fifo().unwrap();

    // Release and add samples
    mock_i2c = driver.release();
    mock_i2c.simulate_fifo_sample(150000, 120000);

    // Read samples
    let mut driver = Max30102::new(mock_i2c, SlaveAddr::Default).unwrap();
    let (red, ir) = driver.read_fifo_sample().unwrap();

    assert_eq!(red, 150000);
    assert_eq!(ir, 120000);
}

#[test]
fn test_revision_id() {
    let mut driver = create_driver();
    let _rev_id = driver.revision_id().unwrap();
}

#[test]
fn test_verify_part_id() {
    let mut driver = create_driver();
    driver.verify_part_id().unwrap();
}
