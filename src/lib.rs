#![no_std]
//!
//! Rust Driver for device INA226 High-Side or Low-Side Measurement, Bi-Direcional Current and Power Monitor
//!
//! Datasheet:
//! [INA226](https://www.ti.com/product/INA226)
//!
//! This crate was made for and tested on INA226 from Texas Instruments, it is based on I2C from embedded-hal crate.
//! The implementation of this crate is based on #![no_std] but with some minor adjustments it can be used on std environments.
//! This crate is async compatible (must enable 'async' feature).
//!  
//! This driver allows you to:
//! - Manually configure INA226 address
//! - Automatically detect INA226 address
//! - Fully configure the configuration register
//! - Calibrate the device
//! - Manually configure the Mask/Enable register
//! - Manually configure the Alert register
//! - Read Voltage, shunt voltage, current and power
//! - Test device connection for errors
//! - Reset the device to factory specs
//!
//! This driver contains two modes, non operational and operational:
//! Non Operational:
//! - When creating the device it starts on this mode
//! - you can manualy set the address or auto detect it
//! - when initializing the device it will test the connection and then move to Operational mode.
//!
//! Operational:
//! - Set configuration, mask/enable and alert registers
//! - Set the calibration register based on Rshunt and MaxAmp (set_ina_calibration)
//! - Set the calibration register manually (set_ina_calibration_value)
//! - Read configured values
//! - Reset the device to factory specs
//! - Read voltage, shunt voltage, current and power
//!
//! Please note that after any configuration change, a commit operation must be performed so the information can be sent to the IC.
//!
//! If the device is not calibrated, it assumes a shunt resistor value of 2 mOhms and maximum current of 10 Amps
//!
//! Note on I2C usage: If you need to search for INA Address, then you must use the BUS version of I2C from Embedded hal
//! because you will need to pass an I2C instance for the NEW and another for the INITIALIZE Methods. If you don't need to
//! search for INA Address, then pass NONE to the NEW method and then the I2C instance for INITIALIZE method.
//!

// Imports
use byteorder::{BigEndian, ByteOrder};
use core::fmt::Display;
#[cfg(not(feature = "async"))]
use embedded_hal::i2c::I2c;
#[cfg(feature = "async")]
use embedded_hal_async::i2c::I2c;

#[cfg(not(feature = "no_float"))]
const INTERNAL_SCALING: f64 = 0.00512;
#[cfg(not(feature = "no_float"))]
const SHUNT_LSB: f64 = 0.0000025; // in Volts (2.5 uV)
#[cfg(feature = "no_float")]
const SHUNT_LSB_NV: u32 = 2500; // in Volts (2.5 uV)
#[cfg(not(feature = "no_float"))]
const VOLTAGE_LSB: f64 = 0.00125; // in Volts (1.25 mV)
#[cfg(feature = "no_float")]
const VOLTAGE_LSB_UV: u32 = 1250; // in Volts (1.25 mV)
#[cfg(not(feature = "no_float"))]
const CURRENT_LSB: f64 = 0.001; // in Amps (1 mA)
#[cfg(feature = "no_float")]
const CURRENT_LSB_UA: u32 = 1000; // in Amps (1 mA)
#[cfg(not(feature = "no_float"))]
const POWER_LSB: f64 = 0.025; // in Watts (25 mW)
#[cfg(feature = "no_float")]
const POWER_LSB_UW: u32 = 25000; // in Watts (25 mW)

/// Enum used for ina226 addressing based on pin connection
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum SlaveAddressing {
    Gnd,
    Vs,
    Sda,
    Scl,
}

impl Display for SlaveAddressing {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            SlaveAddressing::Gnd => write!(f, "GND"),
            SlaveAddressing::Vs => write!(f, "VS"),
            SlaveAddressing::Sda => write!(f, "SDA"),
            SlaveAddressing::Scl => write!(f, "SCL"),
        }
    }
}

/// Enum used to identify ina226 registers
enum Register {
    Configuration = 0x00,
    ShuntVoltage = 0x01,
    BusVoltage = 0x02,
    Power = 0x03,
    Current = 0x04,
    Calibration = 0x05,
    MaskEnable = 0x06,
    Alert = 0x07,
    Manufacturer = 0xFE,
    DieId = 0xFF,
}

/// Enum used to determine the number of averages to be used
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum InaAverage {
    _1 = 0x00,
    _4 = 0x01,
    _16 = 0x02,
    _64 = 0x03,
    _128 = 0x04,
    _256 = 0x05,
    _512 = 0x06,
    _1024 = 0x07,
}

/// Set the conversion time for the Bus voltage measurement
#[allow(non_camel_case_types)]
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum InaVbusct {
    _140_us = 0x00,
    _204_us = 0x01,
    _332_us = 0x02,
    _588_us = 0x03,
    _1_1_ms = 0x04,
    _2_116_ms = 0x05,
    _4_156_ms = 0x06,
    _8_244_ms = 0x07,
}

/// Set the conversion time for the Shunt Bus voltage measurement
#[allow(non_camel_case_types)]
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum InaVshct {
    _140_us = 0x00,
    _204_us = 0x01,
    _332_us = 0x02,
    _588_us = 0x03,
    _1_1_ms = 0x04,
    _2_116_ms = 0x05,
    _4_156_ms = 0x06,
    _8_244_ms = 0x07,
}

/// Used to set the ina226 mode of operation
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum InaMode {
    PowerDown = 0x00,
    ShuntVoltageTriggered = 0x01,
    BusVoltageTriggered = 0x02,
    ShuntAndBusTriggered = 0x03,
    PowerDown2 = 0x04,
    ShuntVoltageContinuous = 0x05,
    BusVoltageContinuous = 0x06,
    ShuntAndBusContinuous = 0x07,
}

/// Used to choose the function of alert pin
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum MaskEnable {
    ShuntOverVoltage = 1 << 15,
    ShuntUnderVoltage = 1 << 14,
    BusOverVoltage = 1 << 13,
    BusUnderVoltage = 1 << 12,
    PowerOverLimit = 1 << 11,
    ConversionReady = 1 << 10,
    AlertFunctionFlag = 1 << 4,
    ConversionReadyFlag = 1 << 3,
    MathOverflowFlag = 1 << 2,
    AlertPolarityBit = 1 << 1,
    AlertLatchEnable = 1,
}

/// ina226 in operational mode (connection tested and functional)
#[derive(Debug)]
pub struct Operational;

/// ina226 requiring address and connection test
#[derive(Debug)]
pub struct NonOperational;

/// ina226 register data
#[derive(Debug)]
pub struct INA226<I2C, State = NonOperational> {
    address: Option<u8>,
    i2c: Option<I2C>,
    configuration: u16,
    shunt_voltage: u16,
    bus_voltage: u16,
    power: u16,
    current: u16,
    calibration: u16,
    mask_enable: u16,
    alert_limit: u16,
    manufacturer: u16,
    die_id: u16,
    state: core::marker::PhantomData<State>,
}

///Valid error codes
#[derive(Debug, PartialEq)]
pub enum Error {
    CommunicationErr,
    InvalidParameter,
    InvalidDie,
    InvalidManufacturer,
    MissingAddress,
    MissingI2C,
}

impl Display for Error {
    #[inline]
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::InvalidDie => write!(f, "Invalid Die Number"),
            Error::CommunicationErr => write!(f, "Not found on address"),
            Error::InvalidManufacturer => write!(f, "Invalid Manufacturer"),
            Error::InvalidParameter => write!(f, "Invalid Parameter"),
            Error::MissingAddress => write!(f, "Missing Device Address"),
            Error::MissingI2C => write!(f, "Missing I2C Bus"),
        }
    }
}

/**
 * Returns communication error
 */
#[inline]
fn i2c_comm_error<E>(_: E) -> Error {
    Error::CommunicationErr
}

/**
 * Function that converts physical pin address connection to respective hexadecimal value
 */
#[inline]
pub fn convert_slave_address(a0: SlaveAddressing, a1: SlaveAddressing) -> u8 {
    match (a0, a1) {
        (SlaveAddressing::Gnd, SlaveAddressing::Gnd) => 0x40,
        (SlaveAddressing::Gnd, SlaveAddressing::Vs) => 0x41,
        (SlaveAddressing::Gnd, SlaveAddressing::Sda) => 0x42,
        (SlaveAddressing::Gnd, SlaveAddressing::Scl) => 0x43,
        (SlaveAddressing::Vs, SlaveAddressing::Gnd) => 0x44,
        (SlaveAddressing::Vs, SlaveAddressing::Vs) => 0x45,
        (SlaveAddressing::Vs, SlaveAddressing::Sda) => 0x46,
        (SlaveAddressing::Vs, SlaveAddressing::Scl) => 0x47,
        (SlaveAddressing::Sda, SlaveAddressing::Gnd) => 0x48,
        (SlaveAddressing::Sda, SlaveAddressing::Vs) => 0x49,
        (SlaveAddressing::Sda, SlaveAddressing::Sda) => 0x4A,
        (SlaveAddressing::Sda, SlaveAddressing::Scl) => 0x4B,
        (SlaveAddressing::Scl, SlaveAddressing::Gnd) => 0x4C,
        (SlaveAddressing::Scl, SlaveAddressing::Vs) => 0x4D,
        (SlaveAddressing::Scl, SlaveAddressing::Sda) => 0x4E,
        (SlaveAddressing::Scl, SlaveAddressing::Scl) => 0x4F,
    }
}

impl<I2C, E, State> Default for INA226<I2C, State>
where
    I2C: I2c<Error = E>,
{
    fn default() -> Self {
        Self::new(None)
    }
}

impl<I2C, E, State> INA226<I2C, State>
where
    I2C: I2c<Error = E>,
{
    /**
     * Method used to create a new ina226 instance
     */
    #[inline]
    pub fn new(i2c: Option<I2C>) -> Self {
        INA226 {
            address: None,
            i2c,
            configuration: 0x4127,
            shunt_voltage: 0,
            bus_voltage: 0,
            power: 0,
            current: 0,
            calibration: 8000,
            mask_enable: 0x0000,
            alert_limit: 0,
            manufacturer: 0x5449,
            die_id: 0x2260,
            state: Default::default(),
        }
    }
}

#[maybe_async_cfg::maybe(
    sync(cfg(not(feature = "async")), self = "INA226",),
    async(feature = "async", keep_self)
)]
impl<I2C, E, State> INA226<I2C, State>
where
    I2C: I2c<Error = E>,
{
    /**
     * Private method used to read the ina226 chosen register
     */
    #[inline]
    async fn read_register(&mut self, register: Register) -> Result<[u8; 2], Error> {
        let mut rx_buffer: [u8; 2] = [0; 2];
        let address = self.address.unwrap();

        self.i2c
            .as_mut()
            .unwrap()
            .write(address, &[register as u8])
            .await
            .map_err(i2c_comm_error)?;

        self.i2c
            .as_mut()
            .unwrap()
            .read(address, &mut rx_buffer)
            .await
            .map_err(i2c_comm_error)?;
        Ok(rx_buffer)
    }

    /**
     * Private method used to write to ina226 chosen register
     */
    #[inline]
    async fn write_register(&mut self, register: Register, buf: &mut [u8; 2]) {
        let address = self.address.unwrap();
        let _ = self
            .i2c
            .as_mut()
            .unwrap()
            .write(address, &[register as u8, buf[1], buf[0]])
            .await;
    }

    /**
     * This private method is used to verify if reading/writing to the hardware is working and also check if it is the correct IC.
     * This method requires that the address is previously set.
     */
    #[inline]
    async fn verify_hardware(&mut self) -> Result<(), Error> {
        match self.address {
            None => Err(Error::MissingAddress),
            Some(_) => {
                let die = self.read_register(Register::DieId).await;

                let die = match die {
                    Ok(x) => x,
                    Err(_) => {
                        return Err(Error::CommunicationErr);
                    }
                };

                let manufact = self.read_register(Register::Manufacturer).await;

                let manufact = match manufact {
                    Ok(x) => x,
                    Err(_) => {
                        return Err(Error::CommunicationErr);
                    }
                };

                if self.die_id != BigEndian::read_u16(&die) {
                    return Err(Error::InvalidDie);
                }
                if self.manufacturer != BigEndian::read_u16(&manufact) {
                    return Err(Error::InvalidManufacturer);
                }
                Ok(())
            }
        }
    }
}

#[maybe_async_cfg::maybe(
    sync(cfg(not(feature = "async")), self = "INA226",),
    async(feature = "async", keep_self)
)]
impl<I2C, E> INA226<I2C, NonOperational>
where
    I2C: I2c<Error = E>,
{
    /**
     * This method will initialize the driver, leaving it ready for operation. It requires a valid address
     */
    #[inline]
    pub async fn initialize(&mut self, i2c: I2C) -> Result<INA226<I2C, Operational>, Error> {
        match self.address {
            None => Err(Error::MissingAddress),
            Some(_) => {
                let mut ina = INA226 {
                    address: self.address,
                    i2c: Some(i2c),
                    configuration: self.configuration,
                    shunt_voltage: self.shunt_voltage,
                    bus_voltage: self.bus_voltage,
                    power: self.power,
                    current: self.current,
                    calibration: self.calibration,
                    mask_enable: self.mask_enable,
                    alert_limit: self.alert_limit,
                    manufacturer: self.manufacturer,
                    die_id: self.die_id,
                    state: core::marker::PhantomData::<Operational>,
                };

                ina.verify_hardware().await?;
                let config = ina.read_register(Register::Configuration).await?;
                ina.configuration = BigEndian::read_u16(&config);
                Ok(ina)
            }
        }
    }

    /**
     * This method can be used to manually set the ina226 address. It must be used if there are other devices or ina226 containing
     * address between 0x40 and 0x4f attached to the i2c bus
     */
    #[inline]
    pub fn set_ina_address(&mut self, address: u8) -> &mut Self {
        self.address = Some(address);
        self
    }

    /**
     * This method can be used to automatically search for ina226 address. ATTENTION: It must be the only device attached to the bus
     * containing address between 0x40 and 0x4f. If found, this will automatically set the address, thus avoiding the need to use
     * set_ina_address method
     */
    #[inline]
    pub async fn search_ina_address(&mut self) -> Result<(), Error> {
        let array = [
            SlaveAddressing::Gnd,
            SlaveAddressing::Scl,
            SlaveAddressing::Sda,
            SlaveAddressing::Vs,
        ];
        match self.i2c {
            None => Err(Error::MissingI2C),
            Some(_) => {
                for a0 in array.iter() {
                    for a1 in array.iter() {
                        self.set_ina_address(convert_slave_address(*a0, *a1));
                        if self.verify_hardware().await.is_ok() {
                            return Ok(());
                        }
                    }
                }
                Err(Error::CommunicationErr)
            }
        }
    }

    /**
     * This method returns the ina226 address currently being used
     */
    #[inline]
    pub fn get_ina_address(&self) -> Option<u8> {
        self.address
    }
}

#[maybe_async_cfg::maybe(
    sync(cfg(not(feature = "async")), self = "INA226",),
    async(feature = "async", keep_self)
)]
impl<I2C, E> INA226<I2C, Operational>
where
    I2C: I2c<Error = E>,
{
    /**
     * Method used to return the currently mode set on ina226 device (see ina226 datasheet <https://www.ti.com/product/INA226>)
     */
    #[inline]
    pub fn get_ina_mode(&self) -> Option<InaMode> {
        match self.configuration & 0x0007 {
            0x00 => Some(InaMode::PowerDown),
            0x01 => Some(InaMode::ShuntAndBusTriggered),
            0x02 => Some(InaMode::BusVoltageTriggered),
            0x03 => Some(InaMode::ShuntAndBusTriggered),
            0x04 => Some(InaMode::PowerDown2),
            0x05 => Some(InaMode::ShuntVoltageContinuous),
            0x06 => Some(InaMode::BusVoltageContinuous),
            0x07 => Some(InaMode::ShuntAndBusContinuous),
            _ => None,
        }
    }

    /**
     * Method used to set the mode of operation (see ina226 datasheet <https://www.ti.com/product/INA226>), it requires a commit()
     */
    #[inline]
    pub fn set_ina_mode(&mut self, value: InaMode) -> &mut Self {
        self.configuration &= 0xFFF8;
        self.configuration |= value as u16;
        self
    }

    /**
     * Method used to set the voltage conversion time (see ina226 datasheet <https://www.ti.com/product/INA226>), it requires a commit()
     */
    #[inline]
    pub fn set_ina_vbusct(&mut self, value: InaVbusct) -> &mut Self {
        let value = value as u16;
        self.configuration &= 0xFE3F;
        self.configuration |= value << 6;
        self
    }

    /**
     * Method used to get the currently set voltage conversion time (see ina226 datasheet <https://www.ti.com/product/INA226>)
     */
    #[inline]
    pub fn get_ina_vbusct(&self) -> Option<InaVbusct> {
        let mut result = self.configuration >> 6;
        result &= 0x07;
        match result {
            0x00 => Some(InaVbusct::_140_us),
            0x01 => Some(InaVbusct::_204_us),
            0x02 => Some(InaVbusct::_332_us),
            0x03 => Some(InaVbusct::_588_us),
            0x04 => Some(InaVbusct::_1_1_ms),
            0x05 => Some(InaVbusct::_2_116_ms),
            0x06 => Some(InaVbusct::_4_156_ms),
            0x07 => Some(InaVbusct::_8_244_ms),
            _ => None,
        }
    }

    /**
     * Method used to set the shunt voltage conversion time (see ina226 datasheet <https://www.ti.com/product/INA226>), it requires a commit()
     */
    #[inline]
    pub fn set_ina_vscht(&mut self, value: InaVshct) -> &mut Self {
        let value = value as u16;
        self.configuration &= 0xFFC7;
        self.configuration |= value << 3;
        self
    }

    /**
     * Method used to get the currently set shunt voltage conversion time (see ina226 datasheet <https://www.ti.com/product/INA226>)
     */
    #[inline]
    pub fn get_ina_vscht(&self) -> Option<InaVshct> {
        let mut result = self.configuration >> 3;
        result &= 0x07;
        match result {
            0x00 => Some(InaVshct::_140_us),
            0x01 => Some(InaVshct::_204_us),
            0x02 => Some(InaVshct::_332_us),
            0x03 => Some(InaVshct::_588_us),
            0x04 => Some(InaVshct::_1_1_ms),
            0x05 => Some(InaVshct::_2_116_ms),
            0x06 => Some(InaVshct::_4_156_ms),
            0x07 => Some(InaVshct::_8_244_ms),
            _ => None,
        }
    }

    /**
     * Method used to get the currently set averaging (see ina226 datasheet <https://www.ti.com/product/INA226>)
     */
    #[inline]
    pub fn get_ina_average(&self) -> Option<InaAverage> {
        let mut result = self.configuration >> 9;
        result &= 0x07;
        match result {
            0x00 => Some(InaAverage::_1),
            0x01 => Some(InaAverage::_4),
            0x02 => Some(InaAverage::_16),
            0x03 => Some(InaAverage::_64),
            0x04 => Some(InaAverage::_128),
            0x05 => Some(InaAverage::_256),
            0x06 => Some(InaAverage::_512),
            0x07 => Some(InaAverage::_1024),
            _ => None,
        }
    }

    /**
     * Method used to set the averaging (see ina226 datasheet <https://www.ti.com/product/INA226>), it requires a commit()
     */
    #[inline]
    pub fn set_ina_average(&mut self, value: InaAverage) -> &mut Self {
        let value = value as u16;
        self.configuration &= 0xF1FF;
        self.configuration |= value << 9;
        self
    }

    /**
     * Method used to reset the chip. It generates a system reset that is the same as power-on reset. Resets all registers to default values.
     * This method requires a commit()
     */
    #[inline]
    pub fn ina_reset(&mut self) -> &mut Self {
        self.configuration |= 1 << 15;
        self
    }

    /**
     * Method used to persist all configuration on the ina226 hardware registers. Any previous configuration that
     * is not followed by a commit() is not persisted on the hardware register and thus not valid.
     */
    #[inline]
    pub async fn commit(&mut self) {
        self.write_register(
            Register::Configuration,
            &mut self.configuration.to_le_bytes(),
        )
        .await;
        self.write_register(Register::MaskEnable, &mut self.mask_enable.to_le_bytes())
            .await;
        self.write_register(Register::Alert, &mut self.alert_limit.to_le_bytes())
            .await;
        self.write_register(Register::Calibration, &mut self.calibration.to_le_bytes())
            .await;
    }

    /**
     * Private method used to read the raw voltage from ina226
     */
    #[inline]
    pub async fn read_raw_voltage(&mut self) -> u16 {
        let result: Result<[u8; 2], Error> = self.read_register(Register::BusVoltage).await;
        self.bus_voltage = match result {
            Ok(value) => BigEndian::read_u16(&value),
            Err(_) => 0,
        };
        self.bus_voltage
    }

    /**
     * Private method used to read the raw power from ina226
     */
    #[inline]
    pub async fn read_raw_power(&mut self) -> u16 {
        let result: Result<[u8; 2], Error> = self.read_register(Register::Power).await;
        self.power = match result {
            Ok(value) => BigEndian::read_u16(&value),
            Err(_) => 0,
        };
        self.power
    }

    /**
     * Private method used to read the raw current from ina226
     */
    #[inline]
    pub async fn read_raw_current(&mut self) -> u16 {
        let result: Result<[u8; 2], Error> = self.read_register(Register::Current).await;
        self.current = match result {
            Ok(value) => BigEndian::read_u16(&value),
            Err(_) => 0,
        };
        self.current
    }

    /**
     * Private method used to read the raw shunt voltage from ina226
     */
    #[inline]
    pub async fn read_raw_shunt_voltage(&mut self) -> u16 {
        let result: Result<[u8; 2], Error> = self.read_register(Register::ShuntVoltage).await;
        self.shunt_voltage = match result {
            Ok(value) => BigEndian::read_u16(&value),
            Err(_) => 0,
        };
        self.shunt_voltage
    }

    /**
     * This method uses provided Rshunt and Maximum expected current to set the calibration register
     * Please, See the datasheet <https://www.ti.com/product/INA226>, chapter 7.5 for further information.
     * This method requires a commit()
     */
    #[cfg(not(feature = "no_float"))]
    #[inline]
    pub fn set_ina_calibration(&mut self, rshunt: f64, expect_max_curr: f64) -> &mut Self {
        let cur_lsb = expect_max_curr / (1 << 15) as f64;
        let cal = (INTERNAL_SCALING / (cur_lsb * rshunt)) as u16;

        self.calibration = cal;
        self
    }
    /**
     * Method used to manually set the calibration value (see ina226 datasheet <https://www.ti.com/product/INA226>), it requires a commit()
     */
    #[inline]
    pub fn set_ina_calibration_value(&mut self, value: u16) -> &mut Self {
        self.calibration = value;
        self
    }

    /**
     * This method is used to read the voltage, it multiplies the raw voltage by the IC resolution (in mV)
     * returning the value in volts
     */
    #[cfg(not(feature = "no_float"))]
    #[inline]
    pub async fn read_voltage(&mut self) -> f64 {
        self.read_raw_voltage().await as f64 * VOLTAGE_LSB
    }

    /**
     * This method is used to read the voltage, it multiplies the raw voltage by the IC resolution (in mV)
     * returning the value in millivolts (lower resolution)
     */
    #[cfg(feature = "no_float")]
    #[inline]
    pub async fn read_voltage_millis(&mut self) -> u32 {
        (self.read_raw_voltage().await as u32 * VOLTAGE_LSB_UV) / 1000
    }
    /**
     * This method is used to read the current, it multiplies the raw current by the IC resolution (in mA)
     * returning the value in amps
     */
    #[cfg(not(feature = "no_float"))]
    #[inline]
    pub async fn read_current(&mut self) -> f64 {
        self.read_raw_current().await as f64 * CURRENT_LSB
    }

    /**
     * This method is used to read the current, it multiplies the raw current by the IC resolution (in mA)
     * returning the value in milliamps (lower resolution)
     */
    #[cfg(feature = "no_float")]
    #[inline]
    pub async fn read_current_millis(&mut self) -> u32 {
        (self.read_raw_current().await as u32 * CURRENT_LSB_UA) / 1000
    }

    /**
     * This method is used to read the power, it multiplies the raw power by the IC resolution (in mW)
     * returning the value in watts
     */
    #[cfg(not(feature = "no_float"))]
    #[inline]
    pub async fn read_power(&mut self) -> f64 {
        self.read_raw_power().await as f64 * POWER_LSB
    }

    /**
     * This method is used to read the power, it multiplies the raw power by the IC resolution (in mW)
     * returning the value in milliwatts
     */
    #[cfg(feature = "no_float")]
    #[inline]
    pub async fn read_power_millis(&mut self) -> u32 {
        (self.read_raw_power().await as u32 * POWER_LSB_UW) / 1000
    }

    /**
     * This method is used to read the shunt voltage, it multiplies the raw shunt voltage by the IC resolution (in uV)
     * returning the value in volts
     */
    #[cfg(not(feature = "no_float"))]
    #[inline]
    pub async fn read_shunt_voltage(&mut self) -> f64 {
        self.read_raw_shunt_voltage().await as f64 * SHUNT_LSB
    }

    /**
     * This method is used to read the shunt voltage, it multiplies the raw shunt voltage by the IC resolution (in uV)
     * returning the value in nanovolts
     */
    #[cfg(feature = "no_float")]
    #[inline]
    pub async fn read_shunt_voltage_nanos(&mut self) -> u32 {
        (self.read_raw_shunt_voltage().await as u32 * SHUNT_LSB_NV) / 1000
    }

    /**
     * This method is used to read the masks register (see ina226 datasheet <https://www.ti.com/product/INA226>).
     */
    #[inline]
    pub async fn get_ina_masks(&mut self) -> u16 {
        let result: Result<[u8; 2], Error> = self.read_register(Register::MaskEnable).await;
        self.mask_enable = match result {
            Ok(value) => BigEndian::read_u16(&value),
            Err(_) => 0,
        };
        self.mask_enable
    }

    /**
     * This method is used to erase a given value on the masks register (see ina226 datasheet <https://www.ti.com/product/INA226>), it requires a commit()
     */
    #[inline]
    pub fn erase_ina_mask(&mut self, mask: MaskEnable) -> &mut Self {
        let mask = mask as u16;
        self.mask_enable &= !mask;
        self
    }

    /**
     * This method is used to set masks register (see ina226 datasheet <https://www.ti.com/product/INA226>), it requires a commit()
     */
    #[inline]
    pub fn set_ina_masks(&mut self, mask: MaskEnable) -> &mut Self {
        let mask = mask as u16;
        self.mask_enable |= mask;
        self
    }

    /**
     * This method is used to clear the masks register (see ina226 datasheet <https://www.ti.com/product/INA226>), it requires a commit()
     */
    #[inline]
    pub fn clear_ina_masks(&mut self) -> &mut Self {
        self.mask_enable = 0;
        self
    }
    /**
     * This method is used to set alert value on the alert register (see ina226 datasheet <https://www.ti.com/product/INA226>), it requires a commit()
     */
    #[inline]
    pub fn set_ina_alert(&mut self, value: u16) -> &mut Self {
        self.alert_limit = value;
        self
    }
    /**
     * This method clears the alert register, it requires commit()
     */
    #[inline]
    pub fn clear_ina_alert(&mut self) -> &mut Self {
        self.alert_limit = 0;
        self
    }

    /**
     * This method reads the Alert register and returns the result
     */
    #[inline]
    pub async fn get_ina_alert(&mut self) -> u16 {
        let result: Result<[u8; 2], Error> = self.read_register(Register::Alert).await;
        self.alert_limit = match result {
            Ok(value) => BigEndian::read_u16(&value),
            Err(_) => 0,
        };
        self.alert_limit
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use pretty_assertions::assert_eq;
    extern crate embedded_hal_mock;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
    #[cfg(not(feature = "no_float"))]
    use float_cmp::approx_eq;

    use tests::std::vec::Vec;

    fn vector1(a: u8) -> Vec<u8> {
        let mut v = Vec::new();
        v.push(a);
        v
    }
    fn vector2(a: u8, b: u8) -> Vec<u8> {
        let mut v = Vec::new();
        v.push(a);
        v.push(b);
        v
    }

    #[test]
    fn test_convert_slave_address() {
        assert_eq!(
            0x40,
            convert_slave_address(SlaveAddressing::Gnd, SlaveAddressing::Gnd)
        );
        assert_eq!(
            0x41,
            convert_slave_address(SlaveAddressing::Gnd, SlaveAddressing::Vs)
        );
        assert_eq!(
            0x42,
            convert_slave_address(SlaveAddressing::Gnd, SlaveAddressing::Sda)
        );
        assert_eq!(
            0x43,
            convert_slave_address(SlaveAddressing::Gnd, SlaveAddressing::Scl)
        );
    }

    #[test]
    fn test_verify_hardware_missing_address() {
        let expectations = [];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina: INA226<embedded_hal_mock::common::Generic<I2cTransaction>, NonOperational> =
            INA226::new(Some(i2c.clone()));
        let result = ina.verify_hardware();
        assert_eq!(Error::MissingAddress, result.unwrap_err());

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_verify_hardware_error_on_die() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(3, 4))
                .with_error(embedded_hal::i2c::ErrorKind::Other),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40);
        let result = ina.verify_hardware();
        assert_eq!(Error::CommunicationErr, result.unwrap_err());

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_verify_hardware_error_on_die_id() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(3, 4)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(3, 4)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40);
        let result = ina.verify_hardware();
        assert_eq!(Error::InvalidDie, result.unwrap_err());

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_verify_hardware_error_on_manufacturer() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(3, 4)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(3, 4))
                .with_error(embedded_hal::i2c::ErrorKind::Other),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40);
        let result = ina.verify_hardware();
        assert_eq!(Error::CommunicationErr, result.unwrap_err());

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_verify_hardware_error_on_manufacturer_id() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(3, 4)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40);
        let result = ina.verify_hardware();
        assert_eq!(Error::InvalidManufacturer, result.unwrap_err());

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_verify_hardware_ok() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40);
        let result = ina.verify_hardware();
        assert_eq!((), result.unwrap());

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_set_ina_address() {
        let expectations = [];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40);
        assert_eq!(Some(0x40 as u8), ina.address);
        ina.set_ina_address(0x88);
        assert_eq!(Some(0x88 as u8), ina.address);

        assert_eq!(Some(0x11 as u8), ina.set_ina_address(0x11).address);

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_initialize_missing_address() {
        let expectations = [];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        let result = ina.initialize(i2c.clone());
        assert_eq!(Error::MissingAddress, result.unwrap_err());

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_initialize_fail_hardware() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(3, 4))
                .with_error(embedded_hal::i2c::ErrorKind::Other),
        ];
        let mut i2c = I2cMock::new(&expectations);

        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let result = ina.initialize(i2c.clone());
        assert_eq!(Error::CommunicationErr, result.unwrap_err());

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_initialize_fail_reading_configuration() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4))
                .with_error(embedded_hal::i2c::ErrorKind::Other),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let result = ina.initialize(i2c.clone());
        assert_eq!(Error::CommunicationErr, result.unwrap_err());

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_initialize_ok() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let result = ina.initialize(i2c.clone()).unwrap();

        assert_eq!(result.address, Some(0x40));
        assert_eq!(result.configuration, 0x0304);
        assert_eq!(result.shunt_voltage, 0);
        assert_eq!(result.bus_voltage, 0);
        assert_eq!(result.power, 0);
        assert_eq!(result.current, 0);
        assert_eq!(result.calibration, 8000);
        assert_eq!(result.mask_enable, 0);
        assert_eq!(result.alert_limit, 0);
        assert_eq!(result.manufacturer, 0x5449);
        assert_eq!(result.die_id, 0x2260);

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_set_ina_mode() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();
        result.set_ina_mode(InaMode::BusVoltageContinuous);

        assert_eq!(Some(InaMode::BusVoltageContinuous), result.get_ina_mode());

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_get_ina_mode() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let result = ina.initialize(i2c.clone()).unwrap();

        assert_eq!(Some(InaMode::PowerDown2), result.get_ina_mode());

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_set_ina_vbusct() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();
        result.set_ina_vbusct(InaVbusct::_204_us);

        assert_eq!(Some(InaVbusct::_204_us), result.get_ina_vbusct());

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_get_ina_vbusct() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let result = ina.initialize(i2c.clone()).unwrap();

        assert_eq!(Some(InaVbusct::_1_1_ms), result.get_ina_vbusct());

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_set_ina_vscht() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();
        result.set_ina_vscht(InaVshct::_204_us);

        assert_eq!(Some(InaVshct::_204_us), result.get_ina_vscht());

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_get_ina_vscht() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let result = ina.initialize(i2c.clone()).unwrap();

        assert_eq!(Some(InaVshct::_140_us), result.get_ina_vscht());

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_set_ina_average() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();
        result.set_ina_average(InaAverage::_1024);

        assert_eq!(Some(InaAverage::_1024), result.get_ina_average());

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_get_ina_average() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let result = ina.initialize(i2c.clone()).unwrap();

        assert_eq!(Some(InaAverage::_4), result.get_ina_average());

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_ina_reset() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();
        result.ina_reset();

        assert_eq!(0x01 as u16, result.configuration >> 15);

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_read_raw_voltage_fail() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
            I2cTransaction::write(0x40, vector1(0x02 as u8)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8))
                .with_error(embedded_hal::i2c::ErrorKind::Other),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();

        let voltage = result.read_raw_voltage();
        assert_eq!(0 as u16, voltage);

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_read_raw_voltage_ok() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
            I2cTransaction::write(0x40, vector1(0x02 as u8)),
            I2cTransaction::read(0x40, vector2(0x00 as u8, 0x60 as u8)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();

        let voltage = result.read_raw_voltage();
        assert_eq!(0x0060 as u16, voltage);

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_read_raw_power_fail() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
            I2cTransaction::write(0x40, vector1(0x03 as u8)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8))
                .with_error(embedded_hal::i2c::ErrorKind::Other),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();

        let power = result.read_raw_power();
        assert_eq!(0 as u16, power);

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_read_raw_power_ok() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
            I2cTransaction::write(0x40, vector1(0x03 as u8)),
            I2cTransaction::read(0x40, vector2(0x00 as u8, 0x60 as u8)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();

        let power = result.read_raw_power();
        assert_eq!(0x0060 as u16, power);

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_read_raw_current_fail() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
            I2cTransaction::write(0x40, vector1(0x04 as u8)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8))
                .with_error(embedded_hal::i2c::ErrorKind::Other),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();

        let current = result.read_raw_current();
        assert_eq!(0 as u16, current);

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_read_raw_current_ok() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
            I2cTransaction::write(0x40, vector1(0x04 as u8)),
            I2cTransaction::read(0x40, vector2(0x00 as u8, 0x60 as u8)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();

        let current = result.read_raw_current();
        assert_eq!(0x0060 as u16, current);

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_read_raw_shunt_voltage_fail() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
            I2cTransaction::write(0x40, vector1(0x01 as u8)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8))
                .with_error(embedded_hal::i2c::ErrorKind::Other),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();

        let shunt_voltage = result.read_raw_shunt_voltage();
        assert_eq!(0 as u16, shunt_voltage);

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_read_raw_shunt_voltage_ok() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
            I2cTransaction::write(0x40, vector1(0x01 as u8)),
            I2cTransaction::read(0x40, vector2(0x00 as u8, 0x60 as u8)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();

        let shunt_voltage = result.read_raw_shunt_voltage();
        assert_eq!(0x0060 as u16, shunt_voltage);

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_set_ina_calibration_value() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();
        result.set_ina_calibration_value(4000);
        assert_eq!(4000, result.calibration);

        //finalize execution
        i2c.done();
    }

    #[cfg(not(feature = "no_float"))]
    #[test]
    fn test_read_voltage_ok() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
            I2cTransaction::write(0x40, vector1(0x02 as u8)),
            I2cTransaction::read(0x40, vector2(0x00 as u8, 0x60 as u8)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();

        let voltage = result.read_voltage();
        assert!(approx_eq!(f64, 0.12, voltage, ulps = 5));

        //finalize execution
        i2c.done();
    }

    #[cfg(feature = "no_float")]
    #[test]
    fn test_read_voltage_millis_ok() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
            I2cTransaction::write(0x40, vector1(0x02 as u8)),
            I2cTransaction::read(0x40, vector2(0x00 as u8, 0x60 as u8)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();

        let voltage = result.read_voltage_millis();
        assert_eq!(120, voltage);

        //finalize execution
        i2c.done();
    }

    #[cfg(not(feature = "no_float"))]
    #[test]
    fn test_read_current_ok() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
            I2cTransaction::write(0x40, vector1(0x04 as u8)),
            I2cTransaction::read(0x40, vector2(0x00 as u8, 0x60 as u8)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();

        let current = result.read_current();
        assert!(approx_eq!(f64, 0.096, current, ulps = 5));

        //finalize execution
        i2c.done();
    }

    #[cfg(feature = "no_float")]
    #[test]
    fn test_read_current_millis_ok() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
            I2cTransaction::write(0x40, vector1(0x04 as u8)),
            I2cTransaction::read(0x40, vector2(0x00 as u8, 0x60 as u8)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();

        let current = result.read_current_millis();
        assert_eq!(96, current);

        //finalize execution
        i2c.done();
    }

    #[cfg(not(feature = "no_float"))]
    #[test]
    fn test_read_power_ok() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
            I2cTransaction::write(0x40, vector1(0x03 as u8)),
            I2cTransaction::read(0x40, vector2(0x00 as u8, 0x60 as u8)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();

        let power = result.read_power();
        assert!(approx_eq!(f64, 2.4, power, ulps = 5));

        //finalize execution
        i2c.done();
    }

    #[cfg(feature = "no_float")]
    #[test]
    fn test_read_power_millis_ok() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
            I2cTransaction::write(0x40, vector1(0x03 as u8)),
            I2cTransaction::read(0x40, vector2(0x00 as u8, 0x60 as u8)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();

        let power = result.read_power_millis();
        assert_eq!(2400, power);

        //finalize execution
        i2c.done();
    }

    #[cfg(not(feature = "no_float"))]
    #[test]
    fn test_read_shunt_voltage_ok() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
            I2cTransaction::write(0x40, vector1(0x01 as u8)),
            I2cTransaction::read(0x40, vector2(0x00 as u8, 0x60 as u8)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();

        let shunt_voltage = result.read_shunt_voltage();
        assert!(approx_eq!(f64, 0.00024, shunt_voltage, ulps = 5));

        //finalize execution
        i2c.done();
    }

    #[cfg(feature = "no_float")]
    #[test]
    fn test_read_shunt_voltage_nanos_ok() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
            I2cTransaction::write(0x40, vector1(0x01 as u8)),
            I2cTransaction::read(0x40, vector2(0x00 as u8, 0x60 as u8)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();

        let shunt_voltage = result.read_shunt_voltage_nanos();
        assert_eq!(240, shunt_voltage);

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_get_ina_masks_fail() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
            I2cTransaction::write(0x40, vector1(0x06 as u8)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8))
                .with_error(embedded_hal::i2c::ErrorKind::Other),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();

        let masks = result.get_ina_masks();
        assert_eq!(0 as u16, masks);

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_get_ina_masks_ok() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
            I2cTransaction::write(0x40, vector1(0x06 as u8)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();

        let masks = result.get_ina_masks();
        assert_eq!(0x2260 as u16, masks);

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_set_ina_masks() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();

        result.set_ina_masks(MaskEnable::AlertPolarityBit);
        assert_eq!(0b0000_0000_0000_0010, result.mask_enable);
        result.set_ina_masks(MaskEnable::MathOverflowFlag);
        assert_eq!(0b0000_0000_0000_0110, result.mask_enable);
        result.set_ina_masks(MaskEnable::ShuntUnderVoltage);
        assert_eq!(0b0100_0000_0000_0110, result.mask_enable);

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_erase_ina_mask() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();

        result.set_ina_masks(MaskEnable::AlertPolarityBit);
        result.set_ina_masks(MaskEnable::MathOverflowFlag);
        result.set_ina_masks(MaskEnable::ShuntUnderVoltage);
        assert_eq!(0b0100_0000_0000_0110, result.mask_enable);
        result.erase_ina_mask(MaskEnable::MathOverflowFlag);
        assert_eq!(0b0100_0000_0000_0010, result.mask_enable);
        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_clear_ina_masks() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();

        result.set_ina_masks(MaskEnable::AlertPolarityBit);
        result.set_ina_masks(MaskEnable::MathOverflowFlag);
        result.set_ina_masks(MaskEnable::ShuntUnderVoltage);
        assert_eq!(0b0100_0000_0000_0110, result.mask_enable);
        result.clear_ina_masks();
        assert_eq!(0b0000_0000_0000_0000, result.mask_enable);

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_set_ina_alert() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();

        result.set_ina_alert(1500);
        assert_eq!(1500, result.alert_limit);

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_clear_ina_alert() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();

        result.set_ina_alert(1500);
        assert_eq!(1500, result.alert_limit);
        result.clear_ina_alert();
        assert_eq!(0, result.alert_limit);

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_get_ina_alert_fail() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
            I2cTransaction::write(0x40, vector1(0x07 as u8)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8))
                .with_error(embedded_hal::i2c::ErrorKind::Other),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();

        let alert = result.get_ina_alert();
        assert_eq!(0 as u16, alert);

        //finalize execution
        i2c.done();
    }

    #[test]
    fn test_get_ina_alert_ok() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
            I2cTransaction::write(0x40, vector1(0x07 as u8)),
            I2cTransaction::read(0x40, vector2(0x10 as u8, 0x10 as u8)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();

        let alert = result.get_ina_alert();
        assert_eq!(0x1010 as u16, alert);

        //finalize execution
        i2c.done();
    }

    #[cfg(not(feature = "no_float"))]
    #[test]
    fn test_set_ina_calibration() {
        let expectations = [
            I2cTransaction::write(0x40, vector1(0xFF)),
            I2cTransaction::read(0x40, vector2(0x22 as u8, 0x60 as u8)),
            I2cTransaction::write(0x40, vector1(0xFE)),
            I2cTransaction::read(0x40, vector2(0x54 as u8, 0x49 as u8)),
            I2cTransaction::write(0x40, vector1(0)),
            I2cTransaction::read(0x40, vector2(3, 4)),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut ina = INA226::new(Some(i2c.clone()));
        ina.set_ina_address(0x40 as u8);
        let mut result = ina.initialize(i2c.clone()).unwrap();

        let myrshut = 0.002; //Ohms
        let max_curr = 15.0; //Amps
        result.set_ina_calibration(myrshut, max_curr);
        assert_eq!(5592, result.calibration);

        //finalize execution
        i2c.done();
    }
}
