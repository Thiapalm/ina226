// Imports
use byteorder::{BigEndian, ByteOrder};
use core::fmt::Display;
use embedded_hal::blocking::i2c::{Write, WriteRead};

const SHUNT_LSB: f64 = 0.0000025; // in Volts (2.5 uV)
const VOLTAGE_LSB: f64 = 0.00125; // in Volts (1.25 mV)
const CURRENT_LSB: f64 = 0.001; // in Amps (1 mA)
const POWER_LSB: f64 = 0.025; // in Watts (25 mW)

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

pub enum Register {
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

pub struct Operational;
pub struct NonOperational;

#[derive(Debug, Clone, Copy)]
pub struct Ina226<State = NonOperational> {
    address: Option<u8>,
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

#[derive(Debug)]
pub enum Error {
    CommunicationErr,
    InvalidParameter,
    InvalidDie,
    InvalidManufacturer,
    MissingAddress,
}

impl Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::InvalidDie => write!(f, "Invalid Die Number"),
            Error::CommunicationErr => write!(f, "Not found on address"),
            Error::InvalidManufacturer => write!(f, "Invalid Manufacturer"),
            Error::InvalidParameter => write!(f, "Invalid Parameter"),
            Error::MissingAddress => write!(f, "Missing Device Address"),
        }
    }
}

fn i2c_error<E>(_: E) -> Error {
    Error::CommunicationErr
}

/**
 * Function that converts physical pin address connection to respective hexadecimal value
 */
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

impl Default for Ina226 {
    fn default() -> Self {
        Self::new()
    }
}

impl Ina226 {
    pub fn new() -> Self {
        Ina226 {
            address: None,
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

impl<State> Ina226<State> {
    /**
     * Private method used to read the ina226 chosen register
     */
    fn read_register<I2C, E>(&mut self, i2c: &mut I2C, register: Register) -> Result<[u8; 2], Error>
    where
        I2C: WriteRead<Error = E>,
    {
        let mut rx_buffer: [u8; 2] = [0; 2];
        let address = self.address.unwrap();

        i2c.write_read(address, &[register as u8], &mut rx_buffer)
            .map_err(i2c_error)?;
        Ok(rx_buffer)
    }

    /**
     * Private method used to write to ina226 chosen register
     */
    fn write_register<I2C, E>(&mut self, i2c: &mut I2C, register: Register, buf: &mut [u8; 2])
    where
        I2C: WriteRead<Error = E> + Write<Error = E>,
    {
        let address = self.address.unwrap();
        let _ = i2c.write(address, &[register as u8, buf[1], buf[0]]);
    }
}

impl Ina226<NonOperational> {
    /**
     * This private method is used to verify if reading/writing to the hardware is working and also check if it is the correct IC.
     * This method requires that the address is previously set.
     */
    fn verify_hardware<I2C, E>(&mut self, i2c: &mut I2C) -> Result<(), Error>
    where
        I2C: WriteRead<Error = E> + Write<Error = E>,
    {
        match self.address {
            None => Err(Error::MissingAddress),
            Some(_) => {
                let die = self.read_register(i2c, Register::DieId);

                let die = match die {
                    Ok(x) => x,
                    Err(_) => {
                        return Err(Error::CommunicationErr);
                    }
                };

                let manufact = self.read_register(i2c, Register::Manufacturer).unwrap();

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

    /**
     * This method will initialize the driver, leaving it ready for operation. It requires a valid address
     */
    pub fn initialize<I2C, E>(&mut self, i2c: &mut I2C) -> Result<Ina226<Operational>, Error>
    where
        I2C: WriteRead<Error = E> + Write<Error = E>,
    {
        match self.address {
            None => Err(Error::MissingAddress),
            Some(_) => {
                self.verify_hardware(i2c)?;
                let config = self.read_register(i2c, Register::Configuration)?;
                self.configuration = BigEndian::read_u16(&config);

                Ok(Ina226 {
                    address: self.address,
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
                })
            }
        }
    }

    /**
     * This method can be used to manually set the ina226 address. It must be used if there are other devices or ina226 containing
     * address between 0x40 and 0x4f attached to the i2c bus
     */
    pub fn set_ina_address(&mut self, address: u8) -> &mut Self {
        self.address = Some(address);
        self
    }

    /**
     * This method can be used to automatically search for ina226 address. ATTENTION: It must be the only device attached to the bus
     * containing address between 0x40 and 0x4f. If found, this will automatically set the address, thus avoiding the need to use
     * set_ina_address method
     */
    pub fn search_ina_address<I2C, E>(&mut self, i2c: &mut I2C) -> Result<(), Error>
    where
        I2C: WriteRead<Error = E> + Write<Error = E>,
    {
        let array = [
            SlaveAddressing::Gnd,
            SlaveAddressing::Scl,
            SlaveAddressing::Sda,
            SlaveAddressing::Vs,
        ];
        for a0 in array.iter() {
            for a1 in array.iter() {
                self.set_ina_address(convert_slave_address(*a0, *a1));
                if self.verify_hardware(i2c).is_ok() {
                    return Ok(());
                }
            }
        }
        Err(Error::CommunicationErr)
    }

    /**
     * This method returns the ina226 address currently being used
     */
    pub fn get_ina_address(&self) -> Option<u8> {
        self.address
    }
}

impl Ina226<Operational> {
    /**
     * Method used to return the currently mode set on ina226 device (see ina226 datasheet <https://www.ti.com/product/INA226>)
     */
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
    pub fn set_ina_mode(&mut self, value: InaMode) -> &mut Self {
        self.configuration &= 0xFFF8;
        self.configuration |= value as u16;
        self
    }

    /**
     * Method used to set the voltage conversion time (see ina226 datasheet <https://www.ti.com/product/INA226>), it requires a commit()
     */
    pub fn set_ina_vbusct(&mut self, value: InaVbusct) -> &mut Self {
        let value = value as u16;
        self.configuration &= 0xFE3F;
        self.configuration |= value << 6;
        self
    }

    /**
     * Method used to get the currently set voltage conversion time (see ina226 datasheet <https://www.ti.com/product/INA226>)
     */
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
    pub fn set_ina_vscht(&mut self, value: InaVshct) -> &mut Self {
        let value = value as u16;
        self.configuration &= 0xFFC7;
        self.configuration |= value << 3;
        self
    }

    /**
     * Method used to get the currently set shunt voltage conversion time (see ina226 datasheet <https://www.ti.com/product/INA226>)
     */
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
     * Method used to reset the chip. It generates a system reset that is the same as power-on reset. Resets all registers to default values.
     * This method requires a commit()
     */
    pub fn ina_reset(&mut self) -> &mut Self {
        self.configuration |= 1 << 15;
        self
    }
    /**
     * Method used to set the averaging (see ina226 datasheet <https://www.ti.com/product/INA226>), it requires a commit()
     */
    pub fn set_ina_average(&mut self, value: InaAverage) -> &mut Self {
        let value = value as u16;
        self.configuration &= 0xF1FF;
        self.configuration |= value << 9;
        self
    }

    /**
     * Method used to persist all configuration on the ina226 hardware registers. Any previous configuration that
     * is not followed by a commit() is not persisted on the hardware register and thus not valid.
     */
    pub fn commit<I2C, E>(&mut self, i2c: &mut I2C)
    where
        I2C: WriteRead<Error = E> + Write<Error = E>,
    {
        self.write_register(
            i2c,
            Register::Configuration,
            &mut self.configuration.to_le_bytes(),
        );
        self.write_register(
            i2c,
            Register::MaskEnable,
            &mut self.mask_enable.to_le_bytes(),
        );
        self.write_register(i2c, Register::Alert, &mut self.alert_limit.to_le_bytes());
        self.write_register(
            i2c,
            Register::Calibration,
            &mut self.calibration.to_le_bytes(),
        );
    }

    /**
     * Private method used to read the raw voltage from ina226
     */
    fn read_raw_voltage<I2C, E>(&mut self, i2c: &mut I2C) -> u16
    where
        I2C: WriteRead<Error = E> + Write<Error = E>,
    {
        let result = self.read_register(i2c, Register::BusVoltage);
        self.bus_voltage = match result {
            Ok(value) => BigEndian::read_u16(&value),
            Err(_) => 0,
        };
        self.bus_voltage
    }

    /**
     * Private method used to read the raw power from ina226
     */
    fn read_raw_power<I2C, E>(&mut self, i2c: &mut I2C) -> u16
    where
        I2C: WriteRead<Error = E> + Write<Error = E>,
    {
        let result = self.read_register(i2c, Register::Power);
        self.power = match result {
            Ok(value) => BigEndian::read_u16(&value),
            Err(_) => 0,
        };
        self.power
    }

    /**
     * Private method used to read the raw current from ina226
     */
    fn read_raw_current<I2C, E>(&mut self, i2c: &mut I2C) -> u16
    where
        I2C: WriteRead<Error = E> + Write<Error = E>,
    {
        let result = self.read_register(i2c, Register::Current);
        self.current = match result {
            Ok(value) => BigEndian::read_u16(&value),
            Err(_) => 0,
        };
        self.current
    }

    /**
     * Private method used to read the raw shunt voltage from ina226
     */
    fn read_raw_shunt_voltage<I2C, E>(&mut self, i2c: &mut I2C) -> u16
    where
        I2C: WriteRead<Error = E> + Write<Error = E>,
    {
        let result = self.read_register(i2c, Register::ShuntVoltage);
        self.shunt_voltage = match result {
            Ok(value) => BigEndian::read_u16(&value),
            Err(_) => 0,
        };
        self.shunt_voltage
    }

    /**
     * Method used to set the shunt value (see ina226 datasheet <https://www.ti.com/product/INA226>), it requires a commit()
     */
    pub fn set_ina_shunt_value(&mut self, value: u16) -> &mut Self {
        self.calibration = value;
        self
    }

    /**
     * This method is used to read the voltage, it multiplies the raw voltage by the IC resolution (in mV)
     * returning the value in volts
     */
    pub fn read_voltage<I2C, E>(&mut self, i2c: &mut I2C) -> f64
    where
        I2C: WriteRead<Error = E> + Write<Error = E>,
    {
        self.read_raw_voltage(i2c) as f64 * VOLTAGE_LSB
    }

    /**
     * This method is used to read the current, it multiplies the raw current by the IC resolution (in mA)
     * returning the value in amps
     */
    pub fn read_current<I2C, E>(&mut self, i2c: &mut I2C) -> f64
    where
        I2C: WriteRead<Error = E> + Write<Error = E>,
    {
        self.read_raw_current(i2c) as f64 * CURRENT_LSB
    }

    /**
     * This method is used to read the power, it multiplies the raw power by the IC resolution (in mW)
     * returning the value in watts
     */
    pub fn read_power<I2C, E>(&mut self, i2c: &mut I2C) -> f64
    where
        I2C: WriteRead<Error = E> + Write<Error = E>,
    {
        self.read_raw_power(i2c) as f64 * POWER_LSB
    }

    /**
     * This method is used to read the shunt voltage, it multiplies the raw shunt voltage by the IC resolution (in uV)
     * returning the value in volts
     */
    pub fn read_shunt_voltage<I2C, E>(&mut self, i2c: &mut I2C) -> f64
    where
        I2C: WriteRead<Error = E> + Write<Error = E>,
    {
        self.read_raw_shunt_voltage(i2c) as f64 * SHUNT_LSB
    }

    /**
     * This method is used to read the masks register (see ina226 datasheet <https://www.ti.com/product/INA226>).
     */
    pub fn get_ina_masks<I2C, E>(&mut self, i2c: &mut I2C) -> u16
    where
        I2C: WriteRead<Error = E> + Write<Error = E>,
    {
        let result = self.read_register(i2c, Register::MaskEnable);
        self.mask_enable = match result {
            Ok(value) => BigEndian::read_u16(&value),
            Err(_) => 0,
        };
        self.mask_enable
    }

    /**
     * This method is used to clear the masks register (see ina226 datasheet <https://www.ti.com/product/INA226>), it requires a commit()
     */
    pub fn clear_ina_masks(&mut self, mask: MaskEnable) -> &mut Self {
        let mask = mask as u16;
        self.mask_enable &= !mask;
        self
    }

    /**
     * This method is used to set masks register (see ina226 datasheet <https://www.ti.com/product/INA226>), it requires a commit()
     */
    pub fn set_ina_masks(&mut self, mask: MaskEnable) -> &mut Self {
        let mask = mask as u16;
        self.mask_enable |= mask;
        self
    }

    /**
     * This method is used to set alert value on the alert register (see ina226 datasheet <https://www.ti.com/product/INA226>), it requires a commit()
     */
    pub fn set_ina_alert(&mut self, value: u16) -> &mut Self {
        self.alert_limit = value;
        self
    }
    /**
     * This method clears the alert register, it requires commit()
     */
    pub fn clear_ina_alert(&mut self) -> &mut Self {
        self.alert_limit = 0;
        self
    }

    /**
     * This method reads the Alert register and returns the result
     */
    pub fn get_ina_alert<I2C, E>(&mut self, i2c: &mut I2C) -> u16
    where
        I2C: WriteRead<Error = E> + Write<Error = E>,
    {
        let result = self.read_register(i2c, Register::Alert);
        self.mask_enable = match result {
            Ok(value) => BigEndian::read_u16(&value),
            Err(_) => 0,
        };
        self.alert_limit
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pretty_assertions::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}
