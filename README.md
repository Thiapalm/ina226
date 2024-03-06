# `ina226`

> I²C API for INA226 High-Side or Low-Side Measurement, Bi-Direcional Current and Power Monitor

# [ina226 Datasheet](https://www.ti.com/product/INA226)

# Description

This crate was made for and tested on INA226 from Texas Instruments, it is based on I2C from embedded-hal crate.
The implementation of this crate is based on #![no_std] but with some minor adjustments it can be used on std environments.

This driver allows you to:
- Manually configure INA226 address
- Automatically detect INA226 address
- Fully configure the configuration register
- Calibrate the device
- Manually configure the Mask/Enable register
- Manually configure the Alert register
- Read Voltage, shunt voltage, current and power
- Test device connection for errors
- Reset the device to factory specs

This driver contains two modes, non operational and operational:
Non Operational:
 - When creating the device it starts on this mode
 - you can manualy set the address or auto detect it
 - when initializing the device it will test the connection and then move to Operational mode.

 Operational:
 - Set configuration, mask/enable and alert registers
 - Set the calibration register based on Rshunt and MaxAmp (set_ina_calibration)
 - Set the calibration register manually (set_ina_calibration_value)
 - Read configured values
 - Reset the device to factory specs
 - Read voltage, shunt voltage, current and power

 Please note that after any configuration change, a commit operation must be performed so the information can be sent to the IC.

 If the device is not calibrated, it assumes a shunt resistor value of 2 mOhms and maximum current of 10 Amps

# Version Revision

0.1.0 - First Version
0.2.0 - Implemented Compatibility with embedded_hal 1.0

# Example

To use the driver, you must have a concrete implementation of the
[embedded-hal](https://crates.io/crates/embedded-hal) traits.  This example uses
[stm32f4xx-hal](https://crates.io/crates/stm32f4xx-hal):

If you need to search for INA address on the bus, or need to use it with other I2C devices, then you will need to use embedded_hal_bus implementation as below:


``` rust
use core::cell::RefCell;
use embedded_hal_bus::i2c;

    let mut i2c = dp.I2C1.i2c(
        (scl, sda),
        Mode::Standard {
            frequency: 100.kHz(),
        },
        &clocks,
    );

    let i2c_ref_cell = RefCell::new(i2c);

    let mut ina_device = ina226::Ina226::new(Some(i2c::RefCellDevice::new(&i2c_ref_cell)));

    match ina_device.search_ina_address() {
        Ok(_) => {
            rprintln!(
                "Found Address {:#02x}",
                ina_device.get_ina_address().unwrap()
            );
        }
        Err(x) => {
            rprintln!("Error: {}", x);
            panic!();
        }
    };

    let mut ina_device = match ina_device.initialize(i2c::RefCellDevice::new(&i2c_ref_cell)) {
        Ok(x) => {
            rprintln!("Found INA226!");
            x
        }
        Err(err) => {
            rprintln!("Invalid device: {}", err);
            panic!();
        }
    };
```
If you dont need to search for its address or have other I2C devices on the bus, then just pass None to the new function and use i2c directly


```rust
    let mut i2c = dp.I2C1.i2c(
        (scl, sda),
        Mode::Standard {
            frequency: 100.kHz(),
        },
        &clocks,
    );

    let mut ina_device = ina226::Ina226::new(None);

    ina_device.set_address(0x40);

    let mut ina_device = match ina_device.initialize(i2c) {
        Ok(x) => {
            rprintln!("Found INA226!");
            x
        }
        Err(err) => {
            rprintln!("Invalid device: {}", err);
            panic!();
        }
    };
```

From now on you can configure and use the device normally

```rust
    ina_device
        .set_ina_mode(InaMode::ShuntAndBusContinuous)
        .set_ina_average(InaAverage::_512)
        .set_ina_vbusct(InaVbusct::_1_1_ms)
        .set_ina_vscht(InaVshct::_1_1_ms)
        .commit();

    ina_device.set_ina_calibration_value(6000).commit();

    rprintln!("Voltage: {:.2} V", ina_device.read_voltage());
    rprintln!("Current: {:.3} A", ina_device.read_current());
    rprintln!("Power: {:.3} W", ina_device.read_power());
    rprintln!(
        "Shunt Voltage: {:.6} V",
        ina_device.read_shunt_voltage()
    );

```

# License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)

- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.