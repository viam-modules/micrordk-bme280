//! bmp280 module implements the sensor interface for a Bosch BMP280 Digital Pressure Sensor.
//! (datasheet)[https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf]
//! The chip can communicate via I2C or SPI, determined by the chip select bus (CSB) bin.
//! - CSB Pulled High: IC2
//! - CSB Pulled Low: SPI
//!
//! The chip has two possible I2C addresses, which can be selected by wiring the SDO pin to GND or Vddio.
//! GND: 1110110 (0x76)
//! Vddio: 1110111 (0x77), same as BMP180 and BME280 i2c addresses.
//! The SDO pin cannot be left floating; if left floating, the i2c address will be undefined.
//!
//! Ic2 interface uses the following pins:
//! - SCK: serial clock (SCL)
//! - SDI: data (SDA)
//! - SDO: slave address
//!
//! ## I2C Write
//! Sending the slave address in write mode (RW=0) followed by pairs of register addresses and register data.
//! Start(1bit)->SlaveAddress(7bit)->RW(1bit)->ACKS
//! RegisterAddress(8bit)->ACKS
//! Register Data(8bit)->ACKS
//! ...
//! Stop(1bit)
//!
//! ## I2C Read
//! Sending the slave address in write mode (RW=0) followed by the address of the register to be read.
//! Sending the slave address in read mode (RW=1) is then followed by the slave sending back auto-incremented register data,
//! terminated by a NOACKM and Stop.
//!
//! ## Data Readout
//! To read out data after a conversion, use burst read and not individual registers.
//! To read data, start a burst read from 0xF7(press_msb) to 0xFC(temp_xlsb).
//! Data er read out in an unsigned 20-bit format for both temperature and pressure.

use micro_rdk::common::config::ConfigType;
use micro_rdk::common::registry::{ComponentRegistry, Dependency, RegistryError};
use micro_rdk::common::status::{Status, StatusError};
use micro_rdk::DoCommand;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use micro_rdk::common::sensor::{Readings, Sensor, SensorError, SensorType};

pub fn register_models(registry: &mut ComponentRegistry) -> Result<(), RegistryError> {
    registry.register_sensor("bmp280", &Bmp280::from_config)
}

enum Register {
    TempXlsb = 0xFC,
    TempLsb = 0xFB,
    TempMsb = 0xFA,
    PressXlsb = 0xF9,
    PressLsb = 0xF8,
    PressMsb = 0xF7,
    /// the `config` register sets the rate, filter, and interface options of the device.
    /// Writes to the `config` register might be ignored when in normal mode (cyclic reading).
    /// In sleep mode, writes are not ignored.
    Config = 0xF5,
    /// the `ctrl_meas` register sets the data acquisition options of the device; ie oversampling settings and device power mode.
    CtrlMeas = 0xF4,
    /// the `status` register contains two bits regarding the status of the device.
    /// Bit3 read as 1 whenever a conversion is running, read as 0 when results are transfered to data register  .
    /// Bit0 read as 1 when NVM data being copied to image registers. Read as 0 when copying is done. Data is copied
    /// at power-on-reset and before every conversion    
    Status = 0xF3,
    /// the `reset` register, if the value 0xB6 is written to this register, the device is reset using the complete power-on-reset procesdure.
    Reset = 0xE0,
    Id = 0xD0,
}

enum BmpAddr {
    Gnd = 0x76,
    Hot = 0x77,
}

#[derive(DoCommand)]
pub struct Bmp280 {}

impl Bmp280 {
    pub fn from_config(cfg: ConfigType, deps: Vec<Dependency>) -> Result<SensorType, SensorError> {
        Ok(Arc::new(Mutex::new(MySensor {})))
    }
}

impl Status for Bmp280 {
    fn get_status(&self) -> Result<Option<micro_rdk::google::protobuf::Struct>, StatusError> {
        Ok(Some(micro_rdk::google::protobuf::Struct {
            fields: HashMap::new(),
        }))
    }
}
