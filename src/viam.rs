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
//!
//!
//! Attributions:
//! - initial config snippets from (bmp280 driver)[https://github.com/copterust/bmp280] by coterust (MIT)

use micro_rdk::common::board::Board;
use micro_rdk::common::config::ConfigType;
use micro_rdk::common::generic::DoCommand;
use micro_rdk::common::i2c::{I2CHandle, I2cHandleType};
use micro_rdk::common::registry::{ComponentRegistry, Dependency, RegistryError};
use micro_rdk::common::status::{Status, StatusError};
use micro_rdk::DoCommand;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use micro_rdk::common::sensor::{
    GenericReadingsResult, Readings, Sensor, SensorError, SensorT, SensorType,
};

pub fn register_models(registry: &mut ComponentRegistry) -> Result<(), RegistryError> {
    registry.register_sensor("bmp280-viam", &Bmp280::from_config)
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
    Calib = 0x88,
}

enum BmpAddr {
    Low = 0x76,
    High = 0x77,
}

#[derive(DoCommand)]
pub struct Bmp280 {
    i2c_handle: I2cHandleType,
    i2c_address: u8,
    calib: Calibration,
}

impl Status for Bmp280 {
    fn get_status(&self) -> Result<Option<micro_rdk::google::protobuf::Struct>, StatusError> {
        let status = self.status();
        let mut fields = HashMap::new();
        fields.insert(
            "measuring".to_string(),
            micro_rdk::google::protobuf::Value {
                kind: Some(micro_rdk::google::protobuf::value::Kind::BoolValue(
                    status.measuring,
                )),
            },
        );
        fields.insert(
            "im_update".to_string(),
            micro_rdk::google::protobuf::Value {
                kind: Some(micro_rdk::google::protobuf::value::Kind::BoolValue(
                    status.im_update,
                )),
            },
        );
        Ok(Some(micro_rdk::google::protobuf::Struct { fields }))
    }
}

impl Sensor for Bmp280 {}

impl SensorT for Bmp280 {
    fn get_readings(
        &self,
    ) -> Result<micro_rdk::common::sensor::TypedReadingsResult<T>, SensorError> {
        let register_write: [u8; 1] = [Register::PressMsb as u8];
        let mut result: [u8; 24] = [0; 24];
        self.i2c_handle
            .write_read_i2c(self.i2c_address, &register_write, &mut result)?;
        Ok(self.temp_one_shot())
    }
}

impl Readings for Bmp280 {
    fn get_generic_readings(
        &mut self,
    ) -> Result<micro_rdk::common::sensor::GenericReadingsResult, SensorError> {
        unimplemented!();
    }
}

impl Bmp280 {
    pub fn from_config(cfg: ConfigType, deps: Vec<Dependency>) -> Result<SensorType, SensorError> {
        let board = micro_rdk::common::registry::get_board_from_dependencies(deps);
        if board.is_none() {
            return Err(SensorError::ConfigError("BMP280 missing board attribute"));
        }
        let board = board.unwrap();
        let i2c_handle: I2cHandleType;
        if let Ok(i2c_name) = cfg.get_attribute::<String>("i2c_bus") {
            i2c_handle = board.get_i2c_by_name(i2c_name)?;
        } else {
            return Err(SensorError::ConfigError("BMP280 missing i2c_bus attribute"));
        }
        if let Ok(sdo_setting) = cfg.get_attribute::<String>("sdo_setting") {
            let i2c_address: u8;
            match sdo_setting.as_str() {
                "high" => {
                    i2c_address = BmpAddr::High as u8;
                }
                "low" => {
                    i2c_address = BmpAddr::Low as u8;
                }
                _ => {
                    //log::error!("sdo_setting must be either `high` or `low`");
                    return Err(SensorError::ConfigError("invalid sdo_setting"));
                }
            }

            let mut driver = Self {
                i2c_handle,
                i2c_address,
                calib: Default::default(),
            };
            if driver.id() == 0x58 {
                driver.get_calibration();
            }

            return Ok(Arc::new(Mutex::new(driver)));
        } else {
            //log::error!("sdo_setting attribute must be either `high` or `low`");
            return Err(SensorError::ConfigError(
                "BMP280 missing sdo_setting attribute",
            ));
        }
    }

    fn get_calibration(&mut self) {
        let mut data: [u8; 24] = [0; 24];
        let _ =
            self.i2c_handle
                .write_read_i2c(self.i2c_address, &[Register::Calib as u8], &mut data);

        self.calib.dig_t1 = ((data[1] as u16) << 8) | (data[0] as u16);
        self.calib.dig_t2 = ((data[3] as i16) << 8) | (data[2] as i16);
        self.calib.dig_t3 = ((data[5] as i16) << 8) | (data[4] as i16);
        self.calib.dig_p1 = ((data[7] as u16) << 8) | (data[6] as u16);
        self.calib.dig_p2 = ((data[9] as i16) << 8) | (data[8] as i16);
        self.calib.dig_p3 = ((data[11] as i16) << 8) | (data[10] as i16);
        self.calib.dig_p4 = ((data[13] as i16) << 8) | (data[12] as i16);
        self.calib.dig_p5 = ((data[15] as i16) << 8) | (data[14] as i16);
        self.calib.dig_p6 = ((data[17] as i16) << 8) | (data[16] as i16);
        self.calib.dig_p7 = ((data[19] as i16) << 8) | (data[18] as i16);
        self.calib.dig_p8 = ((data[21] as i16) << 8) | (data[20] as i16);
        self.calib.dig_p9 = ((data[23] as i16) << 8) | (data[22] as i16);
    }

    /// Reads and returns pressure
    pub fn pressure(&mut self) -> f64 {
        let mut data: [u8; 6] = [0, 0, 0, 0, 0, 0];
        let _ = self.i2c_handle.write_read_i2c(
            self.i2c_address,
            &[Register::PressMsb as u8],
            &mut data,
        );
        let press = (data[0] as u32) << 12 | (data[1] as u32) << 4 | (data[2] as u32) >> 4;

        let mut var1 = ((self.calib.t_fine as f64) / 2.0) - 64000.0;
        let mut var2 = var1 * var1 * (self.calib.dig_p6 as f64) / 32768.0;
        var2 += var1 * (self.calib.dig_p5 as f64) * 2.0;
        var2 = (var2 / 4.0) + ((self.calib.dig_p4 as f64) * 65536.0);
        var1 = ((self.calib.dig_p3 as f64) * var1 * var1 / 524288.0
            + (self.calib.dig_p2 as f64) * var1)
            / 524288.0;
        var1 = (1.0 + var1 / 32768.0) * (self.calib.dig_p1 as f64);
        let mut pressure = 1048576.0 - (press as f64);
        if var1 != 0.0 {
            pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
            var1 = (self.calib.dig_p9 as f64) * pressure * pressure / 2147483648.0;
            var2 = pressure * (self.calib.dig_p8 as f64) / 32768.0;
            pressure += (var1 + var2 + (self.calib.dig_p7 as f64)) / 16.0;
        }
        pressure
    }

    /// Reads and returns pressure and resets control
    fn pressure_one_shot(&mut self) -> f64 {
        let pressure = self.pressure();
        self.set_control(Control {
            osrs_t: Oversampling::x2,
            osrs_p: Oversampling::x16,
            mode: PowerMode::Forced,
        });

        pressure
    }

    /// Reads and returns temperature
    fn temp(&mut self) -> f64 {
        let mut data: [u8; 6] = [0, 0, 0, 0, 0, 0];
        let _ = self.i2c_handle.write_read_i2c(
            self.i2c_address,
            &[Register::PressMsb as u8],
            &mut data,
        );
        let _pres = (data[0] as u32) << 12 | (data[1] as u32) << 4 | (data[2] as u32) >> 4;
        let temp = (data[3] as u32) << 12 | (data[4] as u32) << 4 | (data[5] as u32) >> 4;

        let v1 = ((temp as f64) / 16384.0 - (self.calib.dig_t1 as f64) / 1024.0)
            * (self.calib.dig_t2 as f64);
        let v2 = (((temp as f64) / 131072.0 - (self.calib.dig_t1 as f64) / 8192.0)
            * ((temp as f64) / 131072.0 - (self.calib.dig_t1 as f64) / 8192.0))
            * (self.calib.dig_t3 as f64);
        self.calib.t_fine = (v1 + v2) as i32;

        (v1 + v2) / 5120.0
    }

    /// Reads and returns temperature and resets control
    fn temp_one_shot(&mut self) -> f64 {
        let temp = self.temp();
        self.set_control(Control {
            osrs_t: Oversampling::x2,
            osrs_p: Oversampling::x16,
            mode: PowerMode::Forced,
        });

        temp
    }

    /// Returns current config
    fn config(&mut self) -> Config {
        let config = self.read_byte(Register::Config);
        let t_sb = match (config & (0b111 << 5)) >> 5 {
            x if x == Standby::ms0_5 as u8 => Standby::ms0_5,
            x if x == Standby::ms62_5 as u8 => Standby::ms62_5,
            x if x == Standby::ms125 as u8 => Standby::ms125,
            x if x == Standby::ms250 as u8 => Standby::ms250,
            x if x == Standby::ms500 as u8 => Standby::ms500,
            x if x == Standby::ms1000 as u8 => Standby::ms1000,
            x if x == Standby::ms2000 as u8 => Standby::ms2000,
            x if x == Standby::ms4000 as u8 => Standby::ms4000,
            _ => Standby::unknown,
        };
        let filter = match (config & (0b111 << 2)) >> 2 {
            x if x == Filter::off as u8 => Filter::off,
            x if x == Filter::c2 as u8 => Filter::c2,
            x if x == Filter::c4 as u8 => Filter::c4,
            x if x == Filter::c8 as u8 => Filter::c8,
            x if x == Filter::c16 as u8 => Filter::c16,
            _ => Filter::unknown,
        };
        Config { t_sb, filter }
    }

    /// Sets configuration
    fn set_config(&mut self, new: Config) {
        let config: u8 = 0x00;
        let t_sb = (new.t_sb as u8) << 5;
        let filter = (new.filter as u8) << 2;
        self.write_byte(Register::Config, config | t_sb | filter);
    }

    /// Sets control
    fn set_control(&mut self, new: Control) {
        let osrs_t: u8 = (new.osrs_t as u8) << 5;
        let osrs_p: u8 = (new.osrs_p as u8) << 2;
        let control: u8 = osrs_t | osrs_p | (new.mode as u8);
        self.write_byte(Register::CtrlMeas, control);
    }

    /// Returns control
    fn control(&mut self) -> Control {
        let config = self.read_byte(Register::CtrlMeas);
        let osrs_t = match (config & (0b111 << 5)) >> 5 {
            x if x == Oversampling::skipped as u8 => Oversampling::skipped,
            x if x == Oversampling::x1 as u8 => Oversampling::x1,
            x if x == Oversampling::x2 as u8 => Oversampling::x2,
            x if x == Oversampling::x4 as u8 => Oversampling::x4,
            x if x == Oversampling::x8 as u8 => Oversampling::x8,
            _ => Oversampling::x16,
        };
        let osrs_p = match (config & (0b111 << 2)) >> 2 {
            x if x == Oversampling::skipped as u8 => Oversampling::skipped,
            x if x == Oversampling::x1 as u8 => Oversampling::x1,
            x if x == Oversampling::x2 as u8 => Oversampling::x2,
            x if x == Oversampling::x4 as u8 => Oversampling::x4,
            x if x == Oversampling::x8 as u8 => Oversampling::x8,
            _ => Oversampling::x16,
        };
        let mode = match config & 0b11 {
            x if x == PowerMode::Sleep as u8 => PowerMode::Sleep,
            x if x == PowerMode::Forced as u8 => PowerMode::Forced,
            x if x == PowerMode::Normal as u8 => PowerMode::Normal,
            _ => PowerMode::Forced,
        };

        Control {
            osrs_t,
            osrs_p,
            mode,
        }
    }

    /// Returns device status
    fn status(&mut self) -> SensorStatus {
        let status = self.read_byte(Register::Status);
        SensorStatus {
            measuring: 0 != (status & 0b00001000),
            im_update: 0 != (status & 0b00000001),
        }
    }

    /// Returns device id
    fn id(&mut self) -> u8 {
        self.read_byte(Register::Id)
    }

    /// Software reset, emulates POR
    fn reset(&mut self) {
        self.write_byte(Register::Reset, 0xB6); // Magic from documentation
    }

    fn write_byte(&mut self, reg: Register, byte: u8) {
        let mut buffer = [0];
        let _ = self
            .i2c_handle
            .write_read_i2c(self.i2c_address, &[reg as u8, byte], &mut buffer);
    }

    fn read_byte(&mut self, reg: Register) -> u8 {
        let mut data: [u8; 1] = [0];
        let _ = self
            .i2c_handle
            .write_read_i2c(self.i2c_address, &[reg as u8], &mut data);
        data[0]
    }
}

/// Calibration manufacturer settings used to calculate temperature and pressure readings
#[derive(Default)]
struct Calibration {
    dig_t1: u16,
    dig_t2: i16,
    dig_t3: i16,
    t_fine: i32,
    dig_p1: u16,
    dig_p2: i16,
    dig_p3: i16,
    dig_p4: i16,
    dig_p5: i16,
    dig_p6: i16,
    dig_p7: i16,
    dig_p8: i16,
    dig_p9: i16,
}

#[derive(Debug, Copy, Clone)]
/// Control
pub struct Control {
    /// Temperature oversampling
    pub osrs_t: Oversampling,
    /// Pressure oversampling
    pub osrs_p: Oversampling,
    /// Powermode
    pub mode: PowerMode,
}

#[derive(Debug, Copy, Clone)]
#[allow(non_camel_case_types)]
/// Standby time in ms
pub enum Standby {
    /// ms0_5
    ms0_5 = 0b000,
    /// ms62_5
    ms62_5 = 0b001,
    /// ms125_5
    ms125 = 0b010,
    /// ms250
    ms250 = 0b011,
    /// ms500
    ms500 = 0b100,
    /// ms1000
    ms1000 = 0b101,
    /// ms2000
    ms2000 = 0b110,
    /// ms4000
    ms4000 = 0b111,
    /// unknown
    unknown,
}

#[derive(Debug, Copy, Clone)]
#[allow(non_camel_case_types)]
/// The time constant of IIR filter
pub enum Filter {
    /// off
    off = 0x00,
    /// c2
    c2 = 0x01,
    /// c4
    c4 = 0x02,
    /// c8
    c8 = 0x03,
    /// c16
    c16 = 0x04,
    /// unknown
    unknown,
}

/// Configuration register, sets the rate, filter and interface options
/// of the device. Note that writing to this register while device in normal
/// mode may be ignored. Writes in sleep mode are not ignored.
///
/// spi3w_en is intentionally left out of this implementation.
#[derive(Debug, Copy, Clone)]
pub struct Config {
    /// Controls inactive duration in normal mode
    pub t_sb: Standby,
    /// Controls the time constant of IIR filter
    pub filter: Filter,
}

#[derive(Debug, Copy, Clone)]
pub struct SensorStatus {
    /// measuring
    measuring: bool,
    /// im update
    im_update: bool,
}

#[derive(Debug, Copy, Clone)]
#[allow(non_camel_case_types)]
/// Oversampling
pub enum Oversampling {
    /// skipped
    skipped = 0b000,
    /// x1
    x1 = 0b001,
    /// x2
    x2 = 0b010,
    /// x4
    x4 = 0b011,
    /// x8
    x8 = 0b100,
    /// x16
    x16 = 0b101,
}

#[derive(Debug, Copy, Clone)]
/// PowerMode
pub enum PowerMode {
    /// Sleep
    Sleep = 0b00,
    /// Forced
    Forced = 0b01,
    /// Normal
    Normal = 0b11,
}
