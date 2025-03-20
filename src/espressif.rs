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
    GenericReadingsResult, Readings, Sensor, SensorError, SensorResult, SensorT, SensorType,
    TypedReadingsResult,
};

use core::ffi::c_void;

use micro_rdk::esp32::esp_idf_svc::sys::bme280::*;

static mut i2c_bus: i2c_bus_handle_t = std::ptr::null_mut() as *mut c_void;
static mut bme280: bme280_handle_t = std::ptr::null_mut() as *mut c_void;

pub fn register_models(registry: &mut ComponentRegistry) -> Result<(), RegistryError> {
    registry.register_sensor("bme280", &Bme280::from_config)
}

#[derive(DoCommand)]
pub struct Bme280 {}

impl Status for Bme280 {
    fn get_status(&self) -> Result<Option<micro_rdk::google::protobuf::Struct>, StatusError> {
        let fields = HashMap::new();
        Ok(Some(micro_rdk::google::protobuf::Struct { fields }))
    }
}

impl Sensor for Bme280 {}

impl Readings for Bme280 {
    fn get_generic_readings(&mut self) -> Result<GenericReadingsResult, SensorError> {
        Ok(self
            .get_readings()?
            .into_iter()
            .map(|v| (v.0, SensorResult::<f64> { value: v.1 }.into()))
            .collect())
    }
}

impl SensorT<f64> for Bme280 {
    fn get_readings(&self) -> Result<TypedReadingsResult<f64>, SensorError> {
        //log::debug!("free-heap sensor - get readings called");
        let reading = unsafe { esp_get_free_heap_size() };
        let mut x = HashMap::new();
        x.insert("bytes".to_string(), reading as f64);
        //log::debug!("free-heap sensor - get readings OK");
        Ok(x)
    }
}

impl Bme280 {
    pub fn from_config(cfg: ConfigType, deps: Vec<Dependency>) -> Result<SensorType, SensorError> {
        let board = micro_rdk::common::registry::get_board_from_dependencies(deps);
        if board.is_none() {
            return Err(SensorError::ConfigError("BME280 missing board attribute"));
        }
        let board = board.unwrap();
        let i2c_handle: I2cHandleType;
        if let Ok(i2c_name) = cfg.get_attribute::<String>("i2c_bus") {
            i2c_handle = board.get_i2c_by_name(i2c_name)?;
        } else {
            return Err(SensorError::ConfigError("BME280 missing i2c_bus attribute"));
        }

        let _config = i2c_config_t {
            mode: I2C_MODE_MASTER,
            sda_io_num: 21,
            sda_pullup_en: GPIO_PULLUP_ENABLE,
            scl_io_num: 22,
            scl_pullup_en: GPIO_PULLUP_ENABLE,
            __bindgen_anon_1: i2c_config_t__bindgen_ty_1 {
                master: i2c_config_t__bindgen_ty_1__bindgen_ty_1 { clk_speed: 0 },
            },
            clk_flags: 0,
        };

        // use i2c_handle.name() to activate bus using esp-idf component

        /*

        //Step1: Init I2C bus
        i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = I2C_MASTER_SDA_IO,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_io_num = I2C_MASTER_SCL_IO,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = I2C_MASTER_FREQ_HZ,
        };
        i2c_bus = i2c_bus_create(I2C_MASTER_NUM, &conf);

        //Step2: Init bme280
        bme280 = bme280_create(i2c_bus, BME280_I2C_ADDRESS_DEFAULT);
        bme280_default_init(bme280);

        //Step3: Read temperature, humidity and pressure
        float temperature = 0.0, humidity = 0.0, pressure = 0.0;
        bme280_read_temperature(bme280, &temperature);
        bme280_read_humidity(bme280, &humidity);
        bme280_read_pressure(bme280, &pressure);
        }*/
    }
}
