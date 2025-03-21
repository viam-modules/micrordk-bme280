use micro_rdk::common::board::Board;
use micro_rdk::common::config::ConfigType;
use micro_rdk::common::i2c::{I2CHandle, I2cHandleType};
use micro_rdk::common::registry::{ComponentRegistry, Dependency, RegistryError};
use micro_rdk::common::status::{Status, StatusError};
use micro_rdk::esp32::esp_idf_svc::hal::sys::esp;
use micro_rdk::DoCommand;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use micro_rdk::common::sensor::{
    GenericReadingsResult, Readings, Sensor, SensorError, SensorResult, SensorT, SensorType,
    TypedReadingsResult,
};

use micro_rdk::esp32::esp_idf_svc::sys::bme280::*;

static mut I2C_BUS: i2c_bus_handle_t = std::ptr::null_mut();
static mut BME280: bme280_handle_t = std::ptr::null_mut();

//typedef enum{
//#if SOC_I2C_SUPPORT_SLAVE
//    I2C_MODE_SLAVE = 0,   /*!< I2C slave mode */
//#endif
//    I2C_MODE_MASTER,      /*!< I2C master mode */
//    I2C_MODE_MAX,
//} i2c_mode_t;
const I2C_MODE_MASTER: u32 = 0x1;
const I2C_BUS_NO: i32 = 0;

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
        let mut temperature: f32 = 0.0;
        let mut pressure: f32 = 0.0;
        unsafe {
            log::info!("bme280 - reading temp...");
            let r = bme280_read_temperature(BME280, &mut temperature as &mut f32);
            log::warn!("res of reading: {}", r);
            //bme280_read_humidity(bme280, &humidity);
            log::info!("bme280 - reading pressure...");
            let r = bme280_read_pressure(BME280, &mut pressure as &mut f32);
            log::warn!("res of reading: {}", r);
        }
        log::info!("temp: {} - pressure: {}", temperature, pressure);
        let mut x = HashMap::new();
        x.insert("temp_celsius".to_string(), temperature.into());
        x.insert("pressure".to_string(), pressure.into());
        Ok(x)
    }
}

impl Bme280 {
    pub fn from_config(cfg: ConfigType, deps: Vec<Dependency>) -> Result<SensorType, SensorError> {
        // DO NOT use the board i2c interface to initialize the i2c bus, the idf-component will handle it
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
                }
        */

        let config = i2c_config_t {
            mode: I2C_MODE_MASTER,
            sda_io_num: 21,
            sda_pullup_en: true,
            scl_io_num: 22,
            scl_pullup_en: true,
            __bindgen_anon_1: i2c_config_t__bindgen_ty_1 {
                master: i2c_config_t__bindgen_ty_1__bindgen_ty_1 { clk_speed: 100000 },
            },
            clk_flags: 0,
        };

        unsafe {
            log::info!("initializing I2C_BUS: {:?}", I2C_BUS);
            I2C_BUS = i2c_bus_create(I2C_BUS_NO, &config);
            log::info!("initialized I2C_BUS to : {:?}", I2C_BUS);
            log::info!("initializing BME280: {:?}", BME280);
            BME280 = bme280_create(I2C_BUS, BME280_I2C_ADDRESS_DEFAULT);
            log::info!("initialized BME280 to : {:?}", BME280);
            esp!(bme280_default_init(BME280))?;
        }
        Ok(Arc::new(Mutex::new(Self {})))
    }
}
