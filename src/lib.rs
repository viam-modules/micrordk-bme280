use micro_rdk::{
    common::{
        config::ConfigType,
        registry::{ComponentRegistry, Dependency, RegistryError},
        sensor::{
            GenericReadingsResult, Readings, Sensor, SensorError, SensorResult, SensorT,
            SensorType, TypedReadingsResult,
        },
        status::{Status, StatusError},
    },
    esp32::esp_idf_svc::hal::sys::esp,
    DoCommand,
};
use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
};

// idf component re-exported from esp-idf-svc build system
use micro_rdk::esp32::esp_idf_svc::sys::bme280::*;

const I2C_MODE_MASTER: u32 = 0x1;

pub fn register_models(registry: &mut ComponentRegistry) -> Result<(), RegistryError> {
    registry.register_sensor("bme280", &Bme280::from_config)
}

#[derive(DoCommand)]
pub struct Bme280 {
    // handle to I2C bus
    i2c_handle: i2c_bus_handle_t,
    // handle to BME280 device
    driver_handle: bme280_handle_t, // = std::ptr::null_mut()
}

// Safety: The raw pointer handles are bound to the initialization and drop of this struct.
// Additionally, as the current implementation assumes ownership of the i2c bus, when used in isolation,
// there is no risk of another device manipulating the i2c and device handles.
unsafe impl Send for Bme280 {}

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
        let mut humidity: f32 = 0.0;
        unsafe {
            log::debug!("bme280 - reading temperature...");
            esp!(bme280_read_temperature(
                self.driver_handle,
                &mut temperature as &mut f32
            ))?;
            log::debug!("bme280 - reading pressure...");
            esp!(bme280_read_pressure(
                self.driver_handle,
                &mut pressure as &mut f32
            ))?;
            log::debug!("bme280 - reading humidity...");
            esp!(bme280_read_humidity(
                self.driver_handle,
                &mut humidity as &mut f32
            ))?;
        }
        log::debug!(
            "temperature: {}, humidity: {}, pressure: {}",
            temperature,
            humidity,
            pressure
        );
        let mut x: HashMap<String, f64> = HashMap::new();
        x.insert("temperature_c".to_string(), temperature.into());
        x.insert("humidity".to_string(), humidity.into());
        x.insert("pressure_hpa".to_string(), pressure.into());
        Ok(x)
    }
}

impl Bme280 {
    pub fn from_config(cfg: ConfigType, _deps: Vec<Dependency>) -> Result<SensorType, SensorError> {
        // DO NOT use the board i2c interface to initialize the i2c bus, the idf-component will handle it
        let bus_no = cfg
            .get_attribute::<i32>("i2c_bus")
            .inspect_err(|_| {
                log::warn!("`i2c_bus` attribute not found or invalid, defaulting to bus 0")
            })
            .unwrap_or_default();

        let sda_pin = cfg
            .get_attribute::<i32>("sda_pin")
            .inspect_err(|_| {
                log::info!("`sda_pin` attribute not found or invalid, defaulting to pin 22")
            })
            .unwrap_or(21);
        let scl_pin = cfg
            .get_attribute::<i32>("scl_pin")
            .inspect_err(|_| {
                log::info!("`scl_pin` attribute not found or invalid, defaulting to pin 21")
            })
            .unwrap_or(22);

        let config = i2c_config_t {
            mode: I2C_MODE_MASTER,
            sda_io_num: sda_pin,
            sda_pullup_en: true,
            scl_io_num: scl_pin,
            scl_pullup_en: true,
            __bindgen_anon_1: i2c_config_t__bindgen_ty_1 {
                master: i2c_config_t__bindgen_ty_1__bindgen_ty_1 { clk_speed: 100000 },
            },
            clk_flags: 0,
        };

        let mut sensor = Self {
            i2c_handle: std::ptr::null_mut(),
            driver_handle: std::ptr::null_mut(),
        };

        unsafe {
            log::info!("creating I2C_BUS...");

            sensor.i2c_handle = i2c_bus_create(bus_no, &config);
            if sensor.i2c_handle.is_null() {
                log::error!("failed to create i2c bus `{}`", bus_no);
                return Err(SensorError::ConfigError("failed to create i2c bus"));
            }
            log::info!("created I2C_BUS at : {:?}", sensor.i2c_handle);
            sensor.driver_handle = bme280_create(
                sensor.i2c_handle,
                BME280_I2C_ADDRESS_DEFAULT.try_into().unwrap(),
            );
            if sensor.driver_handle.is_null() {
                return Err(SensorError::ConfigError("failed to create BME280 device"));
            }
            log::info!("created BME280 at : {:?}", sensor.driver_handle);
            log::info!("initializing BME280...");
            if esp!(bme280_default_init(sensor.driver_handle)).is_err() {
                log::error!("failed to initialize BME280");
                return Err(SensorError::ConfigError(
                    "failed to initialize BME280 device",
                ));
            }
            log::info!("BME280 initialized successfully");
        }
        Ok(Arc::new(Mutex::new(sensor)))
    }
}

impl Drop for Bme280 {
    fn drop(&mut self) {
        unsafe {
            if !self.driver_handle.is_null() {
                log::debug!("deleting bme280");
                bme280_delete(&mut self.driver_handle as *mut _);
                log::debug!("device_handle value deleting: {:?}", self.driver_handle);
            }
            if !self.i2c_handle.is_null() {
                log::debug!("deleting I2C bus");
                i2c_bus_delete(&mut self.i2c_handle as *mut _);
                log::debug!("i2c_bus handle value deleting: {:?}", self.i2c_handle);
            }
        }
    }
}
