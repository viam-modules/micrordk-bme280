# BME280 module

The BME280 is as combined digital humidity, pressure and temperature sensor based on proven sensing principles. 
The sensor module is housed in an extremely compact metal-lid LGA package with a footprint of only 2.5 × 2.5 mm² with a height of 0.93 mm. 
Its small dimensions and its low power consumption allow the implementation in battery driven devices such as handsets, GPS modules or watches.

[Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf)

This module utilizes the [Viam Sensor Component](https://docs.viam.com/operate/reference/components/sensor/) API and acts as a wrapper around the [BME280 ESP-IDF Component](https://components.espressif.com/components/espressif/bme280).

## Configuration

This component utilizes the following configuration attributes:
- `i2c_bus`: The I2C bus initialized on the board (defaults to `0`)
- `scl_pin`: The `SCL` I2C clock pin (defaults to `22`)
- `sda_pin`: The `SDA` IC2 data pin (defaults to `21`)

> CAUTION: This component's implementation initializes the I2C bus directly and assumes it is the sole device with control of the bus. Do not use in conjunction with the [Viam Board Component](https://docs.viam.com/operate/reference/components/board/) APIs for I2C or other devices on the same bus.

```json
{
  "name": "my-bme280-sensor",
  "api": "rdk:component:sensor",
  "model": "bme280",
  "attributes": {
    "i2c_bus": 0,
	"scl_pin": 21,
	"sda_pin": 22,
  }
}
```
