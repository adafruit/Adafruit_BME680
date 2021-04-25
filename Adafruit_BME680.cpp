/*!
 * @file Adafruit_BME680.cpp
 *
 * @mainpage Adafruit BME680 temperature, humidity, barometric pressure and gas
 * sensor driver
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's BME680 driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit BME680 breakout: https://www.adafruit.com/products/3660
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section author Author
 *
 * Written by Ladyada for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "Adafruit_BME680.h"
#include "Arduino.h"

//#define BME680_DEBUG

/** SPI object **/
SPIClass *_spi = NULL;

/** These SPI pins must be global in order to work with underlying library **/
int8_t _BME68X_SoftwareSPI_MOSI; ///< Global SPI MOSI pin
int8_t _BME68X_SoftwareSPI_MISO; ///< Global SPI MISO pin
int8_t _BME68X_SoftwareSPI_SCK;  ///< Globak SPI Clock pin

/** Our hardware interface functions **/
static int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data,
                       uint32_t len, void *interface);
static int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data,
                        uint32_t len, void *interface);
static int8_t spi_read(uint8_t reg_addr, uint8_t *reg_data,
                       uint32_t len, void *interface);
static int8_t spi_write(uint8_t reg_addr, const uint8_t *reg_data,
                        uint32_t len, void *interface);
static uint8_t spi_transfer(uint8_t x);
static void delay_usec(uint32_t us, void *intf_ptr);

// PUBLIC FUNCTIONS

/*!
 *  @brief  Instantiates sensor with i2c.
 *  @param  *theWire
 *          optional Wire object
 */
Adafruit_BME680::Adafruit_BME680(TwoWire *theWire)
    : _cs(-1), _meas_start(0), _meas_period(0) {
  _wire = theWire;
  _BME68X_SoftwareSPI_MOSI = -1;
  _BME68X_SoftwareSPI_MISO = -1;
  _BME68X_SoftwareSPI_SCK = -1;
}

/*!
 *  @brief  Instantiates sensor with Hardware SPI.
 *  @param  cspin
 *          SPI chip select.
 *  @param  theSPI
 *          optional SPI object
 */
Adafruit_BME680::Adafruit_BME680(int8_t cspin, SPIClass *theSPI)
    : _cs(cspin), _meas_start(0), _meas_period(0) {
  _spi = theSPI;
  _BME68X_SoftwareSPI_MOSI = -1;
  _BME68X_SoftwareSPI_MISO = -1;
  _BME68X_SoftwareSPI_SCK = -1;
}

/*!
 *  @brief  Instantiates sensor with Software (bit-bang) SPI.
 *  @param  cspin
 *          SPI chip select
 *  @param  mosipin
 *          SPI MOSI (Data from microcontroller to sensor)
 *  @param  misopin
 *          SPI MISO (Data to microcontroller from sensor)
 *  @param  sckpin
 *          SPI Clock
 */
Adafruit_BME680::Adafruit_BME680(int8_t cspin, int8_t mosipin, int8_t misopin,
                                 int8_t sckpin)
    : _cs(cspin), _meas_start(0), _meas_period(0) {
  _BME68X_SoftwareSPI_MOSI = mosipin;
  _BME68X_SoftwareSPI_MISO = misopin;
  _BME68X_SoftwareSPI_SCK = sckpin;
}

/*!
 *  @brief  Initializes the sensor
 *          Hardware ss initialized, verifies it is in the I2C or SPI bus, then
 * reads calibration data in preparation for sensor reads.
 *  @param  addr
 *          Optional parameter for the I2C address of BME680. Default is 0x77
 *  @param  initSettings
 *          Optional parameter for initializing the sensor settings.
 *          Default is true.
 *  @return True on sensor initialization success. False on failure.
 */
bool Adafruit_BME680::begin(uint8_t addr, bool initSettings) {
  int8_t rslt;
 
  if (_cs == -1) {    // i2c
    if (_i2cdev) {
      delete _i2cdev;
    }
    _i2cdev = new Adafruit_I2CDevice(addr, _wire);
    if (! _i2cdev->begin()) {
      return false;
    }

    gas_sensor.chip_id = addr;
    gas_sensor.intf = BME68X_I2C_INTF;
    gas_sensor.intf_ptr = (void *)_i2cdev;
    gas_sensor.read = &i2c_read;
    gas_sensor.write = &i2c_write;

  } else {
    digitalWrite(_cs, HIGH);
    pinMode(_cs, OUTPUT);

    if (_BME68X_SoftwareSPI_SCK == -1) {
      // hardware SPI
      _spi->begin();
    } else {
      // software SPI
      pinMode(_BME68X_SoftwareSPI_SCK, OUTPUT);
      pinMode(_BME68X_SoftwareSPI_MOSI, OUTPUT);
      pinMode(_BME68X_SoftwareSPI_MISO, INPUT);
    }

    gas_sensor.chip_id = _cs;
    gas_sensor.intf = BME68X_SPI_INTF;
    gas_sensor.intf_ptr = NULL;
    gas_sensor.read = &spi_read;
    gas_sensor.write = &spi_write;
  }

  gas_sensor.amb_temp = 25; /* The ambient temperature in deg C is used for defining the heater temperature */
  gas_sensor.delay_us = delay_usec;

  rslt = bme68x_init(&gas_sensor);
#ifdef BME680_DEBUG
  Serial.print(F("Init Result: "));
  Serial.println(rslt);
#endif

  if (rslt != BME68X_OK)
    return false;

#ifdef BME680_DEBUG
  Serial.print("T1 = ");
  Serial.println(gas_sensor.calib.par_t1);
  Serial.print("T2 = ");
  Serial.println(gas_sensor.calib.par_t2);
  Serial.print("T3 = ");
  Serial.println(gas_sensor.calib.par_t3);
  Serial.print("P1 = ");
  Serial.println(gas_sensor.calib.par_p1);
  Serial.print("P2 = ");
  Serial.println(gas_sensor.calib.par_p2);
  Serial.print("P3 = ");
  Serial.println(gas_sensor.calib.par_p3);
  Serial.print("P4 = ");
  Serial.println(gas_sensor.calib.par_p4);
  Serial.print("P5 = ");
  Serial.println(gas_sensor.calib.par_p5);
  Serial.print("P6 = ");
  Serial.println(gas_sensor.calib.par_p6);
  Serial.print("P7 = ");
  Serial.println(gas_sensor.calib.par_p7);
  Serial.print("P8 = ");
  Serial.println(gas_sensor.calib.par_p8);
  Serial.print("P9 = ");
  Serial.println(gas_sensor.calib.par_p9);
  Serial.print("P10 = ");
  Serial.println(gas_sensor.calib.par_p10);
  Serial.print("H1 = ");
  Serial.println(gas_sensor.calib.par_h1);
  Serial.print("H2 = ");
  Serial.println(gas_sensor.calib.par_h2);
  Serial.print("H3 = ");
  Serial.println(gas_sensor.calib.par_h3);
  Serial.print("H4 = ");
  Serial.println(gas_sensor.calib.par_h4);
  Serial.print("H5 = ");
  Serial.println(gas_sensor.calib.par_h5);
  Serial.print("H6 = ");
  Serial.println(gas_sensor.calib.par_h6);
  Serial.print("H7 = ");
  Serial.println(gas_sensor.calib.par_h7);
  Serial.print("G1 = ");
  Serial.println(gas_sensor.calib.par_gh1);
  Serial.print("G2 = ");
  Serial.println(gas_sensor.calib.par_gh2);
  Serial.print("G3 = ");
  Serial.println(gas_sensor.calib.par_gh3);
  Serial.print("G1 = ");
  Serial.println(gas_sensor.calib.par_gh1);
  Serial.print("G2 = ");
  Serial.println(gas_sensor.calib.par_gh2);
  Serial.print("G3 = ");
  Serial.println(gas_sensor.calib.par_gh3);
  Serial.print("Heat Range = ");
  Serial.println(gas_sensor.calib.res_heat_range);
  Serial.print("Heat Val = ");
  Serial.println(gas_sensor.calib.res_heat_val);
  Serial.print("SW Error = ");
  Serial.println(gas_sensor.calib.range_sw_err);
#endif

  if (initSettings) {
    setIIRFilterSize(BME68X_FILTER_SIZE_3);
    setODR(BME68X_ODR_NONE);
    setHumidityOversampling(BME68X_OS_2X);
    setPressureOversampling(BME68X_OS_4X);
    setTemperatureOversampling(BME68X_OS_8X);
    setGasHeater(320, 150); // 320*C for 150 ms
  } else {
    setGasHeater(0, 0);
  }
  // don't do anything till we request a reading
  rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &gas_sensor);
 
#ifdef BME680_DEBUG
  Serial.print(F("Opmode Result: "));
  Serial.println(rslt);
#endif

  if (rslt != BME68X_OK)
    return false;

  return true;
}

/*!
 *  @brief  Performs a reading and returns the ambient temperature.
 *  @return Temperature in degrees Centigrade
 */
float Adafruit_BME680::readTemperature(void) {
  performReading();
  return temperature;
}

/*!
 *  @brief Performs a reading and returns the barometric pressure.
 *  @return Barometic pressure in Pascals
 */
float Adafruit_BME680::readPressure(void) {
  performReading();
  return pressure;
}

/*!
 *  @brief  Performs a reading and returns the relative humidity.
 *  @return Relative humidity as floating point
 */
float Adafruit_BME680::readHumidity(void) {
  performReading();
  return humidity;
}

/*!
 *  @brief Calculates the resistance of the MOX gas sensor.
 *  @return Resistance in Ohms
 */
uint32_t Adafruit_BME680::readGas(void) {
  performReading();
  return gas_resistance;
}

/*!
 *  @brief  Calculates the altitude (in meters).
 *          Reads the current atmostpheric pressure (in hPa) from the sensor and
 * calculates via the provided sea-level pressure (in hPa).
 *  @param  seaLevel
 *          Sea-level pressure in hPa
 *  @return Altitude in meters
 */
float Adafruit_BME680::readAltitude(float seaLevel) {
  // Equation taken from BMP180 datasheet (page 16):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude. See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

  float atmospheric = readPressure() / 100.0F;
  return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}

/*!
 *  @brief  Performs a full reading of all 4 sensors in the BME680.
 *          Assigns the internal Adafruit_BME680#temperature,
 * Adafruit_BME680#pressure, Adafruit_BME680#humidity and
 * Adafruit_BME680#gas_resistance member variables
 *  @return True on success, False on failure
 */
bool Adafruit_BME680::performReading(void) { 
  return endReading();
}

/*! @brief Begin an asynchronous reading.
 *  @return When the reading would be ready as absolute time in millis().
 */
uint32_t Adafruit_BME680::beginReading(void) {
  if (_meas_start != 0) {
    /* A measurement is already in progress */
    return _meas_start + _meas_period;
  }

  int8_t rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &gas_sensor);
#ifdef BME680_DEBUG
  Serial.print(F("Opmode Result: "));
  Serial.println(rslt);
#endif
  if (rslt != BME68X_OK)
    return false;

  /* Calculate delay period in microseconds */
  uint32_t delayus_period = (uint32_t)bme68x_get_meas_dur(BME68X_FORCED_MODE, &gas_conf, &gas_sensor) + ((uint32_t)gas_heatr_conf.heatr_dur * 1000);
  //Serial.print("measure: "); Serial.println(bme68x_get_meas_dur(BME68X_FORCED_MODE, &gas_conf, &gas_sensor));
  //Serial.print("heater: "); Serial.println((uint32_t)gas_heatr_conf.heatr_dur * 1000);

  _meas_start = millis();
  _meas_period = delayus_period / 1000;

  return _meas_start + _meas_period;
}

/*! @brief  End an asynchronous reading.
 *          If the asynchronous reading is still in progress, block until it
 * ends. If no asynchronous reading has started, this is equivalent to
 * performReading().
 *  @return Whether success.
 */
bool Adafruit_BME680::endReading(void) {
  uint32_t meas_end = beginReading();

  if (meas_end == 0) {
    return false;
  }

  int remaining_millis = remainingReadingMillis();

  if (remaining_millis > 0) {
#ifdef BME680_DEBUG
    Serial.print(F("Waiting (ms) "));
    Serial.println(remaining_millis);
#endif
    delay(static_cast<unsigned int>(remaining_millis) *
          2); /* Delay till the measurement is ready */
  }
  _meas_start = 0; /* Allow new measurement to begin */
  _meas_period = 0;

#ifdef BME680_DEBUG
  Serial.print(F("t_fine = "));
  Serial.println(gas_sensor.calib.t_fine);
#endif

  struct bme68x_data data;
  int8_t n_fields;

#ifdef BME680_DEBUG
  Serial.println(F("Getting sensor data"));
#endif

  int8_t rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &gas_sensor);
#ifdef BME680_DEBUG
  Serial.print(F("GetData Result: "));
  Serial.println(rslt);
#endif
  if (rslt != BME68X_OK)
    return false;

  if (n_fields) {
    temperature = data.temperature;
    humidity = data.humidity;
    pressure = data.pressure;

#ifdef BME680_DEBUG
    Serial.print(F("data.status 0x"));
    Serial.println(data.status, HEX);
#endif
    if (data.status & (BME68X_HEAT_STAB_MSK | BME68X_GASM_VALID_MSK)) {
      //Serial.print("Gas resistance: "); Serial.println(data.gas_resistance);
      gas_resistance = data.gas_resistance;
    } else {
      gas_resistance = 0;
      //Serial.println("Gas reading unstable!");
    }
  }

  return true;
}

/*! @brief  Get remaining time for an asynchronous reading.
 *          If the asynchronous reading is still in progress, how many millis
 * until its completion. If the asynchronous reading is completed, 0. If no
 * asynchronous reading has started, -1 or
 * Adafruit_BME680::reading_not_started. Does not block.
 *  @return Remaining millis until endReading will not block if invoked.
 */
int Adafruit_BME680::remainingReadingMillis(void) {
  if (_meas_start != 0) {
    /* A measurement is already in progress */
    int remaining_time = (int)_meas_period - (millis() - _meas_start);
    return remaining_time < 0 ? reading_complete : remaining_time;
  }
  return reading_not_started;
}

/*!
 *  @brief  Enable and configure gas reading + heater
 *  @param  heaterTemp
 *          Desired temperature in degrees Centigrade
 *  @param  heaterTime
 *          Time to keep heater on in milliseconds
 *  @return True on success, False on failure
 */
bool Adafruit_BME680::setGasHeater(uint16_t heaterTemp, uint16_t heaterTime) {

  if ((heaterTemp == 0) || (heaterTime == 0)) {
    gas_heatr_conf.enable = BME68X_DISABLE;
  } else {
    gas_heatr_conf.enable = BME68X_ENABLE;
    gas_heatr_conf.heatr_temp = heaterTemp;
    gas_heatr_conf.heatr_dur = heaterTime;
  }

  int8_t rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &gas_heatr_conf, &gas_sensor);
#ifdef BME680_DEBUG
  Serial.print(F("SetHeaterConf Result: "));
  Serial.println(rslt);
#endif
  return rslt == 0;
}


bool Adafruit_BME680::setODR(uint8_t odr) {
  if (odr > BME68X_ODR_NONE)
    return false;

  gas_conf.odr = odr;

  int8_t rslt = bme68x_set_conf(&gas_conf, &gas_sensor);
#ifdef BME680_DEBUG
  Serial.print(F("SetConf Result: "));
  Serial.println(rslt);
#endif
  return rslt == 0;
}


/*!
 *  @brief  Setter for Temperature oversampling
 *  @param  oversample
 *          Oversampling setting, can be BME68X_OS_NONE (turn off Temperature
 * reading), BME68X_OS_1X, BME68X_OS_2X, BME68X_OS_4X, BME68X_OS_8X or
 * BME68X_OS_16X
 *  @return True on success, False on failure
 */

bool Adafruit_BME680::setTemperatureOversampling(uint8_t oversample) {
  if (oversample > BME68X_OS_16X)
    return false;

  gas_conf.os_temp = oversample;

  int8_t rslt = bme68x_set_conf(&gas_conf, &gas_sensor);
#ifdef BME680_DEBUG
  Serial.print(F("SetConf Result: "));
  Serial.println(rslt);
#endif
  return rslt == 0;
}

/*!
 *  @brief  Setter for Humidity oversampling
 *  @param  oversample
 *          Oversampling setting, can be BME68X_OS_NONE (turn off Humidity
 * reading), BME68X_OS_1X, BME68X_OS_2X, BME68X_OS_4X, BME68X_OS_8X or
 * BME68X_OS_16X
 *  @return True on success, False on failure
 */
bool Adafruit_BME680::setHumidityOversampling(uint8_t oversample) {
  if (oversample > BME68X_OS_16X)
    return false;
  
  gas_conf.os_hum = oversample;

  int8_t rslt = bme68x_set_conf(&gas_conf, &gas_sensor);
#ifdef BME680_DEBUG
  Serial.print(F("SetConf Result: "));
  Serial.println(rslt);
#endif
  return rslt == 0;
}

/*!
 *  @brief  Setter for Pressure oversampling
 *  @param  oversample
 *          Oversampling setting, can be BME68X_OS_NONE (turn off Pressure
 * reading), BME68X_OS_1X, BME68X_OS_2X, BME68X_OS_4X, BME68X_OS_8X or
 * BME68X_OS_16X
 *  @return True on success, False on failure
 */
bool Adafruit_BME680::setPressureOversampling(uint8_t oversample) {
  if (oversample > BME68X_OS_16X)
    return false;

  gas_conf.os_pres = oversample;

  int8_t rslt = bme68x_set_conf(&gas_conf, &gas_sensor);
#ifdef BME680_DEBUG
  Serial.print(F("SetConf Result: "));
  Serial.println(rslt);
#endif
  return rslt == 0;
}

/*!
 *  @brief  Setter for IIR filter.
 *  @param  filtersize
 *          Size of the filter (in samples).
 *          Can be BME68X_FILTER_SIZE_0 (no filtering), BME68X_FILTER_SIZE_1,
 * BME68X_FILTER_SIZE_3, BME68X_FILTER_SIZE_7, BME68X_FILTER_SIZE_15,
 * BME68X_FILTER_SIZE_31, BME68X_FILTER_SIZE_63, BME68X_FILTER_SIZE_127
 *  @return True on success, False on failure
 */
bool Adafruit_BME680::setIIRFilterSize(uint8_t filtersize) {
  if (filtersize > BME68X_FILTER_SIZE_127)
    return false;
  gas_conf.filter = filtersize;

  int8_t rslt = bme68x_set_conf(&gas_conf, &gas_sensor);
#ifdef BME680_DEBUG
  Serial.print(F("SetConf Result: "));
  Serial.println(rslt);
#endif
  return rslt == 0;
}

/*!
 *  @brief  Reads 8 bit values over I2C
 */
int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf) {

  Adafruit_I2CDevice *_dev = (Adafruit_I2CDevice *)intf;

  if (! _dev->write_then_read(&reg_addr, 1, reg_data, len, true)) {
    return -1;
  }

  return 0;
}

/*!
 *  @brief  Writes 8 bit values over I2C
 */
int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf) {
  Adafruit_I2CDevice *_dev = (Adafruit_I2CDevice *)intf;

  if (! _dev->write(reg_data, len, true, &reg_addr, 1)) {
    return -1;
  }
  return 0;
}

/*!
 *  @brief  Reads 8 bit values over SPI
 */
static int8_t spi_read(uint8_t reg_addr, uint8_t *reg_data,
                       uint32_t len, void *intf_ptr) {
  /*
#ifdef BME680_DEBUG
  Serial.print("\tSPI $");
  Serial.print(reg_addr, HEX);
  Serial.print(" => ");
#endif

  // If hardware SPI we should use transactions!
  if (_BME68X_SoftwareSPI_SCK == -1) {
    _spi->beginTransaction(
        SPISettings(BME68X_DEFAULT_SPIFREQ, MSBFIRST, SPI_MODE0));
  }

  digitalWrite(cspin, LOW);

  spi_transfer(reg_addr | 0x80);

  while (len--) {
    *reg_data = spi_transfer(0x00);
#ifdef BME680_DEBUG
    Serial.print("0x");
    Serial.print(*reg_data, HEX);
    Serial.print(", ");
#endif
    reg_data++;
  }

  digitalWrite(cspin, HIGH);

  if (_BME68X_SoftwareSPI_SCK == -1) {
    _spi->endTransaction();
  }

#ifdef BME680_DEBUG
  Serial.println("");
#endif
*/
  return 0;
}

/*!
 *  @brief  Writes 8 bit values over SPI
 */
static int8_t spi_write(uint8_t reg_addr, const uint8_t *reg_data,
                        uint32_t len, void *intf_ptr) {
  /*

#ifdef BME680_DEBUG
  Serial.print("\tSPI $");
  Serial.print(reg_addr, HEX);
  Serial.print(" <= ");
#endif

  // If hardware SPI we should use transactions!
  if (_BME68X_SoftwareSPI_SCK == -1) {
    _spi->beginTransaction(
        SPISettings(BME68X_DEFAULT_SPIFREQ, MSBFIRST, SPI_MODE0));
  }

  digitalWrite(cspin, LOW);

  spi_transfer(reg_addr);
  while (len--) {
    spi_transfer(*reg_data);
#ifdef BME680_DEBUG
    Serial.print("0x");
    Serial.print(*reg_data, HEX);
    Serial.print(", ");
#endif
    reg_data++;
  }

  digitalWrite(cspin, HIGH);

  if (_BME68X_SoftwareSPI_SCK == -1) {
    _spi->endTransaction();
  }

#ifdef BME680_DEBUG
  Serial.println("");
#endif
*/
  return 0;
}

static uint8_t spi_transfer(uint8_t x) {
  if (_BME68X_SoftwareSPI_SCK == -1)
    return _spi->transfer(x);

  // software spi
  // Serial.println("Software SPI");
  uint8_t reply = 0;
  for (int i = 7; i >= 0; i--) {
    reply <<= 1;
    digitalWrite(_BME68X_SoftwareSPI_SCK, LOW);
    digitalWrite(_BME68X_SoftwareSPI_MOSI, x & (1 << i));
    digitalWrite(_BME68X_SoftwareSPI_SCK, HIGH);
    if (digitalRead(_BME68X_SoftwareSPI_MISO))
      reply |= 1;
  }
  return reply;
}

static void delay_usec(uint32_t us, void *intf_ptr) { delayMicroseconds(us); yield(); }
