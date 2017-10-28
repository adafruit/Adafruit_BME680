/***************************************************************************
  This is a library for the BME680 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME680 Breakout
  ----> http://www.adafruit.com/products/XXXX

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#ifndef __BME680_H__
#define __BME680_H__

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "bme680.h"


int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);


#define BME680_SIGN_BIT_MASK					(0x08)

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define BME680_DEFAULT_ADDRESS                (0x77)
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    enum
    {
      BME680_REGISTER_DIG_T1              = 0xE9,
      BME680_REGISTER_DIG_T2              = 0x8A,
      BME680_REGISTER_DIG_T3              = 0x8C,
      BME680_REGISTER_DIG_T4              = 0x8D,
      BME680_REGISTER_DIG_T5              = 0x93,

      BME680_REGISTER_DIG_P1              = 0x8E,
      BME680_REGISTER_DIG_P2              = 0x90,
      BME680_REGISTER_DIG_P3              = 0x92,
      BME680_REGISTER_DIG_P4              = 0x94,
      BME680_REGISTER_DIG_P5              = 0x96,
      BME680_REGISTER_DIG_P6              = 0x99,
      BME680_REGISTER_DIG_P7              = 0x98,
      BME680_REGISTER_DIG_P8              = 0x9C,
      BME680_REGISTER_DIG_P9              = 0x9E,
      BME680_REGISTER_DIG_P10             = 0xA0,
      BME680_REGISTER_DIG_P11             = 0x9A,
      BME680_REGISTER_DIG_P12             = 0x9B,
      BME680_REGISTER_DIG_P13             = 0xA1,

      BME680_REGISTER_DIG_H1              = 0xE2,
      BME680_REGISTER_DIG_H2_MSB          = 0xE1,
      BME680_REGISTER_DIG_H2_LSB          = 0xE2,
      BME680_REGISTER_DIG_H3              = 0xE4,
      BME680_REGISTER_DIG_H4              = 0xE5,
      BME680_REGISTER_DIG_H5              = 0xE6,
      BME680_REGISTER_DIG_H6              = 0xE7,
      BME680_REGISTER_DIG_H7              = 0xE8,

      BME680_REGISTER_DIG_GH1             = 0xED,
      BME680_REGISTER_DIG_GH2             = 0xEB,
      BME680_REGISTER_DIG_GH3             = 0xEE,

      BME680_REGISTER_CHIPID             = 0xD0,

      BME680_REGISTER_CONFIG       = 0x75,

      BME680_REGISTER_PRESSUREDATA       = 0xF7,
      BME680_REGISTER_TEMPDATA           = 0xFA,
      BME680_REGISTER_HUMIDDATA          = 0xFD,
      BME680_REGISTER_FIELD0_GASDATA          = 0x2A,
      BME680_REGISTER_FIELD0_GASRANGE          = 0x2B,
    };

#define BME680_FILTER_COEFF_OFF 0
/*=========================================================================*/

/*
class Adafruit_BME680_Unified : public Adafruit_Sensor
{
  public:
    Adafruit_BME680_Unified(int32_t sensorID = -1);

    bool  begin(uint8_t addr = BME680_ADDRESS);
    void  getTemperature(float *temp);
    void  getPressure(float *pressure);
    float pressureToAltitude(float seaLevel, float atmospheric, float temp);
    float seaLevelForAltitude(float altitude, float atmospheric, float temp);
    void  getEvent(sensors_event_t*);
    void  getSensor(sensor_t*);

  private:
    uint8_t   _i2c_addr;
    int32_t   _sensorID;
};

*/


class Adafruit_BME680
{
  public:
    Adafruit_BME680(int8_t cspin = -1);
    Adafruit_BME680(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin);

    bool  begin(uint8_t addr = BME680_DEFAULT_ADDRESS);
    float readTemperature(void);
    float readPressure(void);
    float readHumidity(void);
    float readGas(void);
    float readAltitude(float seaLevel);


    bool setTemperatureOversampling(uint8_t os);
    bool setPressureOversampling(uint8_t os);
    bool setHumidityOversampling(uint8_t os);
    bool setIIRFilterSize(uint8_t fs);
    bool setGasHeater(uint16_t heaterTemp, uint16_t heaterTime);

    bool performReading(void);

  private:

    bool      _filterEnabled, _tempEnabled, _humEnabled, _presEnabled, _gasEnabled;
    uint8_t   _i2caddr;
    int32_t   _sensorID;
    int8_t _cs, _mosi, _miso, _sck;

    uint8_t spixfer(uint8_t x);

    struct bme680_dev gas_sensor;
};

#endif
