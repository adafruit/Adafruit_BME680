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

#include <Adafruit_Sensor.h>
#include <Wire.h>


#define BME680_SIGN_BIT_MASK					(0x08)

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define BME680_ADDRESS                (0x77)
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

      BME680_REGISTER_CONFIG       = 0x75

      BME680_REGISTER_PRESSUREDATA       = 0xF7,
      BME680_REGISTER_TEMPDATA           = 0xFA,
      BME680_REGISTER_HUMIDDATA          = 0xFD,
      BME680_REGISTER_FIELD0_GASDATA          = 0x2A,
      BME680_REGISTER_FIELD0_GASRANGE          = 0x2B,
    };

#define BME680_FILTER_COEFF_OFF 0
/*=========================================================================*/

/*=========================================================================
    CALIBRATION DATA
    -----------------------------------------------------------------------*/
    typedef struct
    {
      uint16_t dig_T1;
      int16_t  dig_T2;
      int8_t  dig_T3;

      uint16_t dig_P1;
      int16_t  dig_P2;
      int8_t  dig_P3;
      int16_t  dig_P4;
      int16_t  dig_P5;
      int8_t  dig_P6;
      int8_t  dig_P7;
      int16_t  dig_P8;
      int16_t  dig_P9;
      uint8_t  dig_P10;

      uint16_t  dig_H1;
      uint16_t  dig_H2;
      int8_t    dig_H3;
      int8_t    dig_H4;
      int8_t    dig_H5;
      uint8_t   dig_H6;
      uint8_t   dig_H7;

      int8_t    dig_GH1;
      int16_t   dig_GH2;
      int8_t    dig_GH3;

      int32_t   t_fine;/**<calibration T_FINE data*/
      uint8_t	res_heat_range;/**<resistance calculation*/
      uint8_t	range_switching_error;/**<range switching error*/
      int8_t    res_heat_val;/**<resistance heat value*/

    } bme680_calib_data;
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
    Adafruit_BME680(void);
    Adafruit_BME680(int8_t cspin);
    Adafruit_BME680(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin);

    bool  begin(uint8_t addr = BME680_ADDRESS);
    float readTemperature(void);
    float readPressure(void);
    float readHumidity(void);
    float readGas(void);
    float readAltitude(float seaLevel);

  private:

    void readCoefficients(void);
    uint8_t spixfer(uint8_t x);

    void      write8(byte reg, byte value);
    uint8_t   read8(byte reg);
    int8_t    readS8(byte reg);
    uint16_t  read16(byte reg);
    uint32_t  read24(byte reg);
    int16_t   readS16(byte reg);
    uint16_t  read16_LE(byte reg); // little endian
    int16_t   readS16_LE(byte reg); // little endian

    uint8_t   _i2caddr;
    int32_t   _sensorID;
    double comp_temperature;

    int8_t _cs, _mosi, _miso, _sck;

    bme680_calib_data _bme680_calib;

};

#endif
