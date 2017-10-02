/***************************************************************************
  This is a library for the BME680 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME680 Breakout
  ----> http://www.adafruit.com/products/2650

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_BME680.h"


/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/


Adafruit_BME680::Adafruit_BME680()
  : _cs(-1), _mosi(-1), _miso(-1), _sck(-1)
{ }

Adafruit_BME680::Adafruit_BME680(int8_t cspin)
  : _cs(cspin), _mosi(-1), _miso(-1), _sck(-1)
{ }

Adafruit_BME680::Adafruit_BME680(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin)
  : _cs(cspin), _mosi(mosipin), _miso(misopin), _sck(sckpin)
{ }


bool Adafruit_BME680::begin(uint8_t a) {
  _i2caddr = a;

  if (_cs == -1) {
    // i2c
    Wire.begin();
  } else {
    digitalWrite(_cs, HIGH);
    pinMode(_cs, OUTPUT);

    if (_sck == -1) {
      // hardware SPI
      SPI.begin();
    } else {
      // software SPI
      pinMode(_sck, OUTPUT);
      pinMode(_mosi, OUTPUT);
      pinMode(_miso, INPUT);
    }
  }

  for (uint8_t i=0; i<0x7F; i++) {
    read8(i);
  }


  if (read8(BME680_REGISTER_CHIPID) != 0x61)
    return false;

  readCoefficients();
  
  // set filter off
  uint8_t config = read8(BME680_PAGE1_FILTER__ADDR);
  config &= ~(0x7 << 2);
  config |= (BME680_FILTER_COEFF_OFF & 0x7) << 2;
  write8(BME680_REGISTER_CONFIG, config);

bme680_set_filter();//IIR filter is off
  //Set before CONTROL_meas (DS 5.4.3)
  write8(BME680_REGISTER_CONTROLHUMID, 0x05); //16x oversampling 

  write8(BME680_REGISTER_CONTROL, 0xB7); // 16x ovesampling, normal mode
  return true;
}

uint8_t Adafruit_BME680::spixfer(uint8_t x) {
  if (_sck == -1)
    return SPI.transfer(x);

  // software spi
  //Serial.println("Software SPI");
  uint8_t reply = 0;
  for (int i=7; i>=0; i--) {
    reply <<= 1;
    digitalWrite(_sck, LOW);
    digitalWrite(_mosi, x & (1<<i));
    digitalWrite(_sck, HIGH);
    if (digitalRead(_miso))
      reply |= 1;
  }
  return reply;
}

/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C/SPI
*/
/**************************************************************************/
void Adafruit_BME680::write8(byte reg, byte value)
{
  if (_cs == -1) {
    Wire.beginTransmission((uint8_t)_i2caddr);
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
    Wire.endTransmission();
  } else {
    if (_sck == -1)
      SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg & ~0x80); // write, bit 7 low
    spixfer(value);
    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      SPI.endTransaction();              // release the SPI bus
  }
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C
*/
/**************************************************************************/
uint8_t Adafruit_BME680::read8(byte reg)
{
  uint8_t value;

  //Serial.print("$"); Serial.print(reg, HEX); 
  if (_cs == -1) {
    Wire.beginTransmission((uint8_t)_i2caddr);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)_i2caddr, (byte)1);
    value = Wire.read();

  } else {
    if (_sck == -1)
      SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg | 0x80); // read, bit 7 high
    value = spixfer(0);
    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      SPI.endTransaction();              // release the SPI bus
  }

  //Serial.print(": 0x"); Serial.println(value, HEX);
  return value;
}


int8_t Adafruit_BME680::readS8(byte reg) {
  return (int8_t)read8(reg);
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit value over I2C
*/
/**************************************************************************/
uint16_t Adafruit_BME680::read16(byte reg)
{
  uint16_t value;

  if (_cs == -1) {
    Wire.beginTransmission((uint8_t)_i2caddr);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)_i2caddr, (byte)2);
    value = ( ((uint16_t)Wire.read()) << 8) | Wire.read();
  } else {
    if (_sck == -1)
      SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg | 0x80); // read, bit 7 high
    value = (spixfer(0) << 8) | spixfer(0);
    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      SPI.endTransaction();              // release the SPI bus
  }

  return value;
}

uint16_t Adafruit_BME680::read16_LE(byte reg) {
  uint16_t temp = read16(reg);
  return (temp >> 8) | (temp << 8);

}

/**************************************************************************/
/*!
    @brief  Reads a signed 16 bit value over I2C
*/
/**************************************************************************/
int16_t Adafruit_BME680::readS16(byte reg)
{
  return (int16_t)read16(reg);

}

int16_t Adafruit_BME680::readS16_LE(byte reg)
{
  return (int16_t)read16_LE(reg);

}


/**************************************************************************/
/*!
    @brief  Reads a 24 bit value over I2C
*/
/**************************************************************************/

uint32_t Adafruit_BME680::read24(byte reg)
{
  uint32_t value;

  if (_cs == -1) {
    Wire.beginTransmission((uint8_t)_i2caddr);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)_i2caddr, (byte)3);
    
    value = Wire.read();
    value <<= 8;
    value |= Wire.read();
    value <<= 8;
    value |= Wire.read();

  } else {
    if (_sck == -1)
      SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg | 0x80); // read, bit 7 high
    
    value = spixfer(0);
    value <<= 8;
    value |= spixfer(0);
    value <<= 8;
    value |= spixfer(0);

    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      SPI.endTransaction();              // release the SPI bus
  }

  return value;
}


/**************************************************************************/
/*!
    @brief  Reads the factory-set coefficients
*/
/**************************************************************************/
void Adafruit_BME680::readCoefficients(void)
{
    _bme680_calib.dig_T1 = read16_LE(BME680_REGISTER_DIG_T1);
    _bme680_calib.dig_T2 = readS16_LE(BME680_REGISTER_DIG_T2);
    _bme680_calib.dig_T3 = readS8(BME680_REGISTER_DIG_T3);

    _bme680_calib.dig_P1 = read16_LE(BME680_REGISTER_DIG_P1);
    _bme680_calib.dig_P2 = readS16_LE(BME680_REGISTER_DIG_P2);
    _bme680_calib.dig_P3 = readS8(BME680_REGISTER_DIG_P3);
    _bme680_calib.dig_P4 = readS16_LE(BME680_REGISTER_DIG_P4);
    _bme680_calib.dig_P5 = readS16_LE(BME680_REGISTER_DIG_P5);
    _bme680_calib.dig_P6 = readS8(BME680_REGISTER_DIG_P6);
    _bme680_calib.dig_P7 = readS8(BME680_REGISTER_DIG_P7);
    _bme680_calib.dig_P8 = readS16_LE(BME680_REGISTER_DIG_P8);
    _bme680_calib.dig_P9 = readS16_LE(BME680_REGISTER_DIG_P9);
    _bme680_calib.dig_P10 = read8(BME680_REGISTER_DIG_P10);

    _bme680_calib.dig_H1 = read16_LE(BME680_REGISTER_DIG_H1) >> 4;
    _bme680_calib.dig_H2 = read8(BME680_REGISTER_DIG_H2_MSB);
    _bme680_calib.dig_H2 <<= 4;
    _bme680_calib.dig_H2 |= (read8(BME680_REGISTER_DIG_H2_LSB) & 0x0F);
    _bme680_calib.dig_H3 = readS8(BME680_REGISTER_DIG_H3);
    _bme680_calib.dig_H4 = readS8(BME680_REGISTER_DIG_H4);
    _bme680_calib.dig_H5 = readS8(BME680_REGISTER_DIG_H5);
    _bme680_calib.dig_H6 = read8(BME680_REGISTER_DIG_H6);
    _bme680_calib.dig_H6 = readS8(BME680_REGISTER_DIG_H7);  


    _bme680_calib.dig_GH1 = readS8(BME680_REGISTER_DIG_GH1);
    _bme680_calib.dig_GH2 = readS16_LE(BME680_REGISTER_DIG_GH2);
    _bme680_calib.dig_GH3 = readS8(BME680_REGISTER_DIG_GH3);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
float Adafruit_BME680::readTemperature(void)
{
  uint32_t uncom_temperature_u32 = read24(BME680_REGISTER_TEMPDATA);
  uncom_temperature_u32 >>= 4;

  // compensate
  double data1_d, data2_d ;

  /* calculate x1 data */  
  data1_d  =
    ((((double)uncom_temperature_u32 / 16384.0)
      - ((double)_bme680_calib.dig_T1 / 1024.0))
     * ((double)_bme680_calib.dig_T2));

  /* calculate x2 data */
  data2_d  =
    (((((double)uncom_temperature_u32 / 131072.0) -
       ((double)_bme680_calib.dig_T1 / 8192.0)) *
      (((double)uncom_temperature_u32 / 131072.0) -
       ((double)_bme680_calib.dig_T1 / 8192.0))) *
     ((double)_bme680_calib.dig_T3 * 16.0));

  /* t fine value*/
  _bme680_calib.t_fine = (int32_t)(data1_d + data2_d);

  /* compensated temperature data*/
  comp_temperature  = ((data1_d + data2_d) /
		  5120.0);

  return comp_temperature;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
float Adafruit_BME680::readPressure(void) {
  int64_t var1, var2, p;

  readTemperature(); // must be done first to get t_fine

  uint32_t uncom_pressure_u32 = read24(BME680_REGISTER_PRESSUREDATA);
  uncom_pressure_u32 >>= 4;

  double data1_d, data2_d, data3_d, pressure_comp;

  data1_d = (((double)_bme680_calib.t_fine / 2.0) - 64000.0);
  data2_d = data1_d * data1_d *
    (((double)_bme680_calib.dig_P6) / (4.0 * 32768.0));
  data2_d = data2_d + (data1_d *
		       ((double)_bme680_calib.dig_P5) * 2.0);
  data2_d = (data2_d / 4.0) +
	(((double)_bme680_calib.dig_P4) * 65536.0);
  data1_d = (((((double)_bme680_calib.dig_P3 * data1_d
		* data1_d * 32.0) / 524288.0)
	      + ((double)_bme680_calib.dig_P2
		 * data1_d)) / 524288.0);
  data1_d = ((1.0 + (data1_d / 32768.0)) *
	     ((double)_bme680_calib.dig_P1));
  pressure_comp = (1048576.0 - ((double)uncom_pressure_u32));

  /* Avoid exception caused by division by zero */
  if (data1_d == 0) {
    return 0;
  }
    
  pressure_comp = (((pressure_comp - (data2_d / 4096.0))
		    * 6250.0) / data1_d);
  data1_d = (((double)_bme680_calib.dig_P9) *
	     pressure_comp * pressure_comp) /  2147483648.0;
  data2_d = pressure_comp *
    (((double)_bme680_calib.dig_P8) / 32768.0);
  data3_d = ((pressure_comp / 256.0) *
	     (pressure_comp / 256.0) *
	       (pressure_comp / 256.0) *
	     (_bme680_calib.dig_P10 / 131072.0));
  pressure_comp =
    (pressure_comp + (data1_d + data2_d + data3_d +
		      ((double)_bme680_calib.dig_P7 * 128.0)) / 16.0);
  return pressure_comp;
}


/**************************************************************************/
/*!

*/
/**************************************************************************/
float Adafruit_BME680::readHumidity(void) {

  readTemperature(); // must be done first to get t_fine & temp

  uint16_t uncom_humidity_u16 = read16(BME680_REGISTER_HUMIDDATA);

  double humidity_comp;
  double var1, var2, var3, var4;

  var1 = ((double)uncom_humidity_u16) - 
    (((double) _bme680_calib.dig_H1 * 16.0) +
     (((double)_bme680_calib.dig_H3 / 2.0)
      * comp_temperature));
  
  var2 = var1 * (((double) _bme680_calib.dig_H2 / 262144.0)
		 *(1.0 + (((double)_bme680_calib.dig_H4 / 16384.0)
			  * comp_temperature)
		   + (((double)_bme680_calib.dig_H5
		       / 1048576.0) * comp_temperature
		      * comp_temperature)));
  var3 = (double) _bme680_calib.dig_H6 / 16384.0;
  var4 = (double) _bme680_calib.dig_H7 / 2097152.0;
  
  humidity_comp = var2 +
    ((var3 + (var4 * comp_temperature)) * var2 * var2);
  if (humidity_comp > 100)
    humidity_comp = 100;
  else if (humidity_comp < 0)
    humidity_comp = 0;
  return humidity_comp;
}


float Adafruit_BME680::readGas(void)
{
  uint16_t gas_adc_u16;

  gas_adc_u16 = read16(BME680_REGISTER_FIELD0_GASDATA);
  gas_adc_u16 >>= 6;
  gas_adc_u16 &= 0x2FF;

  Serial.print("Gas ADC = "); Serial.println(gas_adc_u16);

  uint8_t gas_range_u8;
  gas_range_u8 = read8(BME680_REGISTER_FIELD0_GASRANGE) & 0xF;

  Serial.print("Gas Range = "); Serial.println(gas_range_u8);

  double gas_res_d = 0;

#ifdef RANGE_SWITCH_CORRECTION_ENABLE

  const double lookup_k1_range[BME680_GAS_RANGE_RL_LENGTH] = {
    1, 1, 1, 1, 1, 0.99, 1, 0.992, 1, 1, 0.998, 0.995, 1, 0.99, 1, 1};
  const double lookup_k2_range[BME680_GAS_RANGE_RL_LENGTH] = {
    1, 1, 1, 1, 1.001, 1.007, 1, 0.992, 0.999, 1, 1, 1, 1, 1, 1, 1};
  s8 range_switching_error_val = 0;
  double k1_dev = 0;
  double k1 = 0;
  double k2 = 0;
  
  /* check if sign bit is set */
  if ((_bme680_calib.range_switching_error & BME680_SIGN_BIT_MASK)) {
    range_switching_error_val =
      _bme680_calib.range_switching_error - 16;
  } else {
    range_switching_error_val =
      _bme680_calib.range_switching_error;
  }

  k1_dev = (1340.0 + (5.0 * range_switching_error_val));
  k1 = (k1_dev) * lookup_k1_range[gas_range_u8];
  k2 = lookup_k2_range[gas_range_u8];
  
  gas_res_d = 1.0 / (double)(k2 * ((0.000000125) *
				   (double)(1 << gas_range_u8)
				   * (((((double)gas_adc_u16) - 512.00) + k1) / k1)));

#else

  gas_res_d = 1.0 / ((0.000000125) *
		     (double)(1 << gas_range_u8) *
		     (((double)(gas_adc_u16)-512.00)+(1024.0 / 0.75)) / (1024.0/0.75));

#endif
  
  return gas_res_d;
}

/**************************************************************************/
/*!
    Calculates the altitude (in meters) from the specified atmospheric
    pressure (in hPa), and sea-level pressure (in hPa).

    @param  seaLevel      Sea-level pressure in hPa
    @param  atmospheric   Atmospheric pressure in hPa
*/
/**************************************************************************/
float Adafruit_BME680::readAltitude(float seaLevel)
{
  // Equation taken from BMP180 datasheet (page 16):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude.  See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

  float atmospheric = readPressure() / 100.0F;
  return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}
