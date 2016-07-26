/**************************************************************************/
/* 
  This is an Arduino library for the BOSCH BMP085/BMP180 Barometric Pressure
  and Temperature sensor

  written by Enjoyneering79

  These sensor uses I2C to communicate, 2 pins are required to  
  interface

  Connect BMP085/BMP180 to pins:  SDA  SCL
  Uno, Mini, Pro:                 A4      A5
  Mega2560, Due:                  20      21
  Leonardo:                       2       3
  Atiny85:                        0/PWM   2/A1  (TinyWireM)
  NodeMCU 1.0:                    4/ANY   5/ANY (4 & 5 by default)
  ESP8266 ESP-01:                 ANY     ANY

  BSD license, all text above must be included in any redistribution
*/
 /**************************************************************************/

#include "BMP085_BMP180.h"


/**************************************************************************/
/*
    Constructor
*/
/**************************************************************************/
BMP085_BMP180::BMP085_BMP180(uint8_t res_mode)
{
  if (res_mode > BMP_ULTRAHIGHRES)
  {
    res_mode = BMP_ULTRAHIGHRES;
  }

  _BMP_Resolution = res_mode;
}

/**************************************************************************/
/*
    Initializes I2C and configures the sensor

    NOTE: call this function before doing anything else
*/
/**************************************************************************/
#if defined(ARDUINO_ARCH_ESP8266) || (ESP8266_NODEMCU)
bool BMP085_BMP180::begin(uint8_t sda, uint8_t scl)
{
  _sda = sda;
  _scl = scl;
   
  Wire.begin(_sda, _scl);
#else
bool BMP085_BMP180::begin(void) 
{
  Wire.begin();
#endif

  if (read8(BMP_ID) != BMP_CHIP_ID)
  {
    return false;
  }

  /* reads calibration data */
  _ac1 = read16(BMP_CAL_AC1);
  _ac2 = read16(BMP_CAL_AC2);
  _ac3 = read16(BMP_CAL_AC3);
  _ac4 = read16(BMP_CAL_AC4);
  _ac5 = read16(BMP_CAL_AC5);
  _ac6 = read16(BMP_CAL_AC6);

  _b1 = read16(BMP_CAL_B1);
  _b2 = read16(BMP_CAL_B2);

  _mb = read16(BMP_CAL_MB);
  _mc = read16(BMP_CAL_MC);
  _md = read16(BMP_CAL_MD);

  _BMP_Initialised = true;

  return true;
}

/**************************************************************************/
/*
    Computes B5
*/
/**************************************************************************/
int32_t BMP085_BMP180::computeB5(int32_t UT)
{
  int32_t X1 = (UT - (int32_t)_ac6) * ((int32_t)_ac5) >> 15;
  int32_t X2 = ((int32_t)_mc << 11) / (X1+(int32_t)_md);

  return X1 + X2;
}

/**************************************************************************/
/*
    Reads Raw Temperature Data
*/
/**************************************************************************/
uint16_t BMP085_BMP180::readRawTemperature(void)
{
  if (_BMP_Initialised != true)
  { 
  	#if defined(ARDUINO_ARCH_ESP8266) || (ESP8266_NODEMCU)
     begin(_sda, _scl);
    #else
     begin();
    #endif
  }

  write8(BMP_CONTROL, BMP_READTEMPCMD);
  delay(5);

  return read16(BMP_TEMPDATA);
}

/**************************************************************************/
/*
    Reads Raw Pressure Data
*/
/**************************************************************************/
uint32_t BMP085_BMP180::readRawPressure(void)
{
  uint32_t rawPressure;

  if (_BMP_Initialised != true)
  { 
  	#if defined(ARDUINO_ARCH_ESP8266) || (ESP8266_NODEMCU)
     begin(_sda, _scl);
    #else
     begin();
    #endif
  }
  
  write8(BMP_CONTROL, BMP_READPRESSURECMD + (_BMP_Resolution << 6));

  /* Set a delay for the calculation */
  switch (_BMP_Resolution)
  {
    case BMP_ULTRALOWPOWER:
      delay(5);
    break;
    case BMP_STANDARD:
      delay(8);
    break;
    case BMP_HIGHRES:
      delay(14);
    break;
    case BMP_ULTRAHIGHRES:
      delay(26);
    break;
   }

  rawPressure = read16(BMP_PRESSUREDATA);

  rawPressure <<= 8;
  rawPressure  |= read8(BMP_PRESSUREDATA + 2);
  rawPressure >>= (8 - _BMP_Resolution);

  return rawPressure;
}

/**************************************************************************/
/*
    Reads Pressure, Pa
*/
/**************************************************************************/
int32_t BMP085_BMP180::readPressure(void)
{
  int32_t  UT, UP, B3, B5, B6, X1, X2, X3, pressure;
  uint32_t B4, B7;

  UT = readRawTemperature();
  UP = readRawPressure();

  B5 = computeB5(UT);

  /* Pressure calculation */
  B6 = B5 - 4000;
  X1 = ((int32_t)_b2 * ( (B6 * B6)>>12 )) >> 11;
  X2 = ((int32_t)_ac2 * B6) >> 11;
  X3 = X1 + X2;
  B3 = ((((int32_t)_ac1 * 4 + X3) << _BMP_Resolution) + 2) / 4;

  X1 = ((int32_t)_ac3 * B6) >> 13;
  X2 = ((int32_t)_b1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = ((uint32_t)_ac4 * (uint32_t)(X3 + 32768)) >> 15;
  B7 = ((uint32_t)UP - B3) * (uint32_t)( 50000UL >> _BMP_Resolution );

  if (B7 < 0x80000000)
  {
    pressure = (B7 * 2) / B4;
  } 
  else 
  {
    pressure = (B7 / B4) * 2;
  }

  X1 = (pressure >> 8) * (pressure >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * pressure) >> 16;

  pressure = pressure + ((X1 + X2 + (int32_t)3791) >> 4);

  return pressure;
}

/**************************************************************************/
/*
    Calculates Sea Pressure referenced to current altitude, Pa

    NOTE: - current altitude in meters
*/
/**************************************************************************/
int32_t BMP085_BMP180::calculateSeaLevelPressure(float altitudeMeters)
{
  return (int32_t)(readPressure() / pow(1.0 - altitudeMeters / 44330, 5.255));
}

/**************************************************************************/
/*
    Reads Temperature, deg. C
*/
/**************************************************************************/
float BMP085_BMP180::readTemperature(void)
{
  int32_t UT, B5;   // following ds convention
  float temperature;

  UT = readRawTemperature();

  B5 = computeB5(UT);
  temperature = (B5+8) >> 4;
  temperature /= 10;
  
  return temperature;
}

/**************************************************************************/
/*
    Calculates Altitude referenced to pressure at sea level, meter

    NOTE: peressure at sea level = 101325 Pa (by default)
*/
/**************************************************************************/
float BMP085_BMP180::calculateAltitude(float seaLevelPressure)
{
  return 44330 * (1.0 - pow(readPressure() / seaLevelPressure, 0.1903));
}

/**************************************************************************/
/*
    Reads 8 bit value from the sensor register over I2C
*/
/**************************************************************************/
uint8_t BMP085_BMP180::read8(uint8_t reg)
{
  Wire.beginTransmission(BMP_ADDRESS);
  #if (ARDUINO >= 100)
    Wire.write(reg);
  #else
    Wire.send(reg);
  #endif
    Wire.endTransmission();

  Wire.requestFrom(BMP_ADDRESS, 1);
  #if (ARDUINO >= 100)
    return Wire.read();
  #else
    return Wire.receive();
  #endif
}

/**************************************************************************/
/*
    Reads 16 bit value from the sensor register over I2C
*/
/**************************************************************************/
uint16_t BMP085_BMP180::read16(uint8_t reg)
{
  uint16_t value;

  Wire.beginTransmission(BMP_ADDRESS);
  #if (ARDUINO >= 100)
    Wire.write(reg);
  #else
    Wire.send(reg);
  #endif
    Wire.endTransmission();

  Wire.requestFrom(BMP_ADDRESS, 2);
  #if (ARDUINO >= 100)
    value  = Wire.read() << 8;
    value |= Wire.read();
  #else
    value  = Wire.receive() << 8;
    value |= Wire.receive();
  #endif

  return value;
}

/**************************************************************************/
/*
    Writes 8 bit value to the sensor register over I2C
*/
/**************************************************************************/
void BMP085_BMP180::write8(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(BMP_ADDRESS);

  #if (ARDUINO >= 100)
    Wire.write(reg);
    Wire.write(value);
  #else
    Wire.send(reg);
    Wire.send(value);
  #endif
    Wire.endTransmission();
}
