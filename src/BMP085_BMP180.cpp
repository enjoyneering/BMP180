/***************************************************************************************************/
/* 
  This is an Arduino library for the BOSCH BMP085/BMP180 Barometric Pressure and Temperature sensor

  Range                 Max. Resolution   Max. Accuracy
  30,000Pa..110,000Pa   -+1Pa             -+150Pa
  0C..+65C              -+0.1C            -+1.0C

  written by : enjoyneering79
  sourse code: https://github.com/enjoyneering/

  This sensor uses I2C bus to communicate, specials pins are required to interface

  Connect chip to pins:    SDA        SCL
  Uno, Mini, Pro:          A4         A5
  Mega2560, Due:           20         21
  Leonardo:                2          3
  ATtiny85:                0(5)       2/A1(7)   (ATTinyCore  - https://github.com/SpenceKonde/ATTinyCore
                                                 & TinyWireM - https://github.com/SpenceKonde/TinyWireM)
  ESP8266 ESP-01:          GPIO0/D5   GPIO2/D3  (ESP8266Core - https://github.com/esp8266/Arduino)
  Node_MCU 1.0:            GPIO4/D2   GPIO5/D1
  WeMos D1 Mini:           GPIO4/D2   GPIO5/D1

  NOTE: EOC  pin is not used, shows the end of conversion.
        XCLR pin is not used, reset pin.

  BSD license, all text above must be included in any redistribution
*/
/***************************************************************************************************/

#include "BMP085_BMP180.h"


/**************************************************************************/
/*
    Constructor
*/
/**************************************************************************/
BMP085_BMP180::BMP085_BMP180(BMP085_RESOLUTION res_mode)
{
  _resolution = res_mode;
}

/**************************************************************************/
/*
    Initializes I2C and configures the sensor

    NOTE: call this function before doing anything else
*/
/**************************************************************************/
#if defined(ESP8266)
bool BMP085_BMP180::begin(uint8_t sda, uint8_t scl)
{
  Wire.begin(sda, scl);
  Wire.setClock(100000UL);                    //experimental! ESP8266 i2c bus speed: 100kHz..400kHz/100000UL..400000UL, default 100000UL
  Wire.setClockStretchLimit(230);             //experimental! default 230
#else
bool BMP085_BMP180::begin(void) 
{
  Wire.begin();
  Wire.setClock(100000UL);                    //experimental! AVR i2c bus speed: 31kHz..400kHz/31000UL..400000UL, default 100000UL
#endif

  if (read8(BMP085_GET_ID) != BMP085_CHIP_ID) //safety check, make sure the sensor is connected
  {
    #ifdef BMP085_BMP180
    Serial.println("BMP085/BMP180: can't find the sensor on the bus");
    #endif
    return false;
  }

  readCalibrationCoefficients();

  return true;
}

/**************************************************************************/
/*
    Reads Pressure in Pa
*/
/**************************************************************************/
int32_t BMP085_BMP180::readPressure(void)
{
  int32_t  UT       = 0;
  int32_t  UP       = 0;
  int32_t  B3       = 0;
  int32_t  B5       = 0;
  int32_t  B6       = 0;
  int32_t  X1       = 0; 
  int32_t  X2       = 0; 
  int32_t  X3       = 0;
  int32_t  pressure = 0;
  uint32_t B4       = 0;
  uint32_t B7       = 0;

  UT = readRawTemperature(); //temperature data, 16bit
  UP = readRawPressure();    //pressure data,    16bit to 19bit

  B5 = computeB5(UT);

  /* pressure calculation */
  B6 = B5 - 4000;
  X1 = ((int32_t)_cal_coeff.BMP_B2 * ((B6 * B6) >> 12)) >> 11;
  X2 = ((int32_t)_cal_coeff.BMP_AC2 * B6) >> 11;
  X3 = X1 + X2;
  B3 = ((((int32_t)_cal_coeff.BMP_AC1 * 4 + X3) << _resolution) + 2) / 4;

  X1 = ((int32_t)_cal_coeff.BMP_AC3 * B6) >> 13;
  X2 = ((int32_t)_cal_coeff.BMP_B1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = ((uint32_t)_cal_coeff.BMP_AC4 * (X3 + 32768UL)) >> 15;
  B7 = (UP - B3) * (50000UL >> _resolution);

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

  return pressure = pressure + ((X1 + X2 + 3791L) >> 4);
}

/**************************************************************************/
/*
    Reads Temperature in deg.C
*/
/**************************************************************************/
float BMP085_BMP180::readTemperature(void)
{
  return (float)((computeB5(readRawTemperature()) + 8) >> 4) / 10; //temperature
}

/**************************************************************************/
/*
    Calculates Pressure, in hPa
*/
/**************************************************************************/
float BMP085_BMP180::getHectoPascalPressure(void)
{
  return (float)readPressure() / 100;
}

/**************************************************************************/
/*
    Calculates Pressure, in mmHg
*/
/**************************************************************************/
float BMP085_BMP180::getMillimeterMercuryPressure(void)
{
  return (float)readPressure() * 0.0075;
}

/**************************************************************************/
/*
    Calculates altitude at specific sea level pressure in meters

    NOTE: sea level pressure in Pa
*/
/**************************************************************************/
float BMP085_BMP180::getAltitude(float seaLevelPressure)
{
  return 44330.0 * (1.0 - pow(readPressure() / seaLevelPressure, 0.1903));
}

/**************************************************************************/
/*
    Calculates sea level pressure at specific altitude, in Pa

    Use the see level pressure to compensate current altitude, commonly used 
    in weather reports

    NOTE: altitude in meters, 80 meters by default
*/
/**************************************************************************/
int32_t BMP085_BMP180::getSeaLevelPressure(float altitude)
{
  return (readPressure() / pow(1.0 - altitude / 44330, 5.255));
}

/**************************************************************************/
/*
    Soft reset

    Performs the same sequence as power on reset.
*/
/**************************************************************************/
void BMP085_BMP180::softReset(void)
{
  write8(BMP085_START_SOFT_RESET, BMP085_GET_SOFT_RESET);
}

/**************************************************************************/
/*
    Reads factory calibration coefficients

    Before the first temperature & pressure calculation master reads out
    the calibration coefficients E2PROM.
*/
/**************************************************************************/
void BMP085_BMP180::readCalibrationCoefficients()
{
  _cal_coeff.BMP_AC1 = read16(BMP085_CAL_AC1);
  _cal_coeff.BMP_AC2 = read16(BMP085_CAL_AC2);
  _cal_coeff.BMP_AC3 = read16(BMP085_CAL_AC3);
  _cal_coeff.BMP_AC4 = read16(BMP085_CAL_AC4);
  _cal_coeff.BMP_AC5 = read16(BMP085_CAL_AC5);
  _cal_coeff.BMP_AC6 = read16(BMP085_CAL_AC6);

  _cal_coeff.BMP_B1 = read16(BMP085_CAL_B1);
  _cal_coeff.BMP_B2 = read16(BMP085_CAL_B2);

  _cal_coeff.BMP_MB = read16(BMP085_CAL_MB);
  _cal_coeff.BMP_MC = read16(BMP085_CAL_MC);
  _cal_coeff.BMP_MD = read16(BMP085_CAL_MD);
}

/**************************************************************************/
/*
    Reads raw temperature
*/
/**************************************************************************/
uint16_t BMP085_BMP180::readRawTemperature(void)
{
  write8(BMP085_START_MEASURMENT, BMP085_GET_TEMPERATURE);
  delay(5);                                                //measurement delay

  return read16(BMP085_READ_RESULT);
}

/**************************************************************************/
/*
    Reads raw pressure
*/
/**************************************************************************/
uint32_t BMP085_BMP180::readRawPressure(void)
{
  uint32_t rawPressure = 0;

  write8(BMP085_START_MEASURMENT, BMP085_GET_PRESSURE + (_resolution << 6));

  /* set measurement delay */
  switch (_resolution)
  {
    case BMP085_ULTRALOWPOWER:
      delay(5);
      break;
    case BMP085_STANDARD:
      delay(8);
      break;
    case BMP085_HIGHRES:
      delay(14);
      break;
    case BMP085_ULTRAHIGHRES:
      delay(26);
      break;
   }

  rawPressure = read16(BMP085_READ_RESULT);

  rawPressure <<= 8;
  rawPressure |= read8(BMP085_READ_RESULT + 2);
  rawPressure >>= (8 - _resolution);

  return rawPressure;
}

/**************************************************************************/
/*
    Computes B5
*/
/**************************************************************************/
int32_t BMP085_BMP180::computeB5(int32_t UT)
{
  int32_t X1 = (UT - (int32_t)_cal_coeff.BMP_AC6) * ((int32_t)_cal_coeff.BMP_AC5) >> 15;
  int32_t X2 = ((int32_t)_cal_coeff.BMP_MC << 11) / (X1 + (int32_t)_cal_coeff.BMP_MD);

  return X1 + X2;
}

/**************************************************************************/
/*
    Reads 8 bit value from the sensor register over I2C
*/
/**************************************************************************/
uint8_t BMP085_BMP180::read8(uint8_t reg)
{
  uint8_t  pollCounter = BMP085_POLL_LIMIT;

  do
  {
    pollCounter--;
    if (pollCounter == 0)                                    //error handler
    {
      #ifdef BMP085_DEBUG_INFO
      Serial.println("BMP085/BMP180: can't request a byte");
      #endif
      return BMP085_ERROR;
    }
    Wire.beginTransmission(BMP085_ADDRESS);
    #if (ARDUINO >= 100)
    Wire.write(reg);
    #else
    Wire.send(reg);
    #endif
  }
  while (Wire.endTransmission(true) != 0);                   //true = stop message after transmission & releas the I2C bus

  pollCounter = BMP085_POLL_LIMIT;

  do
  {
    pollCounter--;
    if (pollCounter == 0)                                    //error handler
    {
      #ifdef BMP085_DEBUG_INFO
      Serial.println("BMP085/BMP180: can't read a byte");
      #endif
      return BMP085_ERROR;
    }
    Wire.requestFrom(BMP085_ADDRESS, 1, true);               //true = stop message after transmission & releas the I2C bus
  }
  while (Wire.available() != 1);                             //check rxBuffer

  /* read byte from "wire.h" buffer */
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
  uint8_t  pollCounter = BMP085_POLL_LIMIT;
  uint16_t value       = 0;

  do
  {
    pollCounter--;
    if (pollCounter == 0)                                       //error handler
    {
      #ifdef BMP085_DEBUG_INFO
      Serial.println("BMP085/BMP180: can't request two bytes");
      #endif
      return BMP085_ERROR;
    }
    Wire.beginTransmission(BMP085_ADDRESS);
    #if (ARDUINO >= 100)
    Wire.write(reg);
    #else
    Wire.send(reg);
    #endif
  }
  while (Wire.endTransmission(true) != 0);

  pollCounter = BMP085_POLL_LIMIT;

  do
  {
    pollCounter--;
    if (pollCounter == 0)                                       //error handler
    {
      #ifdef BMP085_DEBUG_INFO
      Serial.println("BMP085/BMP180: can't read two bytes");
      #endif
      return BMP085_ERROR;
    }
    Wire.requestFrom(BMP085_ADDRESS, 2, true);                  //true = stop message after transmission & releas the I2C bus
  }
  while (Wire.available() != 2);                                //check rxBuffer

  /* read 2 bytes from "wire.h" buffer */
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
  uint8_t pollCounter = BMP085_POLL_LIMIT;

  do
  {
    pollCounter--;
    if (pollCounter == 0)                                  //error handler
    {
      #ifdef BMP085_DEBUG_INFO
      Serial.println("BMP085/BMP180: can't write a byte");
      #endif
      return;
    }
    Wire.beginTransmission(BMP085_ADDRESS);
    #if (ARDUINO >= 100)
    Wire.write(reg);
    Wire.write(value);
    #else
    Wire.send(reg);
    Wire.send(value);
    #endif
  }
  while (Wire.endTransmission(true) != 0);
}
