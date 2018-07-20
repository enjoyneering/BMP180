/***************************************************************************************************/
/*
  This is an Arduino basic library for Bosch BMP180 & BMP085 barometric pressure &
  temperature sensor
  
  Range                 typ. resolution   typ. accuracy   typ. relative accuracy
  30,000Pa..110,000Pa   ±1Pa              ±100Pa          ±12Pa
  0°C..+65°C            ±0.1°C            ±1.0°C          xx

  written by : enjoyneering79
  sourse code: https://github.com/enjoyneering/

  This sensor uses I2C bus to communicate, specials pins are required to interface
  Board:                                    SDA                    SCL
  Uno, Mini, Pro, ATmega168, ATmega328..... A4                     A5
  Mega2560, Due............................ 20                     21
  Leonardo, Micro, ATmega32U4.............. 2                      3
  Digistump, Trinket, ATtiny85............. 0/physical pin no.5    2/physical pin no.7
  Blue Pill, STM32F103xxxx boards.......... PB7*                   PB6*
  ESP8266 ESP-01:.......................... GPIO0/D5               GPIO2/D3
  NodeMCU 1.0, WeMos D1 Mini............... GPIO4/D2               GPIO5/D1
  ESP32.................................... GPIO21                 GPIO22

                                           *STM32F103xxxx pins PB6/PB7 are 5v tolerant, but
                                            bi-directional logic level converter is recommended

  NOTE:
  - EOC  pin is not used, shows the end of conversion
  - XCLR pin is not used, reset pin

  Frameworks & Libraries:
  ATtiny Core           - https://github.com/SpenceKonde/ATTinyCore
  ESP32 Core            - https://github.com/espressif/arduino-esp32
  ESP8266 Core          - https://github.com/esp8266/Arduino
  ESP8266 I2C lib fixed - https://github.com/enjoyneering/ESP8266-I2C-Driver
  STM32 Core            - https://github.com/rogerclarkmelbourne/Arduino_STM32

  GNU GPL license, all text above must be included in any redistribution, see link below for details:
  - https://www.gnu.org/licenses/licenses.html
*/
/***************************************************************************************************/

#include "BMP180.h"


/**************************************************************************/
/*
    Constructor

    NOTE:
    - BMP180_ULTRALOWPOWER, pressure oversampled 1 time  & consumption 3μA
    - BMP180_STANDARD,      pressure oversampled 2 times & consumption 5μA
    - BMP180_HIGHRES,       pressure oversampled 4 times & consumption 7μA
    - BMP180_ULTRAHIGHRES,  pressure oversampled 8 times & consumption 12μA
*/
/**************************************************************************/
BMP180::BMP180(BMP180_RESOLUTION res_mode)
{
  _resolution = res_mode;
}

/**************************************************************************/
/*
    begin()

    Initializes I2C and configures the sensor

    NOTE: call this function before doing anything else
*/
/**************************************************************************/
#if defined(ESP8266)
bool BMP180::begin(uint8_t sda, uint8_t scl)
{
  Wire.begin(sda, scl);
  Wire.setClock(100000UL);                                  //experimental! ESP8266 i2c bus speed: 100kHz..400kHz/100000UL..400000UL, default 100000UL
  Wire.setClockStretchLimit(230);                           //experimental! default 230usec
#else
bool BMP180::begin(void) 
{
  Wire.begin();
  Wire.setClock(100000UL);                                  //experimental! AVR i2c bus speed: 31kHz..400kHz/31000UL..400000UL, default 100000UL
#endif

  if (read8(BMP180_GET_ID) != BMP180_CHIP_ID) return false; //safety check, make sure the sensor is connected

  return readCalibrationCoefficients();                     //safety check, make sure all coefficients are valid
}

/**************************************************************************/
/*
    getPressure()

    Calculates compensated pressure, in Pa

    NOTE:
    - resolutin ±1Pa with accuracy ±150Pa at range 30,000Pa..110,000Pa
*/
/**************************************************************************/
int32_t BMP180::getPressure(void)
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

  UT = readRawTemperature();                                             //read uncompensated temperature, 16-bit
  if (UT == BMP180_ERROR) return BMP180_ERROR;                           //error handler, collision on i2c bus

  UP = readRawPressure();                                                //read uncompensated pressure, 19-bit
  if (UP == BMP180_ERROR) return BMP180_ERROR;                           //error handler, collision on i2c bus

  B5 = computeB5(UT);

  /* pressure calculation */
  B6 = B5 - 4000;
  X1 = ((int32_t)_cal_coeff.bmpB2 * ((B6 * B6) >> 12)) >> 11;
  X2 = ((int32_t)_cal_coeff.bmpAC2 * B6) >> 11;
  X3 = X1 + X2;
  B3 = ((((int32_t)_cal_coeff.bmpAC1 * 4 + X3) << _resolution) + 2) / 4;

  X1 = ((int32_t)_cal_coeff.bmpAC3 * B6) >> 13;
  X2 = ((int32_t)_cal_coeff.bmpB1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = ((uint32_t)_cal_coeff.bmpAC4 * (X3 + 32768L)) >> 15;
  B7 = (UP - B3) * (50000UL >> _resolution);

  if   (B7 < 0x80000000) pressure = (B7 * 2) / B4;
  else                   pressure = (B7 / B4) * 2;

  X1 = pow((pressure >> 8), 2);
  X1 = (X1 * 3038L) >> 16;
  X2 = (-7357L * pressure) >> 16;

  return pressure = pressure + ((X1 + X2 + 3791L) >> 4);
}

/**************************************************************************/
/*
    getTemperature()

    Calculates compensated temperature, in °C

    NOTE:
    - resolution ±0.1°C with accuracy ±1.0°C at range 0°C..+65°C
*/
/**************************************************************************/
float BMP180::getTemperature(void)
{
  int16_t rawTemperature = readRawTemperature();

  if (rawTemperature == BMP180_ERROR) return BMP180_ERROR;                                       //error handler, collision on i2c bus
                                      return (float)((computeB5(rawTemperature) + 8) >> 4) / 10;
}

/**************************************************************************/
/*
    getSeaLevelPressure()

    Converts current pressure to sea level pressure at specific true
    altitude, in Pa

    NOTE:
    - true altitude is the actual elevation above sea level, to find out
      your current true altitude do search with google earth or gps
    - see level pressure is commonly used in weather reports & forecasts
      to compensate current true altitude
    - for example, we know that a sunny day happens if the current sea
      level pressure is 250Pa above the average sea level pressure of
      101325 Pa, so by converting the current pressure to sea level &
      comparing it with an average sea level pressure we can instantly
      predict the weather conditions
*/
/**************************************************************************/
int32_t BMP180::getSeaLevelPressure(int16_t trueAltitude)
{
  int32_t pressure = getPressure();

  if (pressure == BMP180_ERROR) return BMP180_ERROR;
                                return (pressure / pow(1.0 - (float)trueAltitude / 44330, 5.255));
}

/**************************************************************************/
/*
    softReset()

    Soft reset

    NOTE:
    - performs the same sequence as power on reset
*/
/**************************************************************************/
void BMP180::softReset(void)
{
  write8(BMP180_START_SOFT_RESET, BMP180_GET_SOFT_RESET);
}

/**************************************************************************/
/*
    readFirmwareVersion()

    Reads ML & AL Version

    NOTE:
    - ML version is LSB, 4-bit..0-bit
    - AL version is MSB, 7-bit..5-bit
*/
/**************************************************************************/
uint8_t BMP180::readFirmwareVersion(void)
{
  return read8(BMP180_GET_VERSION);
}

/**************************************************************************/
/*
    readDeviceID()

    Reads chip ID
*/
/**************************************************************************/
uint8_t BMP180::readDeviceID(void)
{
  if (read8(BMP180_GET_ID) == BMP180_CHIP_ID) return 180;
                                              return false;
}

/**************************************************************************/
/*
    readCalibrationCoefficients()

    Reads factory calibration coefficients from E2PROM

    NOTE:
    - every sensor module has individual calibration coefficients
    - before first temperature & pressure calculation master have to read
      calibration coefficients from 176-bit E2PROM
*/
/**************************************************************************/
bool BMP180::readCalibrationCoefficients()
{
  int32_t value = 0;

  for (uint8_t reg = BMP180_CAL_AC1; reg <= BMP180_CAL_MD; reg++)
  {
    value = read16(reg);

    if (value == BMP180_ERROR) return false; //error handler, collision on i2c bus

    switch (reg)
    {
      case BMP180_CAL_AC1:                   //used for pressure computation
        _cal_coeff.bmpAC1 = value;
        break;

      case BMP180_CAL_AC2:                   //used for pressure computation
        _cal_coeff.bmpAC2 = value;
        break;

      case BMP180_CAL_AC3:                   //used for pressure computation
        _cal_coeff.bmpAC3 = value;
        break;

      case BMP180_CAL_AC4:                   //used for pressure computation
        _cal_coeff.bmpAC4 = value;
        break;

      case BMP180_CAL_AC5:                   //used for temperature computation
        _cal_coeff.bmpAC5 = value;
        break;

      case BMP180_CAL_AC6:                   //used for temperature computation
        _cal_coeff.bmpAC6 = value;
        break;

      case BMP180_CAL_B1:                    //used for pressure computation
        _cal_coeff.bmpB1 = value;
        break;

      case BMP180_CAL_B2:                    //used for pressure computation
        _cal_coeff.bmpB2 = value;
        break;

      case BMP180_CAL_MB:                    //???
        _cal_coeff.bmpMB = value;
        break;

      case BMP180_CAL_MC:                    //used for temperature computation
        _cal_coeff.bmpMC = value;
        break;

      case BMP180_CAL_MD:                    //used for temperature computation
        _cal_coeff.bmpMD = value;
        break;
    }
  }

  return true;
}

/**************************************************************************/
/*
    readRawTemperature()

    Reads raw/uncompensated temperature value, 16-bit
*/
/**************************************************************************/
uint16_t BMP180::readRawTemperature(void)
{
  /* send temperature measurement command */
  if (write8(BMP180_START_MEASURMENT, BMP180_GET_TEMPERATURE) != true) return BMP180_ERROR; //error handler, collision on i2c bus

  /* set measurement delay */
  delay(5);

  /* read result */
  return read16(BMP180_READ_ADC_MSB);                                                       //reads msb + lsb
}

/**************************************************************************/
/*
    readRawPressure()

    Reads raw/uncompensated pressure value, 19-bits
*/
/**************************************************************************/
uint32_t BMP180::readRawPressure(void)
{
  uint8_t  regControl  = 0;
  uint32_t rawPressure = 0;

  /* convert resolution to register control */
  switch (_resolution)
  {
    case BMP180_ULTRALOWPOWER:               //oss0
      regControl = BMP180_GET_PRESSURE_OSS0;
      break;

    case BMP180_STANDARD:                    //oss1
      regControl = BMP180_GET_PRESSURE_OSS1;
      break;

    case BMP180_HIGHRES:                     //oss2
      regControl = BMP180_GET_PRESSURE_OSS2;
      break;

    case BMP180_ULTRAHIGHRES:                //oss3
      regControl = BMP180_GET_PRESSURE_OSS2;
      break;
  }

  /* send pressure measurement command */
  if (write8(BMP180_START_MEASURMENT, regControl) != true) return BMP180_ERROR; //error handler, collision on i2c bus

  /* set measurement delay */
  switch (_resolution)
  {
    case BMP180_ULTRALOWPOWER:
      delay(5);
      break;

    case BMP180_STANDARD:
      delay(8);
      break;

    case BMP180_HIGHRES:
      delay(14);
      break;

    case BMP180_ULTRAHIGHRES:
      delay(26);
      break;
  }

  /* read result msb + lsb */
  rawPressure = read16(BMP180_READ_ADC_MSB);            //16-bits
  if (rawPressure == BMP180_ERROR) return BMP180_ERROR; //error handler, collision on i2c bus

  /* read result xlsb */
  rawPressure <<= 8;
  rawPressure |= read8(BMP180_READ_ADC_XLSB);           //19-bits

  rawPressure >>= (8 - _resolution);

  return rawPressure;
}

/**************************************************************************/
/*
    computeB5()

    Computes B5 value

    NOTE:
    - to compensate raw/uncompensated temperature
    - also used for compensated pressure calculation
*/
/**************************************************************************/
int32_t BMP180::computeB5(int32_t UT)
{
  int32_t X1 = ((UT - (int32_t)_cal_coeff.bmpAC6) * (int32_t)_cal_coeff.bmpAC5) >> 15;
  int32_t X2 = ((int32_t)_cal_coeff.bmpMC << 11) / (X1 + (int32_t)_cal_coeff.bmpMD);

  return X1 + X2;
}

/**************************************************************************/
/*
    read8()

    Reads 8-bit value over I2C
*/
/**************************************************************************/
uint8_t BMP180::read8(uint8_t reg)
{
  Wire.beginTransmission(BMP180_ADDRESS);
  #if (ARDUINO >= 100)
  Wire.write(reg);
  #else
  Wire.send(reg);
  #endif
  if (Wire.endTransmission(true) != 0) return BMP180_ERROR; //error handler, collision on i2c bus

  #if defined(_VARIANT_ARDUINO_STM32_)
  Wire.requestFrom(BMP180_ADDRESS, 1);
  #else
  Wire.requestFrom(BMP180_ADDRESS, 1, true);                //true, stop message after transmission & releas i2c bus
  #endif
  if (Wire.available() != 1) return BMP180_ERROR;           //check "wire.h" rxBuffer & error handler, collision on i2c bus

  /* read byte from "wire.h" rxBuffer */
  #if (ARDUINO >= 100)
  return Wire.read();
  #else
  return Wire.receive();
  #endif
}

/**************************************************************************/
/*
    read16()

    Reads 16-bits value over I2C
*/
/**************************************************************************/
uint16_t BMP180::read16(uint8_t reg)
{
  uint16_t value = 0;

  Wire.beginTransmission(BMP180_ADDRESS);
  #if (ARDUINO >= 100)
  Wire.write(reg);
  #else
  Wire.send(reg);
  #endif
  if (Wire.endTransmission(true) != 0) return BMP180_ERROR; //error handler, collision on i2c bus

  #if defined(_VARIANT_ARDUINO_STM32_)
  Wire.requestFrom(BMP180_ADDRESS, 2);
  #else
  Wire.requestFrom(BMP180_ADDRESS, 2, true);                //true, stop message after transmission & releas i2c bus
  #endif
  if (Wire.available() != 2) return BMP180_ERROR;           //check "wire.h" rxBuffer & error handler, collision on i2c bus

  /* read 2-bytes from "wire.h" rxBuffer */
  #if (ARDUINO >= 100)
  value  = Wire.read() << 8;                                //read msb
  value |= Wire.read();                                     //read lsb
  #else
  value  = Wire.receive() << 8;
  value |= Wire.receive();
  #endif

  return value;
}

/**************************************************************************/
/*
    write8()

    Writes 8-bits value over I2C
*/
/**************************************************************************/
bool BMP180::write8(uint8_t reg, uint8_t control)
{
  Wire.beginTransmission(BMP180_ADDRESS);

  #if (ARDUINO >= 100)
  Wire.write(reg);
  Wire.write(control);
  #else
  Wire.send(reg);
  Wire.send(control);
  #endif
  
  if (Wire.endTransmission(true) == 0) return true;
                                       return false; //error handler, collision on i2c bus
}
