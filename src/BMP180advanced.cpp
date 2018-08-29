/***************************************************************************************************/
/*
  This is an Arduino advanced library for Bosch BMP180 & BMP085 barometric pressure &
  temperature sensor
  
  Range                 typ. resolution   typ. accuracy   typ. relative accuracy
  30,000Pa..110,000Pa   1Pa               ±100Pa          ±12Pa
  0°C..+65°C            0.1°C             ±1.0°C          xx

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

#include "BMP180advanced.h"


/**************************************************************************/
/*
    Constructor
*/
/**************************************************************************/
BMP180advanced::BMP180advanced(BMP180_RESOLUTION res_mode) : BMP180(res_mode)
{

}

/**************************************************************************/
/*
    getPressure_hPa()

    Converts pressure in Pa to hecto pascal/hPa

    NOTE:
    - returns "BMP180_ERROR" if collision on i2c bus happend
*/
/**************************************************************************/
float BMP180advanced::getPressure_hPa(int32_t pressure)
{
  if (pressure == BMP180_FORCE_READ_DATA) pressure = getPressure();

  if (pressure == BMP180_ERROR) return BMP180_ERROR;
                                return (float)pressure / 100;
}

/**************************************************************************/
/*
    getPressure_mmHg()

    Converts pressure in Pa to millimeter of mercury/mmHg

    NOTE:
    - returns "BMP180_ERROR" if collision on i2c bus happend
*/
/**************************************************************************/
float BMP180advanced::getPressure_mmHg(int32_t pressure)
{
  if (pressure == BMP180_FORCE_READ_DATA) pressure = getPressure();

  if (pressure == BMP180_ERROR) return BMP180_ERROR;
                                return (float)pressure * 0.00750;
}

/**************************************************************************/
/*
    getPressure_inHg()

    Converts pressure in Pa to inches of mercury/inHg

    NOTE:
    - returns "BMP180_ERROR" if collision on i2c bus happend
*/
/**************************************************************************/
float BMP180advanced::getPressure_inHg(int32_t pressure)
{
  if (pressure == BMP180_FORCE_READ_DATA) pressure = getPressure();

  if (pressure == BMP180_ERROR) return BMP180_ERROR;
                                return (float)pressure * 0.000295;
}

/**************************************************************************/
/*
    getSeaLevelPressure_hPa()

    Converts current pressure to sea level pressure at specific true
    altitude, in hPa

    NOTE:
    - true altitude is the actual elevation above sea level, to find out
      your current true altitude do search with google earth or gps
    - see level pressure is commonly used in weather reports & forecasts
      to compensate current true altitude
    - for example, we know that a sunny day happens if the current sea
      level pressure is 250Pa above the average sea level pressure of
      101325 Pa, so by converting the current pressure to sea level &
      comparing it with an average sea level pressure, we can instantly
      predict the weather conditions 
*/
/**************************************************************************/
float BMP180advanced::getSeaLevelPressure_hPa(int16_t trueAltitude)
{
  return getPressure_hPa(getSeaLevelPressure(trueAltitude));
}

/**************************************************************************/
/*
    getSeaLevelPressure_mmHg()

    Converts current pressure to sea level pressure at specific true
    altitude, in mmHg

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
float BMP180advanced::getSeaLevelPressure_mmHg(int16_t trueAltitude)
{
  return getPressure_mmHg(getSeaLevelPressure(trueAltitude));
}

/**************************************************************************/
/*
    getSeaLevelPressure_inHg()

    Converts current pressure to sea level pressure at specific true
    altitude, in inHg

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
float BMP180advanced::getSeaLevelPressure_inHg(int16_t trueAltitude)
{
  return getPressure_inHg(getSeaLevelPressure(trueAltitude));
}

/**************************************************************************/
/*
    getForecast()

    Simple forecast

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
uint8_t BMP180advanced::getForecast(int16_t trueAltitude)
{
  int32_t pressure = 0;

  pressure = getSeaLevelPressure(trueAltitude);

  if (pressure == BMP180_ERROR) return BMP180_ERROR;            //error handler, collision on i2c bus

  pressure = pressure - BMP180_SEA_LEVEL_PRESSURE;              //negative value poor weather, positive value good weather

  if (pressure <  -250)                    return 0;            //thunderstorm
  if (pressure >= -250 && pressure < -50)  return 1;            //rain
  if (pressure >= -50  && pressure <  0)   return 2;            //cloudy
  if (pressure >=  0   && pressure <  50)  return 3;            //partly cloudy
  if (pressure >=  50  && pressure <  250) return 4;            //clear
  if (pressure >=  250)                    return 5;            //sunny 
                                           return BMP180_ERROR;                   
}
