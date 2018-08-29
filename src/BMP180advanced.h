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

#ifndef BMP180advanced_h
#define BMP180advanced_h
  
#include <BMP180.h>


#define BMP180_SEA_LEVEL_PRESSURE 101325 //average see level pressure, in Pa
#define BMP180_FORCE_READ_DATA    -255   //force to read data over i2c

class BMP180advanced: public BMP180
{
 public:
  BMP180advanced(BMP180_RESOLUTION = BMP180_ULTRAHIGHRES);              //BMP180_ULTRAHIGHRES, by default

  float   getPressure_hPa(int32_t pressure = BMP180_FORCE_READ_DATA);  //in hPa
  float   getPressure_mmHg(int32_t pressure = BMP180_FORCE_READ_DATA); //in mmHg
  float   getPressure_inHg(int32_t pressure = BMP180_FORCE_READ_DATA); //in inHg

  float   getSeaLevelPressure_hPa(int16_t trueAltitude = 115);          //in hPa, by default true altitude id 115 meters
  float   getSeaLevelPressure_mmHg(int16_t trueAltitude = 115);         //in mmHg
  float   getSeaLevelPressure_inHg(int16_t trueAltitude = 115);         //in inHg

  uint8_t getForecast(int16_t trueAltitude = 115);

  
 private:

};

#endif
