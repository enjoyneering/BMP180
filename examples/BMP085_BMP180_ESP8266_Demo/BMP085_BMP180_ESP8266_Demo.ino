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
#include <Wire.h>        //use bug free i2c driver https://github.com/enjoyneering/ESP8266-I2C-Driver
#include <BMP180.h>
#include <ESP8266WiFi.h>

/*
BMP180(resolution)

resolution:
BMP180_ULTRALOWPOWER - pressure oversampled 1 time  & power consumption 3μA
BMP180_STANDARD      - pressure oversampled 2 times & power consumption 5μA
BMP180_HIGHRES       - pressure oversampled 4 times & power consumption 7μA
BMP180_ULTRAHIGHRES  - pressure oversampled 8 times & power consumption 12μA, library default
*/
BMP180 myBMP(BMP180_ULTRAHIGHRES);
  
void setup()
{
  WiFi.persistent(false);             //disable saving wifi config into SDK flash area
  WiFi.forceSleepBegin();             //disable swAP & station by calling "WiFi.mode(WIFI_OFF)" & put modem to sleep

  Serial.begin(115200);

  while (myBMP.begin(D2, D1) != true) //sda, scl
  {
    Serial.println(F("Bosch BMP180/BMP085 is not connected or fail to read calibration coefficients"));
    delay(5000);
  }
  
  Serial.println(F("Bosch BMP180/BMP085 sensor is OK")); //(F()) saves string to flash & keeps dynamic memory 
}
  
void loop()
{
  Serial.print(F("Temperature.......: ")); Serial.print(myBMP.getTemperature(), 1); Serial.println(F(" +-1.0C"));
  Serial.print(F("Pressure..........: ")); Serial.print(myBMP.getPressure());       Serial.println(F(" +-100Pa"));

  /* 
     Converts current pressure to sea level pressure assuming your true altitude is 115 meters

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
  Serial.print(F("See level pressure: ")); Serial.print(myBMP.getSeaLevelPressure(115)); Serial.println(F(" Pa"));

  Serial.print(F("Starts over again in 10 sec."));
  delay(10000);  
}
