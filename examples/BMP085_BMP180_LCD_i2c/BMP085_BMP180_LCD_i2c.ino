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
#pragma GCC optimize ("O2")     //code optimisation controls - "O2" & "O3" code performance, "Os" code size

#include <Wire.h>               //for ESP8266 use bug free i2c driver https://github.com/enjoyneering/ESP8266-I2C-Driver
#include <BMP180.h>
#include <LiquidCrystal_I2C.h>  //https://github.com/enjoyneering/LiquidCrystal_I2C
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#endif

#define LCD_ROWS           4    //qnt. of lcd rows
#define LCD_COLUMNS        20   //qnt. of lcd columns

#define LCD_DEGREE_SYMBOL  0xDF //degree symbol from lcd ROM, see p.9 of GDM2004D datasheet
#define LCD_SPACE_SYMBOL   0x20 //space  symbol from lcd ROM, see p.9 of GDM2004D datasheet

#define MAX_TEMPERATURE    50   //max temp, in °C

const uint8_t iconTemperature[8] PROGMEM = {0x04, 0x0E, 0x0E, 0x0E, 0x0E, 0x1F, 0x1F, 0x0E}; //PROGMEM saves variable to flash & keeps dynamic memory free
const uint8_t iconPressure[8]    PROGMEM = {0x04, 0x04, 0x15, 0x0E, 0x04, 0x00, 0x00, 0x1F};
const uint8_t iconSeePressure[8] PROGMEM = {0x04, 0x04, 0x15, 0x0E, 0x04, 0x00, 0x0A, 0x15};

float value = 0;

/*
BMP180(resolution)

resolution:
BMP180_ULTRALOWPOWER - pressure oversampled 1 time  & power consumption 3μA
BMP180_STANDARD      - pressure oversampled 2 times & power consumption 5μA
BMP180_HIGHRES       - pressure oversampled 4 times & power consumption 7μA
BMP180_ULTRAHIGHRES  - pressure oversampled 8 times & power consumption 12μA, library default
*/
BMP180            myBMP(BMP180_ULTRAHIGHRES);
LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);


void setup()
{
  #if defined(ESP8266)
  WiFi.persistent(false);                                                                //disable saving wifi config into SDK flash area
  WiFi.forceSleepBegin();                                                                //disable swAP & station by calling "WiFi.mode(WIFI_OFF)" & put modem to sleep
  #endif

  Serial.begin(115200);

  /* LCD connection check */  
  while (lcd.begin(LCD_COLUMNS, LCD_ROWS, LCD_5x8DOTS) != true)                          //20x4 display, 5x8 pixels size
  {
    Serial.println(F("PCF8574 is not connected or lcd pins declaration is wrong. Only pins numbers: 4,5,6,16,11,12,13,14 are legal."));
    delay(5000);
  }

  lcd.print(F("PCF8574 is OK"));                                                         //(F()) saves string to flash & keeps dynamic memory free
  delay(1000);

  lcd.clear();

  /* BMP180 or BMP085 connection check */
  while (myBMP.begin() != true)
  {
    lcd.setCursor(0, 0);
    lcd.print(F("BMP180 error"));

    Serial.println(F("Bosch BMP180/BMP085 is not connected or fail to read calibration coefficients"));
    delay(5000);
  }

  lcd.clear();

  lcd.print(F("BMP180 OK"));
  delay(2000);

  lcd.clear();

  /* load custom symbol to CGRAM */
  lcd.createChar(0, iconTemperature);
  lcd.createChar(1, iconPressure);
  lcd.createChar(2, iconSeePressure);

  /* prints static text */
  lcd.setCursor(0, 0);
  lcd.write(0);

  lcd.setCursor(0, 1);
  lcd.write(1);

  lcd.setCursor(0, 2);
  lcd.write(2);
}

void loop()
{
  /* prints temperature text */
  value = myBMP.getTemperature();

  if (value == BMP180_ERROR) myBMP.softReset();

  lcd.setCursor(2, 0);
  if   (value != BMP180_ERROR) lcd.print(value, 1);
  else                         lcd.print(F("xx"));                                       //communication error is occurred
  lcd.write(LCD_DEGREE_SYMBOL);
  lcd.print(F("C"));
  lcd.write(LCD_SPACE_SYMBOL);

  if   (value != BMP180_ERROR) lcd.printHorizontalGraph('T', 3, value, MAX_TEMPERATURE); //name of the bar, 4-th row, current value, max. value
  else                         lcd.printHorizontalGraph('T', 3, 0, MAX_TEMPERATURE);     //print 0 if communication error is occurred

  /* prints pressure text */
  value = myBMP.getPressure();

  lcd.setCursor(2, 1);
  if   (value != BMP180_ERROR) lcd.print(value);
  else                         lcd.print(F("xx"));
  lcd.print(F("Pa"));
  lcd.write(LCD_SPACE_SYMBOL);

  /* prints sea level pressure text */
  value = myBMP.getSeaLevelPressure(115);                                                //true altitude is 115 meters, do search with google earth or gps

  lcd.setCursor(2, 2);
  if   (value != BMP180_ERROR) lcd.print(value);
  else                         lcd.print(F("xx"));
  lcd.print(F("Pa"));
  lcd.write(LCD_SPACE_SYMBOL);

  delay(5000);
}
