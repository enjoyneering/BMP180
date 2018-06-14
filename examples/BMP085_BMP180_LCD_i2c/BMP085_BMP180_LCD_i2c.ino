/***************************************************************************************************/
/* 
  Example for BOSCH BMP180 & BMP085 Barometric Pressure & Temperature sensor

  Range                 max. resolution   max. accuracy
  30,000Pa..110,000Pa   -+1Pa             -+150Pa
  0C..+65C              -+0.1C            -+1.0C
  
  written by : enjoyneering79
  sourse code: https://github.com/enjoyneering/

  This sketch uses I2C bus to communicate, specials pins are required to interface
  Board:                                    SDA                    SCL
  Uno, Mini, Pro, ATmega168, ATmega328..... A4                     A5
  Mega2560, Due............................ 20                     21
  Leonardo, Micro, ATmega32U4.............. 2                      3
  Digistump, Trinket, ATtiny85............. 0/physical pin no.5    2/physical pin no.7
  Blue Pill, STM32F103xxxx boards.......... B7*                    B6*
  ESP8266 ESP-01:.......................... GPIO0/D5               GPIO2/D3
  NodeMCU 1.0, WeMos D1 Mini............... GPIO4/D2               GPIO5/D1

                                            *STM32F103xxxx pins B7/B7 are 5v tolerant, but bi-directional
                                             logic level converter is recommended

  Frameworks & Libraries:
  ATtiny Core           - https://github.com/SpenceKonde/ATTinyCore
  ESP8266 Core          - https://github.com/esp8266/Arduino
  ESP8266 I2C lib fixed - https://github.com/enjoyneering/ESP8266-I2C-Driver
  STM32 Core            - https://github.com/rogerclarkmelbourne/Arduino_STM32

  NOTE:
  - EOC  pin is not used, shows the end of conversion
  - XCLR pin is not used, reset pin

  GNU GPL license, all text above must be included in any redistribution, see link below for details
  - https://www.gnu.org/licenses/licenses.html
*/
/***************************************************************************************************/
#include <Wire.h>
#include <BMP180.h>
#include <LiquidCrystal_I2C.h>  //https://github.com/enjoyneering/LiquidCrystal_I2C

#define LCD_ROWS           4    //qnt. of lcd rows
#define LCD_COLUMNS        20   //qnt. of lcd columns
#define LCD_DEGREE_SYMBOL  0xDF //degree symbol from lcd ROM
#define LCD_SPACE_SYMBOL   0x20 //space  symbol from lcd ROM

#define MAX_TEMPERATURE    35   //max temp, deg.C

const uint8_t icon_pressure[8]     PROGMEM = {0x04, 0x04, 0x15, 0x0E, 0x04, 0x00, 0x00, 0x1F}; //PROGMEM saves variable to flash & keeps dynamic memory free
const uint8_t icon_see_pressure[8] PROGMEM = {0x04, 0x04, 0x15, 0x0E, 0x04, 0x00, 0x0A, 0x15};
const uint8_t icon_temperature[8]  PROGMEM = {0x04, 0x0E, 0x0E, 0x0E, 0x0E, 0x1F, 0x1F, 0x0E};

float temperature = 0;

/*
BMP180(resolution)

resolution:
BMP180_ULTRALOWPOWER
BMP180_STANDARD
BMP180_HIGHRES
BMP180_ULTRAHIGHRES  (by default)
*/

BMP180            myBMP(BMP180_ULTRAHIGHRES);
LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);


void setup()
{
  Serial.begin(115200);

  /* LCD connection check */  
  while (lcd.begin(LCD_COLUMNS, LCD_ROWS, LCD_5x8DOTS) != true) //20x4 display, 5x8 pixels size
  {
    Serial.println(F("PCF8574 is not connected or lcd pins declaration is wrong. Only pins numbers: 4,5,6,16,11,12,13,14 are legal.")); //(F()) saves string to flash & keeps dynamic memory
    delay(5000);
  }

  lcd.print("PCF8574 is OK");
  delay(1000);

  lcd.clear();

  /* BMP180 or BMP085 connection check */
  while (myBMP.begin() != true)
  {
    lcd.setCursor(0, 0);
    lcd.print(F("BMP180 error"));
    delay(5000);
  }

  lcd.clear();

  lcd.print(F("BMP180 OK"));
  delay(2000);

  lcd.clear();

  /* load custom symbol to CGRAM */
  lcd.createChar(0, icon_temperature,  'F');                    //'F' - variable stored in flash
  lcd.createChar(1, icon_pressure,     'F');
  lcd.createChar(2, icon_see_pressure, 'F');

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
  temperature = myBMP.readTemperature();

  lcd.setCursor(2, 0);
  if (temperature != BMP180_ERROR) lcd.print(temperature, 1);
  else                             lcd.print(F("xx"));          //communication error is occurred
  lcd.write(LCD_DEGREE_SYMBOL);
  lcd.print(F("C"));
  lcd.write(LCD_SPACE_SYMBOL);

  lcd.setCursor(2, 1);
  lcd.print(getPressure_hPa());
  lcd.print(F("hPa"));
  lcd.write(LCD_SPACE_SYMBOL);

  lcd.setCursor(2, 2);
  lcd.print(myBMP.getSeaLevelPressure(80) / 100);
  lcd.print(F("hPa"));
  lcd.write(LCD_SPACE_SYMBOL);

  lcd.printHorizontalGraph('T', 3, temperature, MAX_TEMPERATURE); //name of the bar, 3-rd row, current value, max. value

  delay(5000);
}
