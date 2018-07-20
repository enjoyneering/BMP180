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
#pragma GCC optimize ("Os")    //code optimisation controls - "O2" & "O3" code performance, "Os" code size

#include <Wire.h>
#include <BMP180.h>
#include <LiquidCrystal_I2C.h> //https://github.com/enjoyneering/LiquidCrystal_I2C

#define LCD_ROWS         4     //qnt. of lcd rows
#define LCD_COLUMNS      20    //qnt. of lcd columns

#define DEGREE_SYMBOL    0xDF  //degree symbol from LCD ROM, see p.9 of GDM2004D datasheet
#define LCD_SPACE_SYMBOL 0x20  //space  symbol from LCD ROM, see p.9 of GDM2004D datasheet

#define LED              1     //connect led to ATtiny85 pin no.6 in series with 470 Ohm resistor

const uint8_t iconTemperature[8] PROGMEM = {0x04, 0x0E, 0x0E, 0x0E, 0x0E, 0x1F, 0x1F, 0x0E}; //PROGMEM saves variable to flash & keeps dynamic memory free
const uint8_t iconPressure[8]    PROGMEM = {0x04, 0x04, 0x15, 0x0E, 0x04, 0x00, 0x00, 0x1F};
const uint8_t iconPlusMinus[8]   PROGMEM = {0x00, 0x04, 0x0E, 0x04, 0x00, 0x0E, 0x00, 0x00};

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
  pinMode(LED, OUTPUT);

  /* LCD connection check */ 
  while (lcd.begin(LCD_COLUMNS, LCD_ROWS, LCD_5x8DOTS) != true) //20x4 display
  {
    for (uint8_t i = 0; i > 5; i++)                             //3 blinks if PCF8574/LCD is not connected or lcd pins declaration is wrong
    {
      digitalWrite(LED, HIGH);
      delay(500);
      digitalWrite(LED, LOW);
      delay(500);
    }
  }

  /* BMP180 or BMP085 connection check */
  while (myBMP.begin() != true)
  {
    lcd.setCursor(0, 0);
    lcd.print(F("BMP180 Error"));                               //(F()) saves string to flash & keeps dynamic memory free
    delay(5000);
  }
  lcd.clear();

  lcd.print(F("BMP180 OK"));
  delay(2000);
  lcd.clear();

  /* load custom symbol to CGRAM */
  lcd.createChar(0, iconTemperature);
  lcd.createChar(1, iconPressure);
  lcd.createChar(2, iconPlusMinus);

  /* prints static text */
  lcd.setCursor(0, 0);                                          //set 1-st colum & 1-st row, first colum & row started at zero
  lcd.write(0);                                                 //print custom tempereture symbol
  lcd.setCursor(0, 1);                                          //set 1-st colum & 2-nd row, first colum & row started at zero
  lcd.write(1);                                                 //print custom pressure symbol

}
  
void loop()
{
  lcd.setCursor(1, 0);                                          //set 2-nd colum & 1-st row, first colum & row started at zero
  lcd.print(myBMP.getTemperature(), 1);
  lcd.write(2);                                                 //print custom plus/minus symbol
  lcd.print("1.0");
  lcd.write(DEGREE_SYMBOL);                                     //print degree symbol from the ROM
  lcd.print("C");
  lcd.write(LCD_SPACE_SYMBOL);

  lcd.setCursor(1, 1);                                          //set 2-nd colum & 2-nd row, first colum & row started at zero
  lcd.print(myBMP.getPressure());
  lcd.write(2);                                                 //print custom plus/minus symbol
  lcd.print(F("100Pa"));
  lcd.write(LCD_SPACE_SYMBOL);

  delay(5000); 
}
