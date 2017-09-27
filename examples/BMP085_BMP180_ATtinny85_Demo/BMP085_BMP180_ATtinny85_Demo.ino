/***************************************************************************************************/
/* 
  Example for BOSCH BMP085/BMP180 Barometric Pressure and Temperature sensor

  Range                 Max. Resolution   Max. Accuracy
  30 000Pa..110 000Pa   -+1Pa             -+150Pa
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
  NodeMCU 1.0:             GPIO4/D2   GPIO5/D1
  WeMos D1 Mini:           GPIO4/D2   GPIO5/D1

  NOTE: EOC  pin is not used, shows the end of conversion.
        XCLR pin is not used, reset pin.
*/
/***************************************************************************************************/
#include <TinyWireM.h>
#include <BMP085_BMP180.h>
#include <LiquidCrystal_I2C.h> //https://github.com/enjoyneering/LiquidCrystal_I2C

#define LCD_ROWS         4     //qnt. of lcd rows
#define LCD_COLUMNS      20    //qnt. of lcd columns
#define DEGREE_SYMBOL    0xDF  //degree symbol from the LCD ROM
#define LCD_SPACE_SYMBOL 0x20  //space  symbol from lcd ROM
#define LED              1     //connect led to ATtiny85 pin no.6 in series with 470 Ohm resistor

uint8_t temperature_icon[8] = {0x04, 0x0E, 0x0E, 0x0E, 0x0E, 0x1F, 0x1F, 0x0E};
uint8_t pressure_icon[8]    = {0x04, 0x04, 0x15, 0x0E, 0x04, 0x00, 0x00, 0x1F};
uint8_t plus_minus_icon[8]  = {0x00, 0x04, 0x0E, 0x04, 0x00, 0x0E, 0x00, 0x00};

/*
BMP085_BMP180(resolution)

resolution:
BMP085_ULTRALOWPOWER
BMP085_STANDARD
BMP085_HIGHRES
BMP085_ULTRAHIGHRES  (by default)
*/
BMP085_BMP180     myBMP(BMP085_ULTRAHIGHRES);
LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);

void setup()
{
  pinMode(LED, OUTPUT);

  /* LCD connection check */ 
  while (lcd.begin(LCD_COLUMNS, LCD_ROWS) != 1) //20x4 display
  {
    for (uint8_t i = 0; i > 3; i++)             //3 blinks if PCF8574/LCD is not connected or lcd pins declaration is wrong
    {
      digitalWrite(LED, HIGH);
      delay(300);
      digitalWrite(LED, LOW);
      delay(300);
    }
    delay(2000);
  }

  /* BMP085 or BMP180 connection check */
  while (myBMP.begin() != true)
  {
    lcd.setCursor(0, 0);
    lcd.print("BMP085 error");
    delay(5000);
  }
  lcd.clear();

  lcd.print("BMP085 OK");
  delay(2000);
  lcd.clear();

  lcd.createChar(0, temperature_icon);
  lcd.createChar(1, pressure_icon);
  lcd.createChar(2, plus_minus_icon);
}
  
void loop()
{
  lcd.setCursor(0, 0);                          //set 1-st colum & 1-st row. NOTE: 1-st colum & row started at zero
  lcd.write((uint8_t)0);                        //print custom tempereture symbol
  lcd.print(myBMP.readTemperature(), 1);
  lcd.write((uint8_t)2);                        //print custom plus/minus symbol
  lcd.print("1.0");
  lcd.write(DEGREE_SYMBOL);                     //print degree symbol from the ROM
  lcd.print("C");
  lcd.write(LCD_SPACE_SYMBOL);

  lcd.setCursor(0, 1);                          //set 1-st colum & 2-nd row. NOTE: 1-st colum & row started at zero
  lcd.write((uint8_t)1);                        //print custom pressure symbol
  lcd.print(myBMP.readPressure());
  lcd.write((uint8_t)2);                        //print custom plus/minus symbol
  lcd.print(F("150Pa"));
  lcd.write(LCD_SPACE_SYMBOL);

  delay(5000); 
}
