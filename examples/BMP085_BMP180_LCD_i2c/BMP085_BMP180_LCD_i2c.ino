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
#include <Wire.h>
#include <BMP085_BMP180.h>
#include <LiquidCrystal_I2C.h>  //https://github.com/enjoyneering/LiquidCrystal_I2C

#define LCD_ROWS           4    //qnt. of lcd rows
#define LCD_COLUMNS        20   //qnt. of lcd columns
#define LCD_DEGREE_SYMBOL  0xDF //degree symbol from lcd ROM
#define LCD_SPACE_SYMBOL   0x20 //space  symbol from lcd ROM

#define MAX_TEMPERATURE    35   //max temp, deg.C

uint8_t icon_pressure[8]     = {0x04, 0x04, 0x15, 0x0E, 0x04, 0x00, 0x00, 0x1F};
uint8_t icon_see_pressure[8] = {0x04, 0x04, 0x15, 0x0E, 0x04, 0x00, 0x0A, 0x15};
uint8_t icon_temperature[8]  = {0x04, 0x0E, 0x0E, 0x0E, 0x0E, 0x1F, 0x1F, 0x0E};
float   temperature          = 0;

/*
MP085_BMP180(resolution)

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
  Serial.begin(115200);

  /* LCD connection check */  
  while (lcd.begin(LCD_COLUMNS, LCD_ROWS, LCD_5x8DOTS) != true) //20x4 display, LCD_5x8DOTS pixels size, SDA - D2, SCL - D1
  {
    Serial.println("PCF8574 is not connected or lcd pins declaration is wrong. Only pins numbers: 4,5,6,16,11,12,13,14 are legal.");
    delay(5000);
  }
  lcd.print("PCF8574 is OK");
  delay(1000);
  lcd.clear();

  /* BMP085/BMP180 connection check */
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

  /* load custom symbol to CGRAM */
  lcd.createChar(0, icon_temperature);
  lcd.createChar(1, icon_pressure);
  lcd.createChar(2, icon_see_pressure);

  /* prints static text */
  lcd.setCursor(0, 0);
    lcd.write((uint8_t)0);

  lcd.setCursor(0, 1);
    lcd.write((uint8_t)1);

  lcd.setCursor(0, 2);
    lcd.write((uint8_t)2);
}

void loop()
{
  temperature = myBMP.readTemperature();

  lcd.setCursor(2, 0);
    lcd.print(temperature, 1);
    lcd.write(LCD_DEGREE_SYMBOL);
    lcd.print("C");
    lcd.write(LCD_SPACE_SYMBOL);

  lcd.setCursor(2, 1);
    lcd.print(myBMP.getHectoPascalPressure());
    lcd.print("hPa");
    lcd.write(LCD_SPACE_SYMBOL);

  lcd.setCursor(2, 2);
    lcd.print(myBMP.getSeaLevelPressure(80) / 100);
    lcd.print("hPa");
    lcd.write(LCD_SPACE_SYMBOL);

    lcd.printHorizontalGraph('T', 3, temperature, MAX_TEMPERATURE); //name of the bar, 3-rd row, current value, max. value

  delay(5000);
}
