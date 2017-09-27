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

/*
MP085_BMP180(resolution)

resolution:
BMP085_ULTRALOWPOWER
BMP085_STANDARD
BMP085_HIGHRES
BMP085_ULTRAHIGHRES  (by default)
*/
BMP085_BMP180 myBMP(BMP085_ULTRAHIGHRES);
  
void setup()
{
  Serial.begin(115200);

  while (myBMP.begin(D1, D2) != true)
  {
    Serial.println(F("BOSCH BMP085/BMP180 sensor is faild or not connected"));
    delay(5000);
  }
  
  Serial.println(F("BOSCH BMP085/BMP180 sensor is active"));
}
  
void loop()
{
  Serial.print(F("Temperature.......: ")); Serial.print(myBMP.readTemperature(), 1); Serial.println(F(" +-1.0C"));
  Serial.print(F("Pressure..........: ")); Serial.print(myBMP.readPressure()); Serial.println(F(" +-150Pa"));
    
  /* 
     Calculate Altitude assuming the Sea Level Pressure is 100500 Pa.
     NOTE: - You can get more accurate Altitude if you know your current Sea Level Pressure.
           - The Sea Level Pressure depends on the weather and such...
  */
  Serial.print(F("Altitude..........:")); Serial.print(myBMP.getAltitude(100500)); Serial.println(F(" meters"));

  /* 
     Calculate The Sea Level Pressure assuming your current altitude is 80 meters
     NOTE: - You can get more accurate Sea Level Pressure if you know your current Altitude.
  */
  Serial.print(F("See level pressure:")); Serial.print(myBMP.getSeaLevelPressure(80)); Serial.println(F(" Pa"));

  Serial.print(F("Starts over again in 8 sec."));
  delay(8000);  
}
