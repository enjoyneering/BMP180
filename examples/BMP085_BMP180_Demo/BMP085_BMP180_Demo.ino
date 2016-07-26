/**************************************************************************/
/* 
  These sensor uses I2C to communicate. Two pins are required to  
  interface.

  Connect BMP085/BMP180 to pins:  SDA  SCL
  Uno, Mini, Pro:                 A4      A5
  Mega2560, Due:                  20      21
  Leonardo:                       2       3
  Atiny85:                        0/PWM   2/A1 (TinyWireM)
  NodeMCU 1.0:                    ANY     ANY

  NOTE: EOC pin  is not used, it signifies at end of conversion.
        XCLR pin is a reset pin, also not used.

  written by Enjoyneering79

  BSD license, all text above must be included in any redistribution
*/
 /**************************************************************************/
#if defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny26__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny45__) || (__AVR_ATtiny84__) || defined(__AVR_ATtiny85__)
 #include <TinyWireM.h>
 #define Wire TinyWireM
#else defined
 #include <Wire.h>
#endif

#include <BMP085_BMP180.h>

/*
BMP085_BMP180(resolution)

Resolution:
BMP_ULTRALOWPOWER
BMP_STANDARD
BMP_HIGHRES
BMP_ULTRAHIGHRES

Default:
BMP085_BMP180(BMP_ULTRAHIGHRES)
*/

BMP085_BMP180 myBMP;
  
void setup()
{
  Serial.begin(115200);
  Serial.println(F(""));

 #if defined(ARDUINO_ARCH_ESP8266) || (ESP8266_NODEMCU)
  while (myBMP.begin(D1, D2) != true)
 #else
  while (myBMP.begin() != true
 #endif
  {
    Serial.println(F("BOSCH BMP085/BMP180 sensor is not present..."));
    delay(5000);
  }
  
  Serial.println(F("BOSCH BMP085/BMP180 sensor is present"));
}
  
void loop()
{
  Serial.print(F("Temperature = "));
  Serial.print(myBMP.readTemperature());
  Serial.println(F(" +-1.0 deg.C"));
    
  Serial.print(F("Pressure = "));
  Serial.print(myBMP.readPressure());
  Serial.println(F(" +-100Pa"));
    
  /* calculate Altitude assuming Sea Level Pressure is 100500 Pa */
  /* you can get more accurate Altitude if you know your current Sea Level Pressure. */
  /* NOTE: Sea Level Pressure is vary with weather and such... */
  Serial.print(F("Altitude (calculated) = "));
  Serial.print(myBMP.calculateAltitude(100500));
  Serial.println(F(" meters"));

  /* calculate Sea Level Pressure assuming your current altitude is 150 meters */
  /* you can get more accurate Sea Level Pressure if you know your current Altitude. */
  Serial.print(F("Pressure at sealevel (calculated) = "));
  Serial.print(myBMP.calculateSeaLevelPressure(150));
  Serial.println(F(" Pa"));

  Serial.print(F("Starts over again in 8 sec."));
  delay(8000);  
}
