/**************************************************************************/
/* 
  This is an Arduino library for the BOSCH BMP085/BMP180 Barometric Pressure
  and Temperature sensor

  written by Enjoyneering79

  These sensor uses I2C to communicate, 2 pins are required to  
  interface

  Connect BMP085/BMP180 to pins:  SDA  SCL
  Uno, Mini, Pro:                 A4      A5
  Mega2560, Due:                  20      21
  Leonardo:                       2       3
  Atiny85:                        0/PWM   2/A1  (TinyWireM)
  NodeMCU 1.0:                    4/ANY   5/ANY (4 & 5 by default)
  ESP8266 ESP-01:                 ANY     ANY

  BSD license, all text above must be included in any redistribution
*/
 /**************************************************************************/
  
#if ARDUINO >= 100
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#if defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny26__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny45__) || (__AVR_ATtiny84__) || defined(__AVR_ATtiny85__)
 #include <TinyWireM.h>
 #define Wire TinyWireM
#elif defined (ARDUINO_ARCH_ESP8266) || (ESP8266_NODEMCU)
 #include <Wire.h>
#else defined
 #include <avr/pgmspace.h>
 #include <Wire.h>
#endif


#define BMP_ADDRESS           0x77 /* BMP085 & BMP180 Address on i2c serial bus */

#define BMP_CHIP_ID           0x55 /* BMP085 & BMP180 ID number */

#define BMP_ULTRALOWPOWER     0
#define BMP_STANDARD          1
#define BMP_HIGHRES           2
#define BMP_ULTRAHIGHRES      3

#define BMP_CAL_AC1           0xAA  /* Register: Calibration data (16 bits) */
#define BMP_CAL_AC2           0xAC  /* Register: Calibration data (16 bits) */
#define BMP_CAL_AC3           0xAE  /* Register: Calibration data (16 bits) */   
#define BMP_CAL_AC4           0xB0  /* Register: Calibration data (16 bits) */
#define BMP_CAL_AC5           0xB2  /* Register: Calibration data (16 bits) */
#define BMP_CAL_AC6           0xB4  /* Register: Calibration data (16 bits) */
#define BMP_CAL_B1            0xB6  /* Register: Calibration data (16 bits) */
#define BMP_CAL_B2            0xB8  /* Register: Calibration data (16 bits) */
#define BMP_CAL_MB            0xBA  /* Register: Calibration data (16 bits) */
#define BMP_CAL_MC            0xBC  /* Register: Calibration data (16 bits) */
#define BMP_CAL_MD            0xBE  /* Register: Calibration data (16 bits) */

#define BMP_ID                0xD0  /* Register: Get Device ID */

#define BMP_CONTROL           0xF4 
#define BMP_TEMPDATA          0xF6
#define BMP_PRESSUREDATA      0xF6
#define BMP_READTEMPCMD       0x2E
#define BMP_READPRESSURECMD   0x34


class BMP085_BMP180
{
 public:
  BMP085_BMP180(uint8_t res_mode = BMP_ULTRAHIGHRES);         /* by default - BMP_ULTRAHIGHRES */

 #if defined (ARDUINO_ARCH_ESP8266) || (ESP8266_NODEMCU)
   bool   begin(uint8_t sda, uint8_t scl);
 #else
   bool   begin(void);
 #endif
  float   readTemperature(void);
  int32_t readPressure(void);
  int32_t calculateSeaLevelPressure(float altitudeMeters = 0); /* by default altitude = 0 meters */
  float   calculateAltitude(float sealevelPressure = 101325);  /* by default sea level pressure = 101325 Pa */

  
 private:
  boolean   _BMP_Initialised;
  int16_t   _ac1, _ac2, _ac3, _b1, _b2, _mb, _mc, _md;
  uint16_t  _ac4, _ac5, _ac6;
  uint8_t   _BMP_Resolution;
  #if defined (ARDUINO_ARCH_ESP8266) || (ESP8266_NODEMCU)
   uint8_t  _sda, _scl;
  #endif

  uint16_t  readRawTemperature(void);
  uint32_t  readRawPressure(void);
  int32_t   computeB5(int32_t UT);
  uint8_t   read8(uint8_t addr);
  uint16_t  read16(uint8_t addr);
  void      write8(uint8_t addr, uint8_t data);

};
