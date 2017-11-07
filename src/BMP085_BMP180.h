/***************************************************************************************************/
/* 
  This is an Arduino library for the BOSCH BMP085/BMP180 Barometric Pressure and Temperature sensor
  
  Range                 Max. Resolution   Max. Accuracy
  30,000Pa..110,000Pa   -+1Pa             -+150Pa
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

  BSD license, all text above must be included in any redistribution
*/
/***************************************************************************************************/

#ifndef BMP085_BMP180_h
#define BMP085_BMP180_h
  
#if ARDUINO >= 100 && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#if defined(__AVR_ATtinyX4__) || defined(__AVR_ATtinyX5__) || defined(__AVR_ATtinyX313__)
#include <TinyWireM.h>
#define  Wire TinyWireM
#else
#include <Wire.h>
#endif

#if defined(__AVR__)
#include <avr/pgmspace.h>
#elif defined(ESP8266)
#include <pgmspace.h>
#endif

//#define BMP085_DEBUG_INFO             //enable/disable serial debug info

#define BMP085_ADDRESS           0x77   //i2c address

#define BMP085_CHIP_ID           0x55   //ID number

/* registers */
#define BMP085_CAL_AC1           0xAA  //ac1 16-bit calibration data register
#define BMP085_CAL_AC2           0xAC  //ac2 16-bit calibration data register
#define BMP085_CAL_AC3           0xAE  //ac3 16-bit calibration data register
#define BMP085_CAL_AC4           0xB0  //ac4 16-bit calibration data register
#define BMP085_CAL_AC5           0xB2  //ac5 16-bit calibration data register
#define BMP085_CAL_AC6           0xB4  //ac6 16-bit calibration data register
#define BMP085_CAL_B1            0xB6  //b1  16-bit calibration data register
#define BMP085_CAL_B2            0xB8  //b2  16-bit calibration data register
#define BMP085_CAL_MB            0xBA  //mb  16-bit calibration data register
#define BMP085_CAL_MC            0xBC  //mc  16-bit calibration data register
#define BMP085_CAL_MD            0xBE  //md  16-bit calibration data register

#define BMP085_GET_ID            0xD0  //device ID register
#define BMP085_START_SOFT_RESET  0xE0  //soft reset register
#define BMP085_GET_SOFT_RESET    0xB6  //soft reset control

#define BMP085_START_MEASURMENT  0xF4  //start measurment register
#define BMP085_READ_RESULT       0xF6  //read  result register
#define BMP085_GET_TEMPERATURE   0x2E  //start temperature control
#define BMP085_GET_PRESSURE      0x34  //start pressure control

/* misc */
#define BMP085_ERROR             0xFF  //returns 255, if communication error is occurred
#define BMP085_POLL_LIMIT        8     //i2c retry limit

/* resolution and power controls */
typedef enum
{
  BMP085_ULTRALOWPOWER = 0x00,         //low power mod,
  BMP085_STANDARD      = 0x01,         //standard mod
  BMP085_HIGHRES       = 0x02,         //hi resesolution mode
  BMP085_ULTRAHIGHRES  = 0x03          //ultra hi resesolution mode
}
BMP085_RESOLUTION;

/* calibration coefficients */
typedef struct
{
  int16_t  BMP_AC1 = 0;
  int16_t  BMP_AC2 = 0;
  int16_t  BMP_AC3 = 0;
  uint16_t BMP_AC4 = 0;
  uint16_t BMP_AC5 = 0;
  uint16_t BMP_AC6 = 0;

  int16_t  BMP_B1 = 0;
  int16_t  BMP_B2 = 0;

  int16_t  BMP_MB = 0;
  int16_t  BMP_MC = 0;
  int16_t  BMP_MD = 0;
}
BMP085_CAL_COEFF;


class BMP085_BMP180
{
 public:
  BMP085_BMP180(BMP085_RESOLUTION = BMP085_ULTRAHIGHRES); //BMP085_ULTRAHIGHRES by default

  #if defined(ESP8266)
  bool    begin(uint8_t sda = SDA, uint8_t scl = SCL);
  #else
  bool    begin(void);
  #endif
  float   readTemperature(void);                          //in deg.C
  int32_t readPressure(void);                             //in Pa
  float   getHectoPascalPressure(void);                   //in hPa
  float   getMillimeterMercuryPressure(void);             //in mmHg
  int32_t getSeaLevelPressure(float altitude = 80);       //altitude is 80 meters by default
  float   getAltitude(float sealevelPressure);
  void    softReset(void);

  
 private:
  BMP085_CAL_COEFF _cal_coeff;

  uint8_t  _resolution;

  void     readCalibrationCoefficients(void);
  uint16_t readRawTemperature(void);
  uint32_t readRawPressure(void);
  int32_t  computeB5(int32_t UT);
  uint8_t  read8(uint8_t addr);
  uint16_t read16(uint8_t addr);
  void     write8(uint8_t addr, uint8_t data);
};

#endif
