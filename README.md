# Bosch BMP180 & BMP085
This is an Arduino library for Bosch BMP180 & BMP085 barometric pressure & temperature sensor

Range                 typ. resolution   typ. accuracy   typ. relative accuracy
30,000Pa..110,000Pa   ±1Pa              ±100Pa          ±12Pa
0°C..+65°C            ±0.1°C            ±1.0°C

Supports *all sensors features & more:

- Read & calculate compensated temperature, in °C
- Read & calculate compensated barometric pressure, in Pa
- Reset
- Read chip ID
- Read chip FW version

- Calculate sea level pressure, in Pa
- Convert barometric pressure to hPa, mmHg, inHg
- Convert sea level pressure to hPa, mmHg, inHg
- Simple forecast: thunderstorm, rain, cloudy, partly cloudy, clear, sunny


Tested on:

- Arduino AVR
- Arduino ESP8266
- Arduino STM32

* EOC  pin is not used, shows the end of conversion
* XCLR pin is not used, reset pin
