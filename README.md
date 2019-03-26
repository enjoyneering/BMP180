[![license-badge][]][license] ![version] [![stars][]][stargazers] [![GitHub issues][]][issues]

# Bosch BMP180 & BMP085
This is an Arduino library for Bosch BMP180 & BMP085 barometric pressure & temperature sensor

- Supply voltage:         1.8v - 3.6v
- Range:                  30,000Pa..110,000Pa at 0°C..+65°C
- Typ. resolution:        1Pa    / 0.1°C
- Typ. accuracy:          ±100Pa / ±1.0°C
- Typ. relative accuracy: ±12Pa  / xx°C

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

*EOC  pin is not used, shows the end of conversion

*XCLR pin is not used, reset pin

[license-badge]: https://img.shields.io/badge/License-GPLv3-blue.svg
[license]:       https://choosealicense.com/licenses/gpl-3.0/
[version]:       https://img.shields.io/badge/Version-1.2.1-green.svg
[stars]:         https://img.shields.io/github/stars/enjoyneering/BMP180.svg
[stargazers]:    https://github.com/enjoyneering/BMP180/stargazers
[GitHub issues]: https://img.shields.io/github/issues/enjoyneering/BMP180.svg
[issues]:        https://github.com/enjoyneering/BMP180/issues/
