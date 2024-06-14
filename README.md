# CARECitizenver1

## Description
CARE Citizen is a project that develops an air quality monitoring system consisting of an open hardware node and a cloud platform in order to increase the number of data points by encouraging the general public to be a data contributor through citizen science. 

This page contains the build instructions for the CARE Citizen node which measures Relative Humidity and Temperature (RH/T), PM2.5, PM10, and CO2. The data gathered is sent to the CARE Citizen platform which stores the data and allows it to be visualized on a dashboard with charts and maps. This project abides by the open hardware principle as such the schematic, firmware, and 3D mounting files, etc. are freely available for the public.

## Table of Contents
- [Installation Arduino](#InstallationArduinoVer)
- [Installation ESP-IDF](#InstallationESP-IDFVer)


## Installation Arduino Ver

1. Install Arduino IDE on their website.
2. Upon installation of the IDE, in Boards Manager, install esp32 by Espressif Systems.
3. In Library Manager, install the following libraries:
        - Sensirion 12C SEN5X by Sensirion
        - PubSubClient by Nick O’Leary
        - MH-Z19 by Jonathan Dempsey
        - TinyGPSPlus by Mikal Hart
        - WiFiManager by tzapu
        - Arduinojson by Benoit Blanchon
        - ESPSoftwareSerial by Dirk Kaar, Peter Lerup
4. Configure Access_Key to your access key in User Information

## Installation ESP-IDF Ver
1. Install Espressif IDE including ESP-IDF and all dependencies.
2. Build the code onto the launch target, esp32c3.
3. Libraries included in the code were from:
        **a. Example ESP-IDF WiFi Provisioning Manager**
        - Example ESP-IDF NMEA GPS Parser  
        - Example ESP-IDF SD SPI   
        - Example ESP-IDF MQTT-TLS
        - Sensirion SEN5x driver (https://github.com/Sensirion/embedded-i2c-sen5x)
        - MHZ19 driver (https://github.com/crisap94/MHZ19/tree/master)
4. Configure Access_Key to your access key in User Information

