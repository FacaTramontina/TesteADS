#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_ADS1X15.h"

Adafruit_ADS1115 ads;
float Voltage = 0.0;

void setup(void) 
{
Serial.begin(9600); 
ads.begin();
}

void loop(void) 
{
int16_t adc0;

adc0 = ads.readADC_SingleEnded(0);
Voltage = (adc0 * 0.1875)/1000;

Serial.print("AIN0: "); 
Serial.print(adc0);
Serial.print("\tVoltage: ");
Serial.println(Voltage, 7); 
Serial.println();

delay(100);
}

/**

https://microcontrollerslab.com/ads1115-external-adc-with-esp32/

I found the way to connect up to four ADS1115 to the same I2C connection.

This TI pdf tells how to do it.
https://www.ti.com/lit/ds/symlink/ads1114.pdf

Here is a short form “how to”:
9.5.1.1
I2C Address Selection
The ADS111x have one address pin, ADDR, that configures the I2C address of the device.
This pin can be connected to GND, VDD, SDA, or SCL, allowing for four different addresses to be selected with one pin, as
shown in Table 4.
The state of address pin ADDR is sampled continuously. Use the GND, VDD and SCL addresses first.
If SDA is used as the device address, hold the SDA line low for at least 100 ns after the SCL line goes low to make sure the device decodes the address correctly during I2C communication.
Table 4.
ADDR Pin Connection and Corresponding Slave Address
ADDR PIN CONNECTION SLAVE ADDRESS
GND 1001000 (48)
VDD 1001001 (49)
SDA 1001010 (50)
SCL 1001011 (51)


no plat.ini

[env:esp32doit-devkit-v1]
platform = espressif32
board = esp32doit-devkit-v1
framework = arduino
lib_deps =
  adafruit/Adafruit BusIO
  SPI
  adafruit/Adafruit ADS1X15


**/