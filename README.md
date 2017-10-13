# LoRaWAN 32u4 II

This repo shows a quick, easy and cheap way to get the LoRa32u4 II board up and running using the LoRaWAN LMIC library.
The board used in the example is http://www.diymalls.com/index.php?route=product/product&path=74&product_id=88
But there several more to choose from, such as the Moteino Mega https://lowpowerlab.com/shop/product/119 or RocketScream Mini Ultra Pro http://www.rocketscream.com/blog/product/mini-ultra-pro-with-radio/ which both offers more program memory if needed. All three have been tested and works great.

There is a free Wire(I2C) interface and Hardware serial for example GPS. 6 free analog inputs 5+ free IO/PWM etc. The Hardware SPI interface used by RFM95 can be shared if needed.

NB! The code in this repo is geared toward use in EU, but the LMIC library supports US channels as well.

## Required Hardware connections or changes

LoRa Radio RFM95 in WAN mode with arduino-lmic requires the use of DIO0, DIO1 and DIO2.
DIO0 is already connected to ATMEGA32u4 on Arduino pin 7.
DIO1 and DI2 are available as solder points and just needs a short dupont jumper or solder to an unused pin on ATMEGA32u4.
This code uses Arduino pin 5 and 9, but it easy to change in the code (RFM95_DIO_1/RFM_DIO_2).
* Connect Bottom Right Solderpad (RFM95 DIO2) to Arduino pin 9
* Connect Pin DIO1 to Arduino pin 5

Example solder:
<img src="doc/fullboard.jpg" width="40%"><img src="doc/closeup.jpg" width="40%">

## Suggested Hardware modifications

Battery monitoring, the code includes reading battery voltage using A0. A simple voltage divider from BAT pin.
```
BAT-----1MOhm-----3MOhm-----GND
               | 
               ---0.1µF-----GND
               |
               A0
```
## Suggested Prerequisities

### Install IDE

* Atom (https://atom.io/)
* PlatformIO (https://atom.io/packages/platformio-ide)

### Install Arduino libraries
Into libraries folder ex. ~/code/arduino/libraries
* My fork of arduino-lmic (git clone https://github.com/Redferne/arduino-lmic.git)
* elapsedMillis (git clone http://github.com/pfeerick/elapsedMillis/wiki)

Clone this repo to your prefered source folder ex. ~/code/arduino
* (git clone https://github.com/Redferne/LoRaWAN_32u4_II.git)

Add path to your Platform.O (~/.platformio/penv/bin) to your $PATH

## Compile and Program board
* In PlatformIO -> Open Project -> LoraWAN_32u4_II
* Change your monitor_port/upload_port and lib_dir in platformio.ini
* Select Build / Upload / Serial Monitor from the PlatformIO GUI or use a terminal
```
$ pio run
$ pio run -t upload
$ pio run -t monitor
```
If the upload fails. Enter the bootloader by pressing the reset button once (or twice).
The board's white LED starts "breathing" when the bootloader is running.

Please see code for compile options. There are a few kilobytes left for sensor implementation :-)
