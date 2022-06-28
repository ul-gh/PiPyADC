# PiPyADC
Python module for interfacing Texas Instruments ADS1255 and ADS1256
SPI bus based analog-to-digital converters with the Raspberry Pi.

This is a Python-based implementation using the pigpio Python interface
to access the Raspberry Pi hardware through the pigpio system service.

On a Raspberry Pi 3, multi-channel ADC reads require a time overhead in
the order of one millisecond per sample which makes this library suitable
for low-speed, high resolution data acquisition.

As the ADS125x devices feature a sophisticated, configurable, hardware-based
down-sampling filter, there is no penalty in terms of accuracy.

NEW in version 2:
* Uses the pigpio API instead of the deprecated wiringpi library
* Using pgpio, the user scripts do not have to be run as root any more
* Working implementation for multiple ADS125x devices on the same SPI bus
* Proper configuration of unused chip-select line for the Waveshare example

Limitation:
* Same as for version 1: This does not feature high-speed or low delays

Download: https://github.com/ul-gh/PiPyADC

Depends on pigpio library, see: https://abyz.me.uk/rpi/pigpio/python.html
Uses code from: https://github.com/heathsd/PyADS1256

License: GNU LGPLv2.1, see:
https://www.gnu.org/licenses/old-licenses/lgpl-2.1-standalone.html

Ulrich Lukas, 2022-06-28

## Run example on Raspberry Pi OS:
### Install:
	# numpy is optional for running the Isoflux hardware examples
	sudo apt install python3-pip python3-numpy pigpio python3-pigpio
	pip3 install pipyadc
### Enable pigpio system service (started at boot)
	sudo systemctl enable pigpio.service
### Activate SPI bus and reboot system
	# Using raspi-config or, alternatively, using following command:
	sudo sed -E -i s/"(#)(dtparam=spi).*"/"\2=on"/ /boot/config.txt
	sudo reboot
### Run example:
	cd examples/waveshare_board
	./waveshare_example.py


