# PiPyADC
Python module for interfacing Texas Instruments ADS1255 and ADS1256
SPI bus based analog-to-digital converters with the Raspberry Pi.

The ADS1255 and ADS1256 library is an implementation based purely on
Python. Since version 2, this uses the PIGPIO API since wiringpi was
deprecatred by its author before.

On a Raspberry Pi 3, multi-channel ADC reads require a time overhead
of aprox. 300...500 µs which makes the library suitable for
low-speed, high resolution data acquisition.

Download: https://github.com/ul-gh/PiPyADC

Depends on pigpio library, see: https://abyz.me.uk/rpi/pigpio/python.html
Uses code from: https://github.com/heathsd/PyADS1256

License: GNU LGPLv2.1, see:
https://www.gnu.org/licenses/old-licenses/lgpl-2.1-standalone.html

Ulrich Lukas, 2022-06-22

## Run example on Raspbian Stretch:
### Install wiringpi library:
	sudo apt install python-pip
	sudo pip install pipyadc
### Activate SPI bus and reboot system:
	sudo sed -E -i s/"(#)(dtparam=spi).*"/"\2=on"/ /boot/config.txt
	reboot
### Run example:
	cd examples/waveshare_board
	sudo python example.py


