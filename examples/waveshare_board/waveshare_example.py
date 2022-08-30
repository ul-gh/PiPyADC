#!/usr/bin/env python3
"""Example file for class ADS1256 in pipyadc package

ADS1256 AD-converter cycling through eight input channels.

Hardware: Waveshare "High Precision AD/DA" board
interfaced to the Raspberry Pi 2B, 3B or 4
 
Ulrich Lukas 2022-06-28
"""
import time
import logging
from pipyadc import ADS1256
from pipyadc.utils import TextScreen
from pipyadc.ADS1256_definitions import *
# Config file for hardware setup of the waveshare ADS1256 board
import waveshare_config

logging.basicConfig(level=logging.DEBUG)

print("\x1B[2J\x1B[H") # Clear screen
print(__doc__)
print("\nPress CTRL-C to exit.\n")

# For in-place text-mode output
screen = TextScreen()

def text_format_8_ch(digits, volts):
    digits_str = ", ".join([f"{i: 8d}" for i in digits])
    volts_str = ", ".join([f"{i: 8.3f}" for i in volts])
    text = ("    AIN0,     AIN1,     AIN2,     AIN3, "
            "    AIN4,     AIN5,     AIN6,     AIN7\n"
            f"{digits_str}\n\n"
            "Values converted to volts:\n"
            f"{volts_str}\n"
            )
    return text


### START EXAMPLE ###########################################################
#
######  STEP 0: Configure channels
# For channel code values (bitmask) definitions, see ADS1256_definitions.py.
# The values representing the negative and positive input pins connected to
# the ADS1256 hardware multiplexer must be bitwise OR-ed to form eight-bit
# values, which will later be sent to the ADS1256 MUX register. The register
# can be explicitly read and set via ADS1256.mux property, but here we define
# a list of differential channels to be input to the ADS1256.read_sequence()
# method which reads all of them one after another.
#
# ==> Each channel in this context represents a differential pair of physical
# input pins of the ADS1256 input multiplexer.
#
# ==> For single-ended measurements, select AINCOM as the negative input.
#
# AINCOM does not have to be connected to AGND (0V), but it is if the jumper
# on the Waveshare board is set.
POTI = POS_AIN0 | NEG_AINCOM
LDR = POS_AIN1 | NEG_AINCOM
CH2 = POS_AIN2 | NEG_AINCOM
CH3 = POS_AIN3 | NEG_AINCOM
CH4 = POS_AIN4 | NEG_AINCOM
CH5 = POS_AIN5 | NEG_AINCOM
CH6 = POS_AIN6 | NEG_AINCOM
CH7 = POS_AIN7 | NEG_AINCOM

# Arbitrary length tuple of input channel pair values to scan sequentially
CH_SEQUENCE = POTI, LDR, CH2, CH3, CH4, CH5, CH6, CH7

def loop_forever_measurements(ads):
    while True:
        # Returns list of integers, one result for each configured channel
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        # Text-mode output
        voltages = [i * ads.v_per_digit for i in raw_channels]
        screen.put(text_format_8_ch(raw_channels, voltages))
        screen.refresh()
        time.sleep(0.5)

try:
    ###### STEP 1: ADS1256 now supports the context-manager API. [*]
    # Use this to have ADS1256 automatically close the SPI device and
    # pigpio resources at exit:
    with ADS1256(waveshare_config) as ads:
        ###### STEP 2: Configuration and control by setting ADS1256 properties:
        ads.drate = DRATE_100
        # Gain and offset self-calibration can be triggered at any time
        ads.cal_self()
        ######  STEP 3: Get and process data
        loop_forever_measurements(ads)

except KeyboardInterrupt:
    print("\nUser Exit.\n")

### END EXAMPLE #############################################################

# [*]: New in version 2.1.
# Alternatively, ADS1256 now provides the stop() and stop_close_all()
# methods which should be called at exit for manual clean-up.