#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Example file for class ADS1256 in pipyadc package

ADS1256 AD-converter cycling through eight input channels.

Hardware: Waveshare "High Precision AD/DA" board
interfaced to the Raspberry Pi 2B, 3B or 4
 
Ulrich Lukas 2022-06-28
"""
import sys
import time
import pigpio as io
from pipyadc.ADS1256_definitions import *
from pipyadc import ADS1256
# Config file for hardware setup of the waveshare ADS1256 board
import waveshare_config

### START EXAMPLE ###########################################################

###  STEP 0: Configure channels  ###
#
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
# ==> For single-ended measurements, simply select AINCOM as the negative input.
#
# AINCOM does not have to be connected to AGND (0V), but it is if the jumper
# on the Waveshare board is set.
#
POTI = POS_AIN0 | NEG_AINCOM
LDR = POS_AIN1 | NEG_AINCOM
CH2 = POS_AIN2 | NEG_AINCOM
CH3 = POS_AIN3 | NEG_AINCOM
CH4 = POS_AIN4 | NEG_AINCOM
CH5 = POS_AIN5 | NEG_AINCOM
CH6 = POS_AIN6 | NEG_AINCOM
CH7 = POS_AIN7 | NEG_AINCOM

# Arbitrary length tuple of input channel pair values to scan sequentially.
CH_SEQUENCE = POTI, LDR, CH2, CH3, CH4, CH5, CH6, CH7

def do_measurement():
    ###  STEP 1: Get ADS1256 instance  ###
    # This is initialized with imported config file
    ads = ADS1256(waveshare_config)
    # The ADC instance can also be configured at run time
    ads.drate = DRATE_100

    ### STEP 2: Gain and offset self-calibration:
    ads.cal_self()

    print("\n" * 6) # Reserve blank lines where output is later placed
    while True:
        ###  STEP 3: Get data  ###
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        voltages = [i * ads.v_per_digit for i in raw_channels]

        ### Done. Text-mode output: ###
        nice_output(raw_channels, voltages)
        time.sleep(0.5)

### END EXAMPLE #############################################################


# Format nice looking text output:
def nice_output(digits, volts):
    sys.stdout.write(
        "\x1B[J\x1B[6F" # Clear and jump back 6 lines
        +
"""These are the raw sample values for the ADC 1 channels:
    POTI,      LDR,     AIN2,     AIN3,     AIN4,     AIN5,     AIN6,     AIN7
"""
        + ", ".join(["{: 8d}".format(i) for i in digits])
        +
"""

These are the sample values converted to voltage in V for the ADC 1 channels:
    POTI,      LDR,     AIN2,     AIN3,     AIN4,     AIN5,     AIN6,     AIN7
"""
        + ", ".join(["{: 8.3f}".format(i) for i in volts])
    )


# Startup and exit handling for this script
try:
    print("\x1B[2J\x1B[H") # Clear screen
    print(__doc__)
    print("\nPress CTRL-C to exit.\n")
    do_measurement()

except (KeyboardInterrupt):
    print("\nUser exit.\n")
