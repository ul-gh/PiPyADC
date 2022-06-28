#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""PiPyADC: Example file for class ADS1256 in module pipyadc:

Two ADS1256 devices on the same SPI bus,
cycling through eight input channels each.

Hardware: Isoflux ADS1256 board interfaced to the Raspberry Pi 2B, 3B or 4
 
Ulrich Lukas 2022-06-28
"""
import sys
import time
import numpy as np
import pigpio as io
from pipyadc.ADS1256_definitions import *
from pipyadc import ADS1256

# Two config files for different ADS1256 devices connected to the same SPI bus
import device1_config
import device2_config

### START EXAMPLE #############################################################

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
# The other external input screw terminals of the Waveshare board:
CH0 = POS_AIN0 | NEG_AINCOM
CH1 = POS_AIN1 | NEG_AINCOM
CH2 = POS_AIN2 | NEG_AINCOM
CH3 = POS_AIN3 | NEG_AINCOM
CH4 = POS_AIN4 | NEG_AINCOM
CH5 = POS_AIN5 | NEG_AINCOM
CH6 = POS_AIN6 | NEG_AINCOM
CH7 = POS_AIN7 | NEG_AINCOM

# Arbitrary length tuple of input channel pair values to scan sequentially.
CH_SEQUENCE = CH0, CH1, CH2, CH3, CH4, CH5, CH6, CH7

# PIGPIO instance, this is later supplied as an argument to ADS1256().
# This is optional as the ADS1256 class will create it when needed.
pi = io.pi()

def do_measurement():
    ### STEP 1: Initialise ADC objects for two chips connected to the SPI bus.
    ads1 = ADS1256(device1_config, pi)
    ads2 = ADS1256(device2_config, pi)

    # The ADC instances can also be configured at run-time
    ads1.drate = DRATE_100
    ads2.drate = DRATE_100

    ### STEP 2: Gain and offset self-calibration:
    ads1.cal_self()
    ads2.cal_self()

    # Channel gain must be multiplied by LSB weight in volts per digit to
    # display each channels input voltage. The result is a np.array again here:
    buffer1 = np.zeros(len(CH_SEQUENCE), dtype=np.int)
    buffer2 = np.zeros(len(CH_SEQUENCE), dtype=np.int)

    print("\n" * 13) # Reserve blank lines where output text is later placed
    timestamp = time.time() # Limit output data rate to fixed time interval

    # Endless loop reading into buffer and displaying results
    while True:
        # The result channel values are directy read into the array specified
        # as the second argument, which must be a mutable type.
        ads1.read_sequence(CH_SEQUENCE, buffer1)
        ads2.read_sequence(CH_SEQUENCE, buffer2)
        elapsed = time.time() - timestamp
        if elapsed > 1:
            timestamp += 1
            # Calculate moving average of input samples, subtract offset
            ads1_volts = buffer1 * ads1.v_per_digit
            ads2_volts = buffer2 * ads2.v_per_digit
            # Text-mode output
            nice_output(buffer1, ads1_volts,
                        buffer2, ads2_volts,
                        )
### END EXAMPLE ###############################################################


# Format nice looking text output:
def nice_output(digits, volts, digits2, volts2):
    sys.stdout.write(
        "\x1B[J\x1B[13F" # Clear and jump back 13 lines
        +
"""These are the raw sample values for the ADC 1 channels:
    AIN0,     AIN1,     AIN2,     AIN3,     AIN4,     AIN5,     AIN6,     AIN7
"""
        + ", ".join(["{: 8d}".format(i) for i in digits])
        +
"""

These are the raw sample values for the ADC 2 channels:
"""
        + ", ".join(["{: 8d}".format(i) for i in digits2])
        +
"""


These are the sample values converted to voltage in V for the ADC 1 channels:
    AIN0,     AIN1,     AIN2,     AIN3,     AIN4,     AIN5,     AIN6,     AIN7
"""
        + ", ".join(["{: 8.3f}".format(i) for i in volts])
        +
"""

These are the sample values converted to voltage in V for the ADC 2 channels:
"""
        + ", ".join(["{: 8.3f}".format(i) for i in volts2])
    )


# Startup and stop handling for this script
try:
    print("\x1B[2J\x1B[H") # Clear screen
    print(__doc__)
    print("\nPress CTRL-C to exit.\n")
    do_measurement()

except (KeyboardInterrupt):
    print("\nUser exit.\n")
    pi.stop()
 
