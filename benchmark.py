#!/usr/bin/python
# -*- coding: utf-8 -*-
"""PiPyADC: Example file for class ADS1256 in module pipyadc:

ADS1256 timing.

Hardware: Waveshare ADS1256 board interfaced to the Raspberry Pi 3
 
Ulrich Lukas 2017-09-06
"""
import sys
import time
from ADS1256_definitions import *
from pipyadc import ADS1256
import bench_config as conf

# Input pin for the potentiometer on the Waveshare Precision ADC board:
POTI = POS_AIN0|NEG_AINCOM
# Light dependant resistor of the same board:
LDR  = POS_AIN1|NEG_AINCOM
# Eight channels
CH_SEQUENCE = (POTI, LDR)
################################################################################


def do_measurement():
    ### STEP 1: Initialise ADC object:
    ads = ADS1256(conf)
    n_loop = 1000

    ### STEP 2: Gain and offset self-calibration:
    ads.cal_self()

    timestamp1 = time.time()
    for a in range(1, n_loop):
        ### STEP 3: Get data:
        raw_channels = ads.read_continue(CH_SEQUENCE)
#        raw_channels = ads.read_sequence(CH_SEQUENCE)

    timestamp2 = time.time()

    voltages     = [i * ads.v_per_digit for i in raw_channels]
    nice_output(raw_channels, voltages)

    # Timing info
    print("\n"*9)
    delta = timestamp2 - timestamp1
    print("Delta seconds: {}".format(delta))
    per_sample = 1.0E6*delta/(n_loop*len(CH_SEQUENCE))
    print("Per sample microseconds: {}".format(per_sample))
    overhead = per_sample - 1.0e6/n_loop
    print("Overhead per sample microseconds: {}".format(overhead))



#############################################################################
# Format nice looking text output:
def nice_output(digits, volts):
    sys.stdout.write(
          "\0337" # Store cursor position
        +
"""
These are the raw sample values for the channels:
Poti_CH0,  LDR_CH1,     AIN2,     AIN3,     AIN4,     AIN7, Poti NEG, Short 0V
"""
        + ", ".join(["{: 8d}".format(i) for i in digits])
        +
"""

These are the sample values converted to voltage in V for the channels:
Poti_CH0,  LDR_CH1,     AIN2,     AIN3,     AIN4,     AIN7, Poti NEG, Short 0V
"""
        + ", ".join(["{: 8.3f}".format(i) for i in volts])
        + "\n\033[J\0338" # Restore cursor position etc.
    )


# Start data acquisition
try:
    print("\033[2J\033[H") # Clear screen
    print(__doc__)
    print("\nPress CTRL-C to exit.")
    do_measurement()

except (KeyboardInterrupt):
    print("\n"*8 + "User exit.\n")
    sys.exit(0)
