#!/usr/bin/python
# -*- coding: utf-8 -*-
"""PiPyADC: Example file for class ADS1256 in module pipyadc:

ADS1256 cycling through eight input channels.

Default data rate changed to 100 SPS. Check if hardware is connected.
Moving average filter over 32 samples.

Reading ADC sample data directly into a Numpy array as a buffer
for further processing, e.g. FIR filter, PID control, ...

Hardware: Waveshare ADS1256 board interfaced to the Raspberry Pi 3
 
Ulrich Lukas 2017-03-10
"""
import sys
import time
import numpy as np
import itertools
import pigpio as io
from ADS1256_definitions import *
from pipyadc import ADS1256
# In this example, we pretend myconfig_2 was a different configuration file
# named "myconfig_2.py" for a second ADS1256 chip connected to the SPI bus.
import device1_config
import device2_config

### START EXAMPLE ###
################################################################################
###  STEP 0: CONFIGURE CHANNELS AND USE DEFAULT OPTIONS FROM CONFIG FILE: ###
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
# Input pin for the potentiometer on the Waveshare Precision ADC board:
POTI = POS_AIN0|NEG_AINCOM
# Light dependant resistor of the same board:
LDR  = POS_AIN1|NEG_AINCOM
# The other external input screw terminals of the Waveshare board:
EXT2, EXT3, EXT4 = POS_AIN2|NEG_AINCOM, POS_AIN3|NEG_AINCOM, POS_AIN4|NEG_AINCOM
EXT5, EXT6, EXT7 = POS_AIN5|NEG_AINCOM, POS_AIN6|NEG_AINCOM, POS_AIN7|NEG_AINCOM

# You can connect any pin as well to the positive as to the negative ADC input.
# The following reads the voltage of the potentiometer with negative polarity.
# The ADC reading should be identical to that of the POTI channel, but negative.
POTI_INVERTED = POS_AINCOM|NEG_AIN0

# For fun, connect both ADC inputs to the same physical input pin.
# The ADC should always read a value close to zero for this.
SHORT_CIRCUIT = POS_AIN0|NEG_AIN0

# Specify here an arbitrary length list (tuple) of arbitrary input channel pair
# eight-bit code values to scan sequentially from index 0 to last.
# Eight channels fit on the screen nicely for this example..
CH_SEQUENCE = (POTI, LDR, EXT2, EXT3, EXT4, EXT7, POTI_INVERTED, SHORT_CIRCUIT)
################################################################################

##########################  CALIBRATION  CONSTANTS  ############################
# This shows how to use individual channel calibration values.
#
# The ADS1256 has internal gain and offset calibration registers, but these are
# applied to all channels without making any difference.
# I we want to use individual calibration values, e.g. to compensate external
# circuitry parasitics, we can do this very easily in software.
# The following values are only for demonstration and have no meaning.
CH_OFFSET = np.array((-10,   0, -85,   0, 750,   0,   0,   0), dtype=np.int)
GAIN_CAL  = np.array((1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0), dtype=np.float)
################################################################################



#pi = io.pi()

def do_measurement():
    ### STEP 1: Initialise ADC objects for two chips connected to the SPI bus.
    ads1 = ADS1256(device1_config)
#    ads1 = ADS1256(device1_config, pi)
    # (Note1: See ADS1256_default_config.py, see ADS1256 datasheet)
    # (Note2: Input buffer on means limited voltage range 0V...3V for 5V supply)
    ads2 = ADS1256(device2_config)
#    ads2 = ADS1256(device2_config, pi)

    # Just as an example: Change the default sample rate of the ADS1256:
    # This shows how to acces ADS1256 registers via instance property
    ads1.drate = DRATE_100
    ads2.drate = DRATE_100

    ### STEP 2: Gain and offset self-calibration:
    ads1.cal_self()
    ads2.cal_self()

    # Channel gain must be multiplied by LSB weight in volts per digit to
    # display each channels input voltage. The result is a np.array again here:
    CH_GAIN = ads2.v_per_digit * GAIN_CAL
    columns = len(CH_SEQUENCE)
    buffer1 = np.zeros((columns), dtype=np.int)
    buffer2 = np.zeros((columns), dtype=np.int)
    
    # From now, update filter_buffer cyclically with new ADC samples and
    # display results
    timestamp = time.time() # Limit output data rate to fixed time interval
    while True:
        #
        # Do the data acquisition of eight multiplexed input channels
        #
        # The result channel values are directy read into the array specified
        # as the second argument, which must be a mutable type.
        ads1.read_sequence(CH_SEQUENCE, buffer1)
        ads2.read_sequence(CH_SEQUENCE, buffer2)
    
        elapsed = time.time() - timestamp
        if elapsed > 1:
            timestamp += 1

            # Calculate moving average of input samples, subtract offset
            ch_unscaled = buffer1 - CH_OFFSET
            ch_unscaled2 = buffer2 - CH_OFFSET
            ch_volts = ch_unscaled * CH_GAIN
            ch_volts2 = ch_unscaled2 * CH_GAIN

            nice_output([int(i) for i in ch_unscaled], ch_volts,
                        [int(i) for i in ch_unscaled2], ch_volts2
                        )

### END EXAMPLE ###


#############################################################################
# Format nice looking text output:
def nice_output(digits, volts, digits2, volts2):
    sys.stdout.write(
          "\0337" # Store cursor position
          "\n\n"
        +
"""These are the raw sample values for the ADC 1 channels:
Poti_CH0,  LDR_CH1,     AIN2,     AIN3,     AIN4,     AIN7, Poti NEG, Short 0V
"""
        + ", ".join(["{: 8d}".format(i) for i in digits])
        +
"""

These are the raw sample values for the ADC 2 channels:
Poti_CH0,  LDR_CH1,     AIN2,     AIN3,     AIN4,     AIN7, Poti NEG, Short 0V
"""
        + ", ".join(["{: 8d}".format(i) for i in digits2])
        +
"""


These are the sample values converted to voltage in V for the ADC 1 channels:
Poti_CH0,  LDR_CH1,     AIN2,     AIN3,     AIN4,     AIN7, Poti NEG, Short 0V
"""
        + ", ".join(["{: 8.3f}".format(i) for i in volts])
        +
"""

These are the sample values converted to voltage in V for the ADC 2 channels:
Poti_CH0,  LDR_CH1,     AIN2,     AIN3,     AIN4,     AIN7, Poti NEG, Short 0V
"""
        + ", ".join(["{: 8.3f}".format(i) for i in volts2])
        + "\n\033[J\0338" # Restore cursor position etc.
    )



# Start data acquisition
try:
    print("\033[2J\033[H") # Clear screen
    print(__doc__)
    print("\nPress CTRL-C to exit.")
    do_measurement()

except (KeyboardInterrupt):
    print("\n"*18 + "User exit.\n")
#    pi.stop()
 
