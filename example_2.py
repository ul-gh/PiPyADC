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
from ADS1256_definitions import *
from pipyadc import ADS1256
# In this example, we pretend myconfig_2 was a different configuration file
# named "myconfig_2.py" for a second ADS1256 chip connected to the SPI bus.
import ADS1256_default_config as myconfig_2

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

# Using the Numpy library, digital signal processing is easy as (Raspberry) Pi..
# However, this constant only specifies the length of a moving average.
FILTER_SIZE = 32
################################################################################


def do_measurement():
    ### STEP 1: Initialise ADC objects for two chips connected to the SPI bus.
    # In this example, we pretend myconfig_2 was a different configuration file
    # named "myconfig_2.py" for a second ADS1256 chip connected to the SPI bus.
    # This file must be imported, see top of the this file.
    # Omitting the first chip here, as this is only an example.

    #ads1 = ADS1256(myconfig_1)
    # (Note1: See ADS1256_default_config.py, see ADS1256 datasheet)
    # (Note2: Input buffer on means limited voltage range 0V...3V for 5V supply)
    ads2 = ADS1256(myconfig_2)
    
    # Just as an example: Change the default sample rate of the ADS1256:
    # This shows how to acces ADS1256 registers via instance property
    ads2.drate = DRATE_100

    ### STEP 2: Gain and offset self-calibration:
    ads2.cal_self()

    ### Get ADC chip ID and check if chip is connected correctly.
    chip_ID = ads2.chip_ID
    print("\nADC No. 2 reported a numeric ID value of: {}.".format(chip_ID))
    # When the value is not correct, user code should exit here.
    if chip_ID != 3:
        print("\nRead incorrect chip ID for ADS1256. Is the hardware connected?")
    # Passing that step because this is an example:
    #    sys.exit(1)

    # Channel gain must be multiplied by LSB weight in volts per digit to
    # display each channels input voltage. The result is a np.array again here:
    CH_GAIN = ads2.v_per_digit * GAIN_CAL

    # Numpy 2D array as buffer for raw input samples. Each row is one complete
    # sequence of samples for eight input channel pin pairs. Each column stores
    # the number of FILTER_SIZE samples for each channel.
    rows, columns = FILTER_SIZE, len(CH_SEQUENCE)
    filter_buffer = np.zeros((rows, columns), dtype=np.int)
    
    # Fill the buffer first once before displaying continuously updated results
    print("Channels configured: {}\n"
          "Initializing filter (this can take a minute)...".format(
              len(CH_SEQUENCE)))
    for row_number, data_row in enumerate(filter_buffer):
        # Do the data acquisition of eight multiplexed input channels.
        # The ADS1256 read_sequence() method automatically fills into
        # the buffer specified as the second argument:
        ads2.read_sequence(CH_SEQUENCE, data_row)
        # Depending on aquisition speed and filter lenth, this can take long...
        sys.stdout.write(
            "\rProgress: {:3d}%".format(int(100*(row_number+1)/FILTER_SIZE)))
        sys.stdout.flush()


    # From now, update filter_buffer cyclically with new ADC samples and
    # calculate results with averaged results.
    print("\n\nOutput values averaged over {} ADC samples:".format(FILTER_SIZE))
    # The following is an endless loop!
    timestamp = time.time() # Limit output data rate to fixed time interval
    for data_row in itertools.cycle(filter_buffer):
        #
        # Do the data acquisition of eight multiplexed input channels
        #
        # The result channel values are directy read into the array specified
        # as the second argument, which must be a mutable type.
        ads2.read_sequence(CH_SEQUENCE, data_row)
    
        elapsed = time.time() - timestamp
        if elapsed > 1:
            timestamp += 1

            # Calculate moving average of input samples, subtract offset
            ch_unscaled = np.average(filter_buffer, axis=0) - CH_OFFSET
            ch_volts = ch_unscaled * CH_GAIN

            nice_output([int(i) for i in ch_unscaled], ch_volts)

### END EXAMPLE ###


#############################################################################
# Format nice looking text output:
def nice_output(digits, volts):
    sys.stdout.write(
          "\0337" # Store cursor position
        +
"""These are the raw sample values for the channels:
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
 
