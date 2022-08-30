#!/usr/bin/env python3
"""PiPyADC: Example file for class ADS1256 in module pipyadc:

Two ADS1256 devices on the same SPI bus,
cycling through eight input channels each.

Hardware: Isoflux ADS1256 board interfaced to the Raspberry Pi 2B, 3B or 4
 
Ulrich Lukas 2022-06-28
"""
import time
import pigpio
import logging
import numpy as np
from pipyadc import ADS1256
from pipyadc.utils import TextScreen
from pipyadc.ADS1256_definitions import *

# Two config files for different ADS1256 devices connected to the same SPI bus
import device1_config
import device2_config

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

### ADS1256 two-devices configuration EXAMPLE ###
CH0 = POS_AIN0 | NEG_AINCOM
CH1 = POS_AIN1 | NEG_AINCOM
CH2 = POS_AIN2 | NEG_AINCOM
CH3 = POS_AIN3 | NEG_AINCOM
CH4 = POS_AIN4 | NEG_AINCOM
CH5 = POS_AIN5 | NEG_AINCOM
CH6 = POS_AIN6 | NEG_AINCOM
CH7 = POS_AIN7 | NEG_AINCOM

CH_SEQUENCE = CH0, CH1, CH2, CH3, CH4, CH5, CH6, CH7


def loop_forever_measurements(ads1, ads2):
    # Channel gain must be multiplied by LSB weight in volts per digit to
    # display each channels input voltage. The result is a np.array again here:
    buffer1 = np.zeros(len(CH_SEQUENCE), dtype=np.int)
    buffer2 = np.zeros(len(CH_SEQUENCE), dtype=np.int)
    # Limit output data rate to fixed time interval
    timestamp = time.time()
    # Endless loop reading into buffer and displaying results
    while True:
        # The result channel values are directy read into the array specified
        # as the second argument, which must be a mutable type.
        ads1.read_sequence(CH_SEQUENCE, buffer1)
        ads2.read_sequence(CH_SEQUENCE, buffer2)
        elapsed = time.time() - timestamp
        if elapsed > 1:
            timestamp += 1
            # Calculate output voltages for the eight channels
            ads1_volts = buffer1 * ads1.v_per_digit
            ads2_volts = buffer2 * ads2.v_per_digit
            # Text-mode output
            screen.put("\nResults for ADC 1:")
            screen.put(text_format_8_ch(buffer1, ads1_volts))
            screen.put("\nResults for ADC 2:")
            screen.put(text_format_8_ch(buffer2, ads2_volts))
            screen.refresh()


# Startup and stop handling, freeing pigpio and ADS1256 resources at exit
try:
    # PIGPIO instance, could be a re-used instance from elsewhere.
    # ADS1256 class will create an instance if not given an exisging one.
    pi = pigpio.pi()
    ### Initialise ADC objects for two chips connected to the SPI bus.
    ads1 = ADS1256(device1_config, pi)
    ads2 = ADS1256(device2_config, pi)
    # The ADC instances can also be configured at run-time
    ads1.drate = DRATE_100
    ads2.drate = DRATE_100
    ### Gain and offset self-calibration:
    ads1.cal_self()
    ads2.cal_self()
    # Main loop
    loop_forever_measurements(ads1, ads2)

except KeyboardInterrupt:
    ads1.stop()
    ads2.stop()
    print("\nUser exit.\n")

finally:
    pi.stop()
