#!/usr/bin/env python3
"""Benchmark for class ADS1256 in pipyadc package

Hardware: IsoFlux ADS1256 board interfaced to the Raspberry Pi 3
 
Ulrich Lukas 2022-06-28
"""
import time
import logging
from pipyadc import ADS1256
from pipyadc.ADS1256_definitions import *

# ADC settings for benchmark
import bench_config

logging.basicConfig(level=logging.DEBUG)

print("\x1B[2J\x1B[H") # Clear screen
print(__doc__)
print("\nPress CTRL-C to exit.\n")

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

POTI = POS_AIN0|NEG_AINCOM
LDR  = POS_AIN1|NEG_AINCOM
CH_SEQUENCE = (POTI, LDR)


def do_measurement(ads):
    n_loop = 1000
    timestamp1 = time.time()
    for _ in range(1, n_loop):
        raw_channels = ads.read_continue(CH_SEQUENCE)
#        raw_channels = ads.read_sequence(CH_SEQUENCE)
    timestamp2 = time.time()

    voltages     = [i * ads.v_per_digit for i in raw_channels]
    print(text_format_8_ch(raw_channels, voltages))
    # Timing info
    delta = timestamp2 - timestamp1
    print("Delta seconds: {}".format(delta))
    per_sample = 1.0E6*delta/(n_loop*len(CH_SEQUENCE))
    print("Per sample microseconds: {}".format(per_sample))
    overhead = per_sample - 1.0e6/n_loop
    print("Overhead per sample microseconds: {}".format(overhead))


# Start data acquisition
try:
    with ADS1256(bench_config) as ads:
        ads.cal_self()
        do_measurement(ads)

except (KeyboardInterrupt):
    print("\nUser exit.\n")
