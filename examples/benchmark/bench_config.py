import logging
from typing import Literal

from pipyadc.ADS1256_definitions import *

################ Configuration file for one ADS1256 instance  #################
# SPI bus configuration and GPIO pins used for the ADS1255/ADS1256.
#
# These settings are compatible with the "Waveshare High-Precision AD/DA" board.
#
# To create multiple class instances for more than one AD converter, a unique
# configuration must be specified as argument for each instance.

LOGLEVEL = logging.WARNING

# 0 for main SPI bus, 1 for auxiliary SPI bus.
SPI_BUS: int = 0

# Only releveant when using hardware chip select. (FIXME: Not implemented)
# This determines which chip select pin is activated for this chip.
# Main SPI has CS 0 and 1. Aux SPI has 0, 1, 2
SPI_CHANNEL: Literal[0, 1, 2] = 0

# SPI clock rate in Hz. The ADS1256 supports a minimum of 1/10th of the output
# sample data rate in Hz to 1/4th of the oscillator CLKIN_FREQUENCY which
# results in a value of 1920000 Hz for the Waveshare board. However, since
# the Raspberry pi only supports power-of-two fractions of the 250MHz system
# clock, the closest value would be 1953125 Hz, which is slightly out of spec
# for the ADS1256. Choosing 250MHz/256 = 976563 Hz is a safe choice.
SPI_FREQUENCY: Literal[976563, 1953125] = 976563

# If set to True this will perform a chip reset using the hardware reset line
# when initializing the device.
CHIP_HARD_RESET_ON_START: bool = False

# This is expected to be "3". Do not change unless hardware is changed.
CHIP_ID: int = 3

#### Raspberry Pi GPIO configuration ##########################################
# =====> NEW in version 2 since using pigpio instead of wiringpi library:
# =====> Raspberry Pi pinning now uses the Broadcom numbering scheme!

# Tuple of all (chip select) GPIO numbers to be configured as an output and
# initialised to (inactive) logic high state before bus communication starts.
# Necessary for more than one SPI device if GPIOs are not otherwise handled.
CHIP_SELECT_GPIOS_INITIALIZE: tuple[int, ...] = (22, 23)
# Chip select GPIO pin number.
# This is required as hardware chip select can not be used with the ADS125x
# devices using this library
CS_PIN: int = 22
# If DRDY is not connected to an input, a sufficient DRDY_TIMEOUT must be
# specified further below and aquisition will be slower.
DRDY_PIN: int = 17
# Hardware reset pin is optional but strongly suggested in case multiple devices
# are connected to the bus as the ADS125x will lock-up in case multiple chips
# are selected simultaneously by accident.
RESET_PIN: int = 18  # Set to None if not used.
# Optional power down pin
PDWN_PIN: int = 27  # Set to None if not used.
###############################################################################

##################  ADS1256 Constant Configuration Settings  ##################
# Seconds to wait in case DRDY pin is not connected or the chip
# does not respond. See table 21 of ADS1256 datasheet: When using a
# sample rate of 2.5 SPS and issuing a self calibration command,
# the timeout can be up to 1228 milliseconds:
DRDY_TIMEOUT: float = 2.0
# Optional delay in seconds to avoid busy wait and reduce CPU load when
# polling the DRDY pin. Default is 0.000001 or 1 µs (timing not accurate)
DRDY_DELAY: float = 0.000001
# Master clock rate in Hz. Default is 7680000:
CLKIN_FREQUENCY: int = 7680000
################################################################################


# All following settings are accessible through ADS1256 class properties

##############  ADS1256 Default Runtime Adjustable Properties  #################
# Analog reference voltage between VREFH and VREFN pins
v_ref: float = 2.5
# Gain seting of the integrated programmable amplifier. This value must be
# one of (GAIN_1, GAIN_2, GAIN_4, GAIN_8, GAIN_16, GAIN_32, GAIN_64).
# Gain = 1, V_ref = 2.5V ==> full-scale input voltage = 5.00V, corresponding
# to a 24-bit two's complement output value of 2**23 - 1 = 8388607
gain_flags: int = GAIN_1
################################################################################

####################  ADS1256 Default Register Settings  #######################
# REG_STATUS:
# When enabling the AUTOCAL flag: Any following operation that changes
# PGA GAIN, DRATE or BUFFER flags triggers a self calibration:
# THIS REQUIRES an additional timeout via WaitDRDY() after each such operation.
# Note: BUFFER_ENABLE means the ADC input voltage range is limited
# to (AVDD-2V),see datasheet
status: int = BUFFER_ENABLE
# REG_MUX:
# Default: positive input = AIN0, negative input = AINCOM
mux: int = POS_AIN0 | NEG_AINCOM
# REG_ADCON:
# Disable clk out signal (if not needed, source of disturbance),
# sensor detect current sources disabled, gain setting as defined above:
adcon: int = CLKOUT_OFF | SDCS_OFF | gain_flags
# REG_DRATE:
# 10 SPS places a filter zero at 50 Hz and 60 Hz for line noise rejection
drate: int = DRATE_10
# REG_IO: No GPIOs needed
gpio: int = 0x00
################################################################################
