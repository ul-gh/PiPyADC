# -*- coding: utf-8 -*-
from ADS1256_definitions import *
################  Raspberry Pi Physical Interface Properties  #################
# SPI bus configuration and GPIO pins used for the ADS1255/ADS1256.
# These defaults are used by the constructor of the ADS1256 class.
#
# To create multiple class instances for more than one AD converter, a unique
# configuration must be specified as argument for each instance.
#
# The following pins are compatible
# with the Waveshare High Precision AD/DA board on the Raspberry Pi 2B and 3B
#
# SPI_CHANNEL corresponds to the chip select hardware bin controlled by the
# SPI hardware. For the Waveshare board this pin is not even connected, so this
# code does not use hardware-controlled CS and this is a don't care value.
# FIXME: Implement hardware chip select as an option.
SPI_CHANNEL   = 1
# SPI_MODE specifies clock polarity and phase; MODE=1 <=> CPOL=0, CPHA=1
SPI_MODE      = 1
# SPI clock rate in Hz. The ADS1256 supports a minimum of 1/10th of the output
# sample data rate in Hz to 1/4th of the oscillator CLKIN_FREQUENCY which
# results in a value of 1920000 Hz for the Waveshare board. However, since
# the Raspberry pi only supports power-of-two fractions of the 250MHz system
# clock, the closest value would be 1953125 Hz, which is slightly out of spec
# for the ADS1256. Choosing 250MHz/256 = 976563 Hz is a safe choice.
SPI_FREQUENCY = 976563
# Risking the slightly out-of-spec speed:
#SPI_FREQUENCY = 1953125

# The RPI GPIOs used: All of these are optional and must be set to None if not
# used. In This case, the inputs must be hardwired to the correct logic level
# and a sufficient DRDY_TIMEOUT must be specified further below.
# Obviously, when not using hardware polling of the DRDY signal, acquisition
# will be much slower with long delays. See datasheet..
#CS_PIN      = None
CS_PIN      = 15 
DRDY_PIN    = 11
RESET_PIN   = 12
PDWN_PIN    = 13
###############################################################################

##################  ADS1256 Constant Configuration Settings  ###################
# Seconds to wait in case DRDY pin is not connected or the chip
# does not respond. See table 21 of ADS1256 datasheet: When using a
# sample rate of 2.5 SPS and issuing a self calibration command,
# the timeout can be up to 1228 milliseconds:
DRDY_TIMEOUT    = 2
# Optional delay in seconds to avoid busy wait and reduce CPU load when
# polling the DRDY pin. Default is 0.000001 or 1 µs (timing not accurate)
DRDY_DELAY      = 0.000001
# Master clock rate in Hz. Default is 7680000:
CLKIN_FREQUENCY = 7680000
################################################################################


# All following settings are accessible through ADS1256 class properties

##############  ADS1256 Default Runtime Adjustable Properties  #################
# Analog reference voltage between VREFH and VREFN pins
v_ref = 2.5
# Gain seting of the integrated programmable amplifier. This value must be
# one of (GAIN_1, GAIN_2, GAIN_4, GAIN_8, GAIN_16, GAIN_32, GAIN_64).
# Gain = 1, V_ref = 2.5V ==> full-scale input voltage = 5.00V, corresponding
# to a 24-bit two's complement output value of 2**23 - 1 = 8388607
gain_flags = GAIN_1
################################################################################

####################  ADS1256 Default Register Settings  #######################
# REG_STATUS:
# When enabling the AUTOCAL flag: Any following operation that changes
# PGA GAIN, DRATE or BUFFER flags triggers a self calibration:
# THIS REQUIRES an additional timeout via WaitDRDY() after each such operation.
# Note: BUFFER_ENABLE means the ADC input voltage range is limited
# to (AVDD-2V),see datasheet
status = BUFFER_ENABLE
# REG_MUX:
# Default: positive input = AIN0, negative input = AINCOM
mux = POS_AIN0 | NEG_AINCOM
# REG_ADCON:
# Disable clk out signal (not needed, source of disturbance),
# sensor detect current sources disabled, gain setting as defined above:
adcon = CLKOUT_OFF | SDCS_OFF | gain_flags
# REG_DRATE: 
# 10 SPS places a filter zero at 50 Hz and 60 Hz for line noise rejection
drate  = DRATE_10
# REG_IO: No GPIOs needed
gpio = 0x00
################################################################################


"""###########  Extended Description: ADS1256 Registers  ###############
Logically OR all desired option values together to form a 1 byte command
and write it to the respective register

Available Registers with address definitions:
        REG_STATUS  = 0x00
        REG_MUX     = 0x01
        REG_ADCON   = 0x02
        REG_DRATE   = 0x03
        REG_IO      = 0x04
        REG_OFC0    = 0x05
        REG_OFC1    = 0x06
        REG_OFC2    = 0x07
        REG_FSC0    = 0x08
        REG_FSC1    = 0x09
        REG_FSC2    = 0x0A


==> REG_STATUS, Status Register Configuration:

    Bits 7-4: ID3, ID2, ID1, ID0 Factory Programmed Identification Bits 
    (Read Only)

    Bit 3 ORDER: Data Output Bit Order

    0 = Most Significant Bit First (default)
    1 = Least Significant Bit First

    Input data is always shifted in most significant byte and bit first.
    Output data is always shifted out most significant byte first. The
    ORDER bit only controls the bit order of the output data within the
    byte.

    Bit 2 ACAL: Auto-Calibration

    0 = Auto-Calibration Disabled (default)
    1 = Auto-Calibration Enabled

    When Auto-Calibration is enabled, self-calibration begins at the
    completion of the WREG command that changes the PGA (bits 0-2 of ADCON
    register), DR (bits 7-0 in the DRATE register) or BUFEN (bit 1 in the
    STATUS register) values.

    THIS REQUIRES that an additional timeout (DREADY polling)
    is implemented after each such operation!


    Bit 1 BUFEN: Analog Input Buffer Enable

    0 = Buffer Disabled (default)
    1 = Buffer Enabled

    Bit 0 DRDY: Data Ready (Read Only)

    This bit duplicates the state of the DRDY pin, which is inverted logic.

    Option value definitions:
        BUFFER_ENABLE    = 0x02
        AUTOCAL_ENABLE   = 0x04
        ORDER_LSB        = 0x08


==> REG_MUX, Input multiplexer register, channel selection:

    High nibble selects positive input, low nibble negative input.

    Pin selection codes: logic OR together to form the register value.
    Example: ads1256.mux = POS_AIN0 | NEG_AINCOM

    # Pin selection codes for the positive input:
    POS_AIN0   = 0x00
    POS_AIN1   = 0x10
    POS_AIN2   = 0x20
    POS_AIN3   = 0x30
    POS_AIN4   = 0x40
    POS_AIN5   = 0x50
    POS_AIN6   = 0x60
    POS_AIN7   = 0x70
    POS_AINCOM = 0x80
    # Pin selection codes for the negative input:
    NEG_AIN0   = 0x00
    NEG_AIN1   = 0x01
    NEG_AIN2   = 0x02
    NEG_AIN3   = 0x03
    NEG_AIN4   = 0x04
    NEG_AIN5   = 0x05
    NEG_AIN6   = 0x06
    NEG_AIN7   = 0x07
    NEG_AINCOM = 0x08


==> REG_ADCON, A/D Control Register

    Bit 7 Reserved, always 0 (Read Only)

    Bits 6-5 CLK1, CLK0: D0/CLKOUT Clock Out Rate Setting

    00 = Clock Out OFF
    01 = Clock Out Frequency = fCLKIN (default)
    10 = Clock Out Frequency = fCLKIN/2
    11 = Clock Out Frequency = fCLKIN/4

    When not using CLKOUT, it is recommended that it be turned off. These
    bits can only be reset using the RESET pin.

    Bits 4-3 SDCS1, SCDS0: Sensor Detect Current Sources

    00 = Sensor Detect OFF (default)
    01 = Sensor Detect Current = 0.5uA
    10 = Sensor Detect Current = 2uA
    11 = Sensor Detect Current = 10uA

    The Sensor Detect Current Sources can be activated to verify the
    integrity of an external sensor supplying a signal to the ADS1255/6.
    A shorted sensor produces a very small signal while an open-circuit
    sensor produces a very large signal.

    Bits 2-0 PGA2, PGA1, PGA0: Programmable Gain Amplifier Setting

    Option value definitions:
        # Clock output pin setting
        CLKOUT_OFF      = 0x00
        CLKOUT_EQUAL    = 0x20
        CLKOUT_HALF     = 0x40
        CLKOUT_FOURTH   = 0x60

        # Sensor Detect Current Source
        SDCS_OFF     = 0x00
        SDCS_500pA   = 0x08
        SDCS_2uA     = 0x10
        SDCS_10uA    = 0x18

        # Gain level flag bits definitions:
        GAIN_1      = 0x00
        GAIN_2      = 0x01
        GAIN_4      = 0x02
        GAIN_8      = 0x03
        GAIN_16     = 0x04
        GAIN_32     = 0x05
        GAIN_64     = 0x06


==> DRATE Register: A/D Data Rate Address 0x03

    Bits 7-0 DR[7: 0]: Data Rate Setting(1)
    The 16 valid Data Rate settings. Only these are valid. 

    Sample rate definitions for CLKIN_FREQUENCY = 7.68 MHz (fCLKIN).
    Actual sample rates scale linearly with differing clock rates.

        DRATE_30000     = 0b11110000 # 30,000SPS (default)
        DRATE_15000     = 0b11100000 # 15,000SPS
        DRATE_7500      = 0b11010000 # 7,500SPS
        DRATE_3750      = 0b11000000 # 3,750SPS
        DRATE_2000      = 0b10110000 # 2,000SPS
        DRATE_1000      = 0b10100001 # 1,000SPS
        DRATE_500       = 0b10010010 # 500SPS
        DRATE_100       = 0b10000010 # 100SPS
        DRATE_60        = 0b01110010 # 60SPS
        DRATE_50        = 0b01100011 # 50SPS
        DRATE_30        = 0b01010011 # 30SPS
        DRATE_25        = 0b01000011 # 25SPS
        DRATE_15        = 0b00110011 # 15SPS
        DRATE_10        = 0b00100011 # 10SPS
        DRATE_5         = 0b00010011 # 5SPS
        DRATE_2_5       = 0b00000011 # 2.5SPS
"""


