################### CONSTANT DEFINITIONS for class ADS1256 ####################
# Register addresses:
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
NUM_REG     = 11

# REG_STATUS: Flags:
BUFFER_ENABLE    = 0x02
AUTOCAL_ENABLE   = 0x04
ORDER_LSB        = 0x08

# REG_MUX: High nibble selects positive input, low nibble negative input.
# Pin selection codes: logic OR together to form the register value.
# Example: ads1256.mux = POS_AIN0 | NEG_AINCOM
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

# REG_ADCON: Gain levels
# Note: You can set the numeric values 1, 2, 4, 8, 16, 32 and 64 directly
# via class/instance property ADS1256.pga_gain.
GAIN_1      = 0x00
GAIN_2      = 0x01
GAIN_4      = 0x02
GAIN_8      = 0x03
GAIN_16     = 0x04
GAIN_32     = 0x05
GAIN_64     = 0x06

# REG_ADCON: Sensor Detect Current Sources
SDCS_OFF     = 0x00
SDCS_500pA   = 0x08
SDCS_2uA     = 0x10
SDCS_10uA    = 0x18

# REG_ADCON: Clock output pin settings
CLKOUT_OFF      = 0x00
CLKOUT_EQUAL    = 0x20
CLKOUT_HALF     = 0x40
CLKOUT_FOURTH   = 0x60

# REG_DRATE: Sample rate definitions:
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


# Chip-level-command Definitions. Special functions implemented in this
# class take care of command order / timeouts and handling of data/results.
CMD_WAKEUP  = 0x00 # Completes SYNC and exits standby mode
CMD_RDATA   = 0x01 # Read data
CMD_RDATAC  = 0x03 # Start read data continuously
CMD_SDATAC  = 0x0F # Stop read data continuously
CMD_RREG    = 0x10 # Read from register
CMD_WREG    = 0x50 # Write to register
CMD_SELFCAL = 0xF0 # Offset and gain self-calibration
CMD_SELFOCAL= 0xF1 # Offset self-calibration
CMD_SELFGCAL= 0xF2 # Gain self-calibration
CMD_SYSOCAL = 0xF3 # System offset calibration
CMD_SYSGCAL = 0xF4 # System gain calibration
CMD_SYNC    = 0xFC # Synchronize the A/D conversion
CMD_STANDBY = 0xFD # Begin standby mode
CMD_RESET   = 0xFE # Reset to power-on values
