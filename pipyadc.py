# -*- coding: utf-8 -*-
"""PiPyADC - Python module for interfacing Texas Instruments SPI
bus based analog-to-digital converters with the Raspberry Pi.

Currently only implemented class in this module is ADS1256 for the
ADS1255 and ADS1256 chips which are register- and command compatible.

Download: https://github.com/ul-gh/PiPyADC

Depends on WiringPi library, see:
https://github.com/WiringPi/WiringPi-Python

Uses code from: https://github.com/heathsd/PyADS1256

License: GNU LGPLv2.1, see:
https://www.gnu.org/licenses/old-licenses/lgpl-2.1-standalone.html

Ulrich Lukas, 2017-03-03
"""
import time
import struct
import wiringpi as wp
from ADS1256_definitions import *
import ADS1256_default_config

class ADS1256(object):
    """Python class for interfacing the ADS1256 and ADS1255 analog to
    digital converters with the Raspberry Pi.

    This is part of module PiPyADC
    Download: https://github.com/ul-gh/PiPyADC
    
    Default pin and settings configuration is for the Open Hardware
    "Waveshare High-Precision AD/DA Board"

    See file ADS1256_default_config.py for
    configuration settings and description.

    Register read/write access is implemented via Python class/instance
    properties. Commands are implemented as functions.

    See help(ADS1256) for usage of the properties and functions herein.

    See ADS1256_definitions.py for chip registers, flags and commands.
    
    Documentation source: Texas Instruments ADS1255/ADS1256
    datasheet SBAS288: http://www.ti.com/lit/ds/sbas288j/sbas288j.pdf
    """
    @property
    def v_ref(self):
        """Get/Set ADC analog reference input voltage differential.
        This is only for calculation of output value scale factor.
        """
        return self._v_ref
    @v_ref.setter
    def v_ref(self, value):
        self._v_ref = value

    @property
    def pga_gain(self):
        """Get/Set ADC programmable gain amplifier setting.
        
        The available options for the ADS1256 are:
        1, 2, 4, 8, 16, 32 and 64.

        This function sets the ADCON register with the code values
        defined in file ADS1256_definitions.py.

        Note: When changing the gain setting at runtime, with activated
        ACAL flag (AUTOCAL_ENABLE), this causes a Wait_DRDY() timeout
        for the calibration process to finish.
        """
        return 2**(self.read_reg(REG_ADCON)&0b111)
    @pga_gain.setter
    def pga_gain(self, value):
        if value not in (1, 2, 4, 8, 16, 32, 64):
            raise ValueError("Argument must be one of: 1, 2, 4, 8, 16, 32, 64")
        else:
            log2val = int.bit_length(value) - 1
            self.write_reg(REG_ADCON, self.adcon&0b11111000 | log2val)
            if self._status & AUTOCAL_ENABLE:
                self.wait_DRDY()

    @property
    def v_per_digit(self):
        """Get ADC LSB weight in volts per numeric output digit.
        Readonly: This is a convenience value calculated from
        gain and v_ref setting.
        """
        return self.v_ref * 2.0/(self.pga_gain * (2**23 - 1))
    @v_per_digit.setter
    def v_per_digit(self, value):
        raise AttributeError("This is a read-only attribute")

    @property
    def status(self):
        """Get/Set value of ADC status register, REG_STATUS (8 bit).
        For available settings flag options, see datasheet and file
        ADS1256_definitions.py. Note: When enabling the AUTOCAL
        flag, any subsequent access to the BUFEN flag, DRATE register
        (drate property) or PGA gain setting (gain property) will cause
        an additional delay for completion of hardware auto-calibration.
        """
        return self.read_reg(REG_STATUS)
    @status.setter
    def status(self, value):
        self.write_reg(REG_STATUS, value)
        self._status = value
        # When AUTOCAL flag has been enabled, a wait_DRDY() is needed here.
        # When AUTOCAL flag had been enabled before and the BUFEN flag has not
        # been changed, this is likely not necessary, but FIXME: Better risk
        # a small possibly unnecessary delay than risk invalid data or settings.
        # Thus, if AUTOCAL flag /is/ enabled, always do a wait_DRDY().
        if self._status & AUTOCAL_ENABLE:
            self.wait_DRDY()

    @property
    def mux(self):
        """Get/Set value of ADC analog input multiplexer register,
        REG_MUX, used for selecting any arbitrary pair of input pins
        as a differential input channel. For single-ended measurements,
        choose NEG_AINCOM as the second input pin.

        The most significant four bits select the positive input pin.
        The least significant four bits select the negative input pin.
        
        Example: ads1256.mux = POS_AIN4 | NEG_AINCOM.

        IMPORTANT:

        When switching inputs during a running conversion cycle,
        invalid data is acquired.

        To prevent this, you must restart the conversion using the
        sync() function or the SYNC hardware pin before doing an
        async_read().

        The resulting delay can be avoided. See functions:

        read_and_next_is(diff_channel)
            for cyclic single-channel reads and:
        
        read_sequence()
            for cyclic reads of multiple channels at once.
        """
        return self.read_reg(REG_MUX)
    @mux.setter
    def mux(self, value):
        self.write_reg(REG_MUX, value)

    @property
    def adcon(self):
        """Get/Set value of the ADC configuration register, REG_ADCON.
        Note: When the AUTOCAL flag is enabled, this causes a
        wait_DRDY() timeout.
        """
        return self.read_reg(REG_ADCON) 
    @adcon.setter
    def adcon(self, value):
        self.write_reg(REG_ADCON, value)
        if self._status & AUTOCAL_ENABLE:
            self.wait_DRDY()

    @property
    def drate(self):
        """Get/Set value of the ADC output sample data rate by setting
        the DRATE register (REG_DRATE).
        This configures the hardware integrated moving average filter.

        When changing the register during a running acquisition,
        invalid data is sampled. In this case, call the sync() method
        to restart the acquisition cycle.

        The available data rates are defined in ADS1256_definitions.py.
        """
        return self.read_reg(REG_DRATE)
    @drate.setter
    def drate(self, value):
        self.write_reg(REG_DRATE, value)

    @property
    def gpio(self):
        """Get the logic level of the four GPIO pins, returned as
        a four-bit bitmask or Set the status of the GPIO register,
        REG_IO, where the most significant four bits represent the
        pin direction, and the least significant four bits determine
        the output logic level.
        A timeout/debounce for the reading is not implemented.
        """
        return 0x0F & self.read_reg(REG_IO)
    @gpio.setter
    def gpio(self, value):
        self.write_reg(REG_IO, value)

    @property
    def ofc(self):
        """Get/Set the three offset compensation registers, OFC0..2.
        This property is supposed to be a signed integer value.
        Gets/sets 24-bit two's complement value in three 8-bit-registers.
        """
        ofc0 = self.read_reg(REG_OFC0)
        ofc1 = self.read_reg(REG_OFC1)
        ofc2 = self.read_reg(REG_OFC2)
        int24_result = ofc2<<16 | ofc1<<8 | ofc0
        # Take care of 24-Bit 2's complement.
        if int24_result < 0x800000:
            return int24_result
        else:
            return int24_result - 0x1000000
    @ofc.setter
    def ofc(self, value):
        value = int(value)
        if value < -0x800000 or value > 0x7FFFFF:
            raise ValueError("Error: Offset value out of signed int24 range")
        else:
            # Generate 24-Bit 2's complement.
            if value < 0:
                value += 0x1000000
            # self.write_reg() automatically truncates to uint8
            self.write_reg(REG_OFC0, value)
            value >>= 8
            self.write_reg(REG_OFC1, value)
            value >>= 8
            self.write_reg(REG_OFC2, value)
 
    @property
    def fsc(self):
        """Get/Set the three full-scale adjustment registers, OFC0..2.
        This property is supposed to be a positive integer value.
        Gets/sets 24-bit unsigned int value in three 8-bit-registers.
        """
        fsc0 = self.read_reg(REG_FSC0)
        fsc1 = self.read_reg(REG_FSC1)
        fsc2 = self.read_reg(REG_FSC2)
        return fsc2<<16 | fsc1<<8 | fsc0
    @fsc.setter
    def fsc(self, value):
        value = int(value)
        if value < 0 or value > 0xFFFFFF:
            raise ValueError("Error: This must be a positive int of 24-bit range")
        else:
            # self.write_reg() automatically truncates to uint8
            self.write_reg(REG_FSC0, value)
            value >>= 8
            self.write_reg(REG_FSC1, value)
            value >>= 8
            self.write_reg(REG_FSC2, value)

    @property
    def chip_ID(self):
        """Get the numeric ID from the ADS chip.
        Useful to check if hardware is connected.

        Value for the ADS1256 on the Waveshare board seems to be a 3.
        """
        self.wait_DRDY()
        return self.read_reg(REG_STATUS) >> 4
    @chip_ID.setter
    def chip_ID(self, value):
        raise AttributeError("This is a read-only attribute")


    # Constructor for the ADC object: Hardware pin configuration must be
    # set up at initialization phase and can not be changed later.
    # Register/Configuration Flag settings are initialized, but these
    # can be changed during runtime via class properties.
    # Default config is read from external file (module) import
    def __init__(self, conf=ADS1256_default_config):
        # Set up the wiringpi object to use physical pin numbers
        wp.wiringPiSetupPhys()
        # Config and initialize the SPI and GPIO pins used by the ADC.
        # The following four entries are actively used by the code:
        self.SPI_CHANNEL  = conf.SPI_CHANNEL
        self.DRDY_PIN     = conf.DRDY_PIN
        self.CS_PIN       = conf.CS_PIN
        self.DRDY_TIMEOUT = conf.DRDY_TIMEOUT
        self.DRDY_DELAY   = conf.DRDY_DELAY

        # Only one GPIO input:
        if conf.DRDY_PIN is not None:
            self.DRDY_PIN = conf.DRDY_PIN
            wp.pinMode(conf.DRDY_PIN,  wp.INPUT)

        # GPIO Outputs. Only the CS_PIN is currently actively used. ~RESET and 
        # ~PDWN must be set to static logic HIGH level if not hardwired:
        for pin in (conf.CS_PIN,
                    conf.RESET_PIN,
                    conf.PDWN_PIN):
            if pin is not None:
                wp.pinMode(pin, wp.OUTPUT)
                wp.digitalWrite(pin, wp.HIGH)
        
        # Initialize the wiringpi SPI setup. Return value is the Linux file
        # descriptor for the SPI bus device:
        fd = wp.wiringPiSPISetupMode(
                conf.SPI_CHANNEL, conf.SPI_FREQUENCY, conf.SPI_MODE)
        if fd == -1:
            raise IOError("ERROR: Could not access SPI device file")
        
        # ADS1255/ADS1256 command timing specifications. Do not change.
        # Delay between requesting data and reading the bus for
        # RDATA, RDATAC and RREG commands (datasheet: t_6 >= 50*CLKIN period).
        self._DATA_TIMEOUT_US = int(1 + (50*1000000)/conf.CLKIN_FREQUENCY)
        # Command-to-command timeout after SYNC and RDATAC
        # commands (datasheet: t11)
        self._SYNC_TIMEOUT_US = int(1 + (24*1000000)/conf.CLKIN_FREQUENCY)
        # See datasheet ADS1256: CS needs to remain low
        # for t_10 = 8*T_CLKIN after last SCLK falling edge of a command.
        # Because this delay is longer than timeout t_11 for the
        # RREG, WREG and RDATA commands of 4*T_CLKIN, we do not need
        # the extra t_11 timeout for these commands when using software
        # chip select selection and the _CS_TIMEOUT_US.
        self._CS_TIMEOUT_US   = int(1 + (8*1000000)/conf.CLKIN_FREQUENCY)
        # When using hardware/hard-wired chip select, still a command-
        # to command timeout of t_11 is needed as a minimum for the
        # RREG, WREG and RDATA commands.
        self._T_11_TIMEOUT_US   = int(1 + (4*1000000)/conf.CLKIN_FREQUENCY)

        # Initialise class properties
        self.v_ref         = conf.v_ref

        # At hardware initialisation, a settling time for the oscillator
        # is necessary before doing any register access.
        # This is approx. 30ms, according to the datasheet.
        time.sleep(0.03)
        self.wait_DRDY()
        # Device reset for defined initial state
        self.reset()

        # Configure ADC registers:
        # Status register not yet set, only variable written to avoid multiple
        # triggering of the AUTOCAL procedure by changing other register flags
        self._status       = conf.status
        # Class properties now configure registers via their setter functions
        self.mux           = conf.mux
        self.adcon         = conf.adcon
        self.drate         = conf.drate
        self.gpio          = conf.gpio
        self.status        = conf.status


    def _chip_select(self):
        # If chip select hardware pin is connected to SPI bus hardware pin or
        # hardwired to GND, do nothing.
        if self.CS_PIN is not None:
            wp.digitalWrite(self.CS_PIN, wp.LOW)

    # Release chip select and implement t_11 timeout
    def _chip_release(self):
        if self.CS_PIN is not None:
            wp.delayMicroseconds(self._CS_TIMEOUT_US)
            wp.digitalWrite(self.CS_PIN, wp.HIGH)
        else:
            # The minimum t_11 timeout between commands, see datasheet Figure 1.
            wp.delayMicroseconds(self._T_11_TIMEOUT_US)

    def _send_uint8(self, *vals):
        # Reads integers in range (0, 255), sends as uint-8 via the SPI bus
        wp.wiringPiSPIDataRW(self.SPI_CHANNEL,
                             struct.pack("{}B".format(len(vals)), *vals))
        # Python3 only:
        # wp.wiringPiSPIDataRW(self.SPI_CHANNEL, bytes(vals))

    def _read_uint8(self, n_vals=1):
        # Returns tuple containing unsigned 8-bit int interpretation of
        # n_vals bytes read via the SPI bus. n_bytes is supposed to 
        n_bytes, data = wp.wiringPiSPIDataRW(self.SPI_CHANNEL, b"\xFF"*n_vals)
        assert n_bytes == n_vals
        return struct.unpack("{}B".format(n_bytes), data)
        # Python3 only:
        # return tuple(data)

    def _read_int24(self):
        # Returns signed int interpretation of three bytes read via the SPI bus
        _, data = wp.wiringPiSPIDataRW(self.SPI_CHANNEL, b"\xFF\xFF\xFF")
        return struct.unpack(">i", data + b"\x00")[0] >> 8
        # Python3 only:
        # return int.from_bytes(data, "big", signed=True)

    def _send_int24(self, val):
        wp.wiringPiSPIDataRW(self.SPI_CHANNEL, struct.pack(">i", val))[1:4]
        # Python3 only:
        # wp.wiringPiSPIDataRW(self.SPI_CHANNEL, int.to_bytes(val, 3, "big"))



    def wait_DRDY(self):
        """Delays until the configured DRDY input pin is pulled to
        active logic low level by the ADS1256 hardware or until the
        DRDY_TIMEOUT in seconds has passed.

        Arguments: None
        Returns: None

        The minimum necessary DRDY_TIMEOUT when not using the hardware
        pin, can be up to approx. one and a half second, see datasheet..
        
        Manually invoking this function is necessary when using the
        automatic calibration feature (ACAL flag). Then, use wait_DRDY()
        after every access that changes the PGA gain bits in
        ADCON register, the DRATE register or the BUFFEN flag.
        """
        start = time.time()
        elapsed = time.time() - start
        # Waits for DRDY pin to go to active low or DRDY_TIMEOUT seconds to pass
        if self.DRDY_PIN is not None:
            drdy_level = wp.digitalRead(self.DRDY_PIN)
            while (drdy_level == wp.HIGH) and (elapsed < self.DRDY_TIMEOUT):
                elapsed = time.time() - start
                drdy_level = wp.digitalRead(self.DRDY_PIN)
                # Delay in order to avoid busy wait and reduce CPU load.
                time.sleep(self.DRDY_DELAY)
            if elapsed >= self.DRDY_TIMEOUT:
                print("\nWarning: Timeout while polling configured DRDY pin!\n")
        else:
            time.sleep(self.DRDY_TIMEOUT)




    def read_reg(self, register):
        """Returns data byte from the specified register
        
        Argument: register address
        """
        self._chip_select()
        self._send_uint8(CMD_RREG | register, 0x00)
        wp.delayMicroseconds(self._DATA_TIMEOUT_US)
        read, = self._read_uint8()
        # Release chip select and implement t_11 timeout
        self._chip_release()
        return read


    def write_reg(self, register, data):
        """Writes data byte to the specified register
 
        Arguments: register address, data byte (uint_8)
        """
        self._chip_select()
        # Tell the ADS chip which register to start writing at,
        # how many additional registers to write (0x00) and send data
        self._send_uint8(CMD_WREG | register, 0x00, data&0xFF)
        # Release chip select and implement t_11 timeout
        self._chip_release()


    def cal_self_offset(self):
        """Perform an input zero calibration using chip-internal
        reference switches.

        Sets the ADS1255/ADS1256 OFC register.
        """
        self._chip_select()
        self._send_uint8(CMD_SELFOCAL)
        self.wait_DRDY()
        # Release chip select and implement t_11 timeout
        self._chip_release()


    def cal_self_gain(self):
        """Perform an input full-scale calibration
        using chip-internal circuitry connected to VREFP and VREFN.

        Sets the ADS1255/ADS1256 FSC register.
        """
        self._chip_select()
        self._send_uint8(CMD_SELFGCAL)
        self.wait_DRDY()
        # Release chip select and implement t_11 timeout
        self._chip_release()


    def cal_self(self):
        """Perform an input zero and full-scale two-point-calibration
        using chip-internal circuitry connected to VREFP and VREFN.

        Sets the ADS1255/ADS1256 OFC and FSC registers.
        """
        self._chip_select()
        self._send_uint8(CMD_SELFCAL)
        self.wait_DRDY()
        # Release chip select and implement t_11 timeout
        self._chip_release()


    def cal_system_offset(self):
        """Set the ADS1255/ADS1256 OFC register such that the
        current input voltage corresponds to a zero output value.
        The input multiplexer must be set to the appropriate pins first.
        """
        self._chip_select()
        self._send_uint8(CMD_SYSOCAL)
        self.wait_DRDY()
        # Release chip select and implement t_11 timeout
        self._chip_release()


    def cal_system_gain(self):
        """Set the ADS1255/ADS1256 FSC register such that the current
        input voltage corresponds to a full-scale output value.
        The input multiplexer must be set to the appropriate pins first.
        """
        self._chip_select()
        self._send_uint8(CMD_SYSGCAL)
        self.wait_DRDY()
        # Release chip select and implement t_11 timeout
        self._chip_release()


    def standby(self):
        """Put chip in low-power standby mode
        """
        self._chip_select()
        self._send_uint8(CMD_STANDBY)
        # Release chip select and implement t_11 timeout
        self._chip_release()


    def wakeup(self):
        """Wake up the chip from standby mode.
        See datasheet for settling time specifications after wake-up.
        Data is ready when the DRDY pin becomes active low.

        You can then use the read_oneshot() function to read a new
        sample of input data.

        Call standby() to enter standby mode again.
        """
        self._chip_select()
        self._send_uint8(CMD_WAKEUP)
        # Release chip select and implement t_11 timeout
        self._chip_release()


    def reset(self):
        """Reset all registers except CLK0 and CLK1 bits
        to reset values and Polls for DRDY change / timeout afterwards.
        """
        self._chip_select()
        self._send_uint8(CMD_RESET)
        self.wait_DRDY()
        # Release chip select and implement t_11 timeout
        self._chip_release()


    def sync(self):
        """Restart the ADC conversion cycle with a SYNC + WAKEUP
        command sequence as described in the ADS1256 datasheet.
        
        This is useful to restart the acquisition cycle after rapid
        changes of the input signals, for example when using an
        external input multiplexer or after changing ADC configuration
        flags.
        """
        self._chip_select()
        self._send_uint8(CMD_SYNC)
        wp.delayMicroseconds(self._SYNC_TIMEOUT_US)
        self._send_uint8(CMD_WAKEUP)
        # Release chip select and implement t_11 timeout
        self._chip_release()


    def read_async(self):
        """Read ADC result as soon as possible
        
        Arguments:  None
        Returns:    Signed integer ADC conversion result

        Issue this command to read a single conversion result for a
        previously set /and stable/ input channel configuration.

        For the default, free-running mode of the ADC, this means
        invalid data is returned when not synchronising acquisition
        and input channel configuration changes.

        To avoid this, after changing input channel configuration or
        with an external hardware multiplexer, use the hardware SYNC
        input pin or use the sync() method to restart the
        conversion cycle before calling read_async().
        
        Because this function does not implicitly restart a running
        acquisition, it is faster that the read_oneshot() method.
        """
        self._chip_select()
        # Wait for data to be ready
        self.wait_DRDY()
        # Send the read command
        self._send_uint8(CMD_RDATA)
        # Wait through the data pause
        wp.delayMicroseconds(self._DATA_TIMEOUT_US)
        # The result is 24 bits big endian two's complement value by default
        int24_result = self._read_int24()
        # Release chip select and implement t_11 timeout
        self._chip_release()
        return int24_result


    def read_oneshot(self, diff_channel):
        """Restart/re-sync ADC and read the specified input pin pair.
        
        Arguments:  8-bit code value for differential input channel
                        (See definitions for the REG_MUX register)
        Returns:    Signed integer conversion result

        Use this function after waking up from STANDBY mode.
        
        When switching inputs during a running conversion cycle,
        invalid data is acquired.

        To prevent this, this function automatically restarts the
        conversion cycle after configuring the input channels.

        The resulting delay can be avoided. See functions:

        read_and_next_is(diff_channel)
            for cyclic single-channel reads and:
        
        read_sequence()
            for cyclic reads of multiple channels at once.

        """
        self._chip_select()
        # Set input pin mux position for this cycle"
        self._send_uint8(CMD_WREG | REG_MUX, 0x00, diff_channel)
        # Restart/start the conversion cycle with set input pins
        self._send_uint8(CMD_SYNC)
        wp.delayMicroseconds(self._SYNC_TIMEOUT_US)
        self._send_uint8(CMD_WAKEUP)
        self.wait_DRDY()
        # Read data from ADC, which still returns the /previous/ conversion
        # result from before changing inputs
        self._send_uint8(CMD_RDATA)
        wp.delayMicroseconds(self._DATA_TIMEOUT_US)
        # The result is 24 bits little endian two's complement value by default
        int24_result = self._read_int24()
        # Release chip select and implement t_11 timeout
        self._chip_release()
        return int24_result


    def read_and_next_is(self, diff_channel):
        """Reads ADC data of presently running or already finished
        conversion, sets and synchronises new input channel config
        for next sequential read.

        Arguments:  8-bit code value for differential input channel
                        (See definitions for the REG_MUX register)
        Returns:    Signed integer conversion result for present read
        
        This enables rapid dycling through different channels and
        implements the timing sequence outlined in the ADS1256
        datasheet (Sept.2013) on page 21, figure 19: "Cycling the
        ADS1256 Input Multiplexer".

        Note: In most cases, a fixed sequence of input channels is known
        beforehand. For that case, this module implements the function:
        
        read_sequence(ch_sequence)
            which automates the process for cyclic data acquisition.
        """
        self._chip_select()
        self.wait_DRDY()

        # Setting mux position for next cycle"
        self._send_uint8(CMD_WREG | REG_MUX, 0x00, diff_channel)
        # Restart/start next conversion cycle with new input config
        self._send_uint8(CMD_SYNC)
        wp.delayMicroseconds(self._SYNC_TIMEOUT_US)
        self._send_uint8(CMD_WAKEUP)
        # The datasheet is a bit unclear if a t_11 timeout is needed here.
        # Assuming the extra timeout is the safe choice:
        wp.delayMicroseconds(self._T_11_TIMEOUT_US)
        # Read data from ADC, which still returns the /previous/ conversion
        # result from before changing inputs
        self._send_uint8(CMD_RDATA)
        wp.delayMicroseconds(self._DATA_TIMEOUT_US)
        # The result is 24 bits little endian two's complement value by default
        int24_result = self._read_int24()
        # Release chip select and implement t_11 timeout
        self._chip_release()
        return int24_result


    def read_continue(self, ch_sequence, ch_buffer=None):
        """Continues reading a cyclic sequence of ADC input channel pin pairs.

        The first data sample is only valid if the ADC data register contains
        valid data from a previous conversion. I.e. the last element of the
        ch_sequence must be the first channel configuration to be read during
        the next following cycle.

        For short sequences, this is faster than the read_sequence() method
        because it does not interrupt an already running and pre-configured
        conversion cycle.

        Argument1:  Tuple (list) of 8-bit code values for differential
                    input channel pins to read sequentially in a cycle.
                    (See definitions for the REG_MUX register)

                    Example:
                    ch_sequence=(POS_AIN0|NEG_AIN1, POS_AIN2|NEG_AINCOM)

        Argument2:  List (array, buffer) of signed integer conversion
                    results for the sequence of input channels.

        Returns:    List (array, buffer) of signed integer conversion
                    results for the sequence of input channels.

        This implements the timing sequence outlined in the ADS1256
        datasheet (Sept.2013) on page 21, figure 19: "Cycling the
        ADS1256 Input Multiplexer" for cyclic data acquisition.
        """
        buf_len = len(ch_sequence)
        if ch_buffer is None:
            ch_buffer = [0] * buf_len
        for i in range(0, buf_len):
            ch_buffer[i] = self.read_and_next_is(ch_sequence[(i+1)%buf_len])
        return ch_buffer


    def read_sequence(self, ch_sequence, ch_buffer=None):
        """Reads a sequence of ADC input channel pin pairs.

        Restarts and re-syncs the ADC for the first sample.

        The time delay resulting from this can be avoided when reading
        the ADC in a cyclical pattern using the read_continue() method.

        Argument1:  Tuple (list) of 8-bit code values for differential
                    input channel pins to read sequentially in a cycle.
                    (See definitions for the REG_MUX register)

                    Example:
                    ch_sequence=(POS_AIN0|NEG_AIN1, POS_AIN2|NEG_AINCOM)

        Argument2:  List (array, buffer) of signed integer conversion
                    results for the sequence of input channels.

        Returns:    List (array, buffer) of signed integer conversion
                    results for the sequence of input channels.

        This implements the timing sequence outlined in the ADS1256
        datasheet (Sept.2013) on page 21, figure 19: "Cycling the
        ADS1256 Input Multiplexer" for cyclic data acquisition.
        """
        self.mux = ch_sequence[0]
        self.sync()
        buf_len = len(ch_sequence)
        if ch_buffer is None:
            ch_buffer = [0] * buf_len
        for i in range(0, buf_len):
            ch_buffer[i] = self.read_and_next_is(ch_sequence[(i+1)%buf_len])
        return ch_buffer




