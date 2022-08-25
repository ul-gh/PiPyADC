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
import logging
import pigpio as io
from .ADS1256_definitions import *
from . import ADS1256_default_config


class ADS1256():
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
    pigpio_instance = None
    num_instances = 0
    occupied_cs_pins = []

    # Hardware pin configuration must be set at initialization phase.
    # Register/Configuration Flag settings are initialized, but these
    # can be changed during runtime via class properties.
    # Default config is read from external file (module).
    def __init__(self, conf=ADS1256_default_config, pi=None):
        if hasattr(conf, "LOGLEVEL"):
            logging.basicConfig(level=conf.LOGLEVEL)
        # Set up the pigpio object if not provided as an argument
        # or already present as a class attribute
        if pi is None:
            self.__class__.num_instances += 1
            if self.__class__.num_instances == 1:
                logging.debug("Creating new pigpio instance")
                self.__class__.pigpio_instance = io.pi()
        else:
            self.__class__.pigpio_instance = pi
        # This is just a short-cut
        self.pi = self.__class__.pigpio_instance

        if not self.pi.connected:
            raise IOError("Could not connect to hardware via pigpio library")

        # Config and initialize the SPI and GPIO pins used by the ADC.
        # The following four entries are actively used by the code:
        self.CS_PIN       = conf.CS_PIN
        self.DRDY_PIN     = conf.DRDY_PIN
        self.RESET_PIN    = conf.RESET_PIN
        self.PDWN_PIN     = conf.PDWN_PIN
        self.DRDY_TIMEOUT = conf.DRDY_TIMEOUT
        self.DRDY_DELAY   = conf.DRDY_DELAY

        if conf.CS_PIN in self.__class__.occupied_cs_pins:
            raise RuntimeError("Chip select pin is already in use. Check config!")
        self.__class__.occupied_cs_pins.append(conf.CS_PIN)
        if hasattr(conf, "CHIP_SELECT_GPIOS_INITIALIZE"):
            if type(conf.CHIP_SELECT_GPIOS_INITIALIZE) is int:
                cs_gpios = (conf.CHIP_SELECT_GPIOS_INITIALIZE, )
            else:
                cs_gpios = conf.CHIP_SELECT_GPIOS_INITIALIZE
            for pin in cs_gpios:
                if pin is not None:
                    logging.debug(f"Initializing chip select pin on GPIO no. {pin}")
                    self.pi.set_mode(pin, io.OUTPUT)
                    self.pi.write(pin, 1)

        # Only one GPIO input:
        if conf.DRDY_PIN is not None:
            self.DRDY_PIN = conf.DRDY_PIN
            self.pi.set_mode(conf.DRDY_PIN, io.INPUT)

        # GPIO Outputs.
        # If chip select pin is set to None, the ADC pin is assumed to be hardwired to GND.
        for pin in (conf.CS_PIN,
                    conf.RESET_PIN,
                    conf.PDWN_PIN,
                    ):
            if pin is not None:
                logging.debug(f"Setting pin to output: {pin}")
                self.pi.set_mode(pin, io.OUTPUT)
                self.pi.write(pin, 1)

        # SPI bus config
        logging.debug(f"Activating SPI using software chip select on GPIO number: {conf.CS_PIN}")
        # The ADS1256 uses SPI MODE=1 <=> CPOL=0, CPHA=1. 
        if hasattr(conf, "SPI_FLAGS"):
            spi_flags = conf.SPI_FLAGS
        else:
            #              bbbbbbRTnnnnWAuuupppmm
            spi_flags  = 0b0000000000000011100001 
        if hasattr(conf, "SPI_BUS") and conf.SPI_BUS == 1:
            spi_flags |= 0b0000000000000100000000 
        # PIGPIO library returns a numeric handle for each chip on this bus.
        # We do not use the HW chip select, thus channel argument is always zero.
        self.spi_id = self.pi.spi_open(0,
                                       conf.SPI_FREQUENCY,
                                       spi_flags
                                       )
        logging.debug(f"Obtained SPI devide handle ID: {self.spi_id}")
        if not self.spi_id >= 0:
            raise RuntimeError("SPI open error!")
        
        # ADS1255/ADS1256 command timing specifications. Do not change.
        # Delay between requesting data and reading the bus for
        # RDATA, RDATAC and RREG commands (datasheet: t_6 >= 50*CLKIN period).
        self._DATA_TIMEOUT = 1E-6 + 50/conf.CLKIN_FREQUENCY
        # Command-to-command timeout after SYNC and RDATAC
        # commands (datasheet: t11)
        self._SYNC_TIMEOUT = 1E-6 + 24/conf.CLKIN_FREQUENCY
        # See datasheet ADS1256: CS needs to remain low
        # for t_10 = 8*T_CLKIN after last SCLK falling edge of a command.
        # Because this delay is longer than timeout t_11 for the
        # RREG, WREG and RDATA commands of 4*T_CLKIN, we do not need
        # the extra t_11 timeout for these commands when using software
        # chip select selection and the _CS_TIMEOUT.
        self._CS_TIMEOUT   = 1E-6 + 8/conf.CLKIN_FREQUENCY
        # When using hardware/hard-wired chip select, still a command-
        # to command timeout of t_11 is needed as a minimum for the
        # RREG, WREG and RDATA commands.
        self._T_11_TIMEOUT   = 1E-6 + 4/conf.CLKIN_FREQUENCY

        self.v_ref         = conf.v_ref

        # Device reset for defined initial state
        if conf.CHIP_HARD_RESET_ON_START:
            self.hard_reset()
        else:
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

        # This invokes a getter function..
        chip_ID = self.chip_ID
        logging.debug(f"Chip ID: {chip_ID}")
        if chip_ID != 3:
            raise RuntimeError("Received wrong chip ID value for ADS1256. "
                               "Hardware connected?")


    def __del__(self):
        if self.__class__.num_instances > 1:
            self.__class__.num_instances -= 1
            logging.debug(f"Number of PIGPIO instances left: {self.__class__.num_instances}")
        elif self.__class__.num_instances == 1:
            logging.debug(f"Closing last PIGPIO instance")
            self.__class__.pigpio_instance.stop()
        # Else leaving external PIGPIO instance active


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

        read_continue()
            for cyclic reads of multiple channels at once - the ADC must not be
            reconfigured between invocations of this function, otherwise false
            data is read for the first value of the sequence

        read_sequence()
            for reading a succession of multiple channels at once, configuring
            all input channels including the first one for each cycle
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
            # self._send_byte() automatically truncates to uint8
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
            # self._send_byte() automatically truncates to uint8
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


    def _chip_select(self):
        # If chip select pin is hardwired to GND, do nothing.
        if self.CS_PIN is not None:
            self.pi.write(self.CS_PIN, 0)

    # Release chip select and implement t_11 timeout
    def _chip_release(self):
        if self.CS_PIN is not None:
            time.sleep(self._CS_TIMEOUT)
            self.pi.write(self.CS_PIN, 1)
        else:
            # The minimum t_11 timeout between commands, see datasheet Figure 1.
            time.sleep(self._T_11_TIMEOUT)

    def _send_byte(self, mybyte):
        # Sends a byte via the SPI bus
        self.pi.spi_write(self.spi_id, chr(mybyte&0xFF))

    def _read_byte(self):
        # Returns a byte read via the SPI bus
        ret_tuple = self.pi.spi_read(self.spi_id, 1)
        return ord(ret_tuple[1])


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
            drdy_level = self.pi.read(self.DRDY_PIN)
            while (drdy_level == 1) and (elapsed < self.DRDY_TIMEOUT):
                elapsed = time.time() - start
                drdy_level = self.pi.read(self.DRDY_PIN)
                # Sleep in order to avoid busy wait and reduce CPU load.
                time.sleep(self.DRDY_DELAY)
            if elapsed >= self.DRDY_TIMEOUT:
                logging.warning("Timeout while polling configured DRDY pin!")
        else:
            time.sleep(self.DRDY_TIMEOUT)




    def read_reg(self, register):
        """Returns data byte from the specified register
        
        Argument: register address
        """
        self._chip_select()
        self.pi.spi_write(self.spi_id, [CMD_RREG|register, 0x00])
        time.sleep(self._DATA_TIMEOUT)
        ret_tuple = self.pi.spi_read(self.spi_id, 1)
        # Release chip select and implement t_11 timeout
        self._chip_release()
        return ord(ret_tuple[1])


    def write_reg(self, register, data):
        """Writes data byte to the specified register
 
        Arguments: register address, data byte (uint_8)
        """
        self._chip_select()
        self.pi.spi_write(self.spi_id, [CMD_WREG|register, 0x00, data])
        # Release chip select and implement t_11 timeout
        self._chip_release()


    def cal_self_offset(self):
        """Perform an input zero calibration using chip-internal
        reference switches.

        Sets the ADS1255/ADS1256 OFC register.
        """
        self._chip_select()
        self.pi.spi_write(self.spi_id, [CMD_SELFOCAL])
        self.wait_DRDY()
        # Release chip select and implement t_11 timeout
        self._chip_release()


    def cal_self_gain(self):
        """Perform an input full-scale calibration
        using chip-internal circuitry connected to VREFP and VREFN.

        Sets the ADS1255/ADS1256 FSC register.
        """
        self._chip_select()
        self.pi.spi_write(self.spi_id, [CMD_SELFGCAL])
        self.wait_DRDY()
        # Release chip select and implement t_11 timeout
        self._chip_release()


    def cal_self(self):
        """Perform an input zero and full-scale two-point-calibration
        using chip-internal circuitry connected to VREFP and VREFN.

        Sets the ADS1255/ADS1256 OFC and FSC registers.
        """
        self._chip_select()
        self.pi.spi_write(self.spi_id, [CMD_SELFCAL])
        self.wait_DRDY()
        # Release chip select and implement t_11 timeout
        self._chip_release()


    def cal_system_offset(self):
        """Set the ADS1255/ADS1256 OFC register such that the
        current input voltage corresponds to a zero output value.
        The input multiplexer must be set to the appropriate pins first.
        """
        self._chip_select()
        self.pi.spi_write(self.spi_id, [CMD_SYSOCAL])
        self.wait_DRDY()
        # Release chip select and implement t_11 timeout
        self._chip_release()


    def cal_system_gain(self):
        """Set the ADS1255/ADS1256 FSC register such that the current
        input voltage corresponds to a full-scale output value.
        The input multiplexer must be set to the appropriate pins first.
        """
        self._chip_select()
        self.pi.spi_write(self.spi_id, [CMD_SYSGCAL])
        self.wait_DRDY()
        # Release chip select and implement t_11 timeout
        self._chip_release()


    def standby(self):
        """Put chip in low-power standby mode
        """
        self._chip_select()
        self.pi.spi_write(self.spi_id, [CMD_STANDBY])
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
        self.pi.spi_write(self.spi_id, [CMD_WAKEUP])
        # Release chip select and implement t_11 timeout
        self._chip_release()

    def reset(self):
        """Reset all registers except CLK0 and CLK1 bits
        to reset values and Polls for DRDY change / timeout afterwards.
        """
        self._chip_select()
        self.pi.spi_write(self.spi_id, [CMD_RESET])
        self.wait_DRDY()
        # Release chip select and implement t_11 timeout
        self._chip_release()

    def hard_reset(self):
        """Reset by toggling the hardware pin as configured as "RESET_PIN".
        """
        if self.RESET_PIN is None:
            raise RuntimeError("Reset pin is not configured!")
        else:
            logging.debug("Toggling reset pin...")
            self.pi.write(self.RESET_PIN, 0)
            time.sleep(100E-6)
            self.pi.write(self.RESET_PIN, 1)
            # At hardware initialisation, a settling time for the oscillator
            # is necessary before doing any register access.
            # This is approx. 30ms, according to the datasheet.
            time.sleep(0.03)
            self.wait_DRDY()
 

    def sync(self):
        """Restart the ADC conversion cycle with a SYNC + WAKEUP
        command sequence as described in the ADS1256 datasheet.
        
        This is useful to restart the acquisition cycle after rapid
        changes of the input signals, for example when using an
        external input multiplexer or after changing ADC configuration
        flags.
        """
        self._chip_select()
        self.pi.spi_write(self.spi_id, [CMD_SYNC])
        time.sleep(self._SYNC_TIMEOUT)
        self.pi.spi_write(self.spi_id, [CMD_WAKEUP])
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
        self.pi.spi_write(self.spi_id, [CMD_RDATA])
        # Wait through the data pause
        time.sleep(self._DATA_TIMEOUT)
        # The result is 24 bits little endian two's complement value by default
        (count, inbytes) = self.pi.spi_read(self.spi_id, 3)
        # Release chip select and implement t_11 timeout
        self._chip_release()

        # Concatenate the bytes
        int24_result = inbytes[0]<<16 | inbytes[1]<<8 | inbytes[2]
        # Take care of 24-bit two's complement
        if int24_result < 0x800000:
            return int24_result
        else:
            return int24_result - 0x1000000


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

        read_continue()
            for cyclic reads of multiple channels at once - the ADC must not be
            reconfigured between invocations of this function, otherwise false
            data is read for the first value of the sequence

        read_sequence()
            for reading a succession of multiple channels at once, configuring
            all input channels including the first one for each cycle
        """
        self._chip_select()
        # Set input pin mux position for this cycle"
        self.pi.spi_write(
            self.spi_id, [CMD_WREG|REG_MUX, 0x00, diff_channel, CMD_SYNC])
        time.sleep(self._SYNC_TIMEOUT)
        self.pi.spi_write(self.spi_id, [CMD_WAKEUP])

        self.wait_DRDY()
        # Read data from ADC, which still returns the /previous/ conversion
        # result from before changing inputs
        self.pi.spi_write(self.spi_id, [CMD_RDATA])
        time.sleep(self._DATA_TIMEOUT)
        # The result is 24 bits little endian two's complement value by default
        (count, inbytes) = self.pi.spi_read(self.spi_id, 3)

        # Release chip select and implement t_11 timeout
        self._chip_release()

        # Concatenate the bytes
        int24_result = inbytes[0]<<16 | inbytes[1]<<8 | inbytes[2]
        # Take care of 24-bit two's complement
        if int24_result < 0x800000:
            return int24_result
        else:
            return int24_result - 0x1000000


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
        self.pi.spi_write(
            self.spi_id, [CMD_WREG|REG_MUX, 0x00, diff_channel, CMD_SYNC])
        time.sleep(self._SYNC_TIMEOUT)
        self.pi.spi_write(self.spi_id, [CMD_WAKEUP])
        # The datasheet is a bit unclear if a t_11 timeout is needed here.
        # Assuming the extra timeout is the safe choice:
        time.sleep(self._T_11_TIMEOUT)

        # Read data from ADC, which still returns the /previous/ conversion
        # result from before changing inputs
        self.pi.spi_write(self.spi_id, [CMD_RDATA])
        time.sleep(self._DATA_TIMEOUT)
        # The result is 24 bits little endian two's complement value by default
        (count, inbytes) = self.pi.spi_read(self.spi_id, 3)

        # Release chip select and implement t_11 timeout
        self._chip_release()

        # Concatenate the bytes
        int24_result = inbytes[0]<<16 | inbytes[1]<<8 | inbytes[2]
        # Take care of 24-bit two's complement
        if int24_result < 0x800000:
            return int24_result
        else:
            return int24_result - 0x1000000


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




