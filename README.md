# PiPyADC
Python module for interfacing Texas Instruments SPI bus based analog-
to-digital converters with the Raspberry Pi.

Currently only implemented class in this module is ADS1256 for the
ADS1255 and ADS1256 chips which are register- and command compatible.
The version implemented uses software/GPIO setting of the chip select signal
instead of using the SPI hardware. This accounts for the hardware specifics
of the "Waveshare High-Precision-AD-DA-Board".

The ADS1255 and ADS1256 library is an implementation based purely on
Python and the WiringPi API. On a Raspberry Pi 3, multi-channel ADC
reads require a time overhead of aprox. 300...500 µs which makes the
library suitable for low-speed, high resolution data acquisition.

Download: https://github.com/ul-gh/PiPyADC

Depends on WiringPi library, see:
https://github.com/WiringPi/WiringPi-Python

Uses code from: https://github.com/heathsd/PyADS1256

License: GNU LGPLv2.1, see:
https://www.gnu.org/licenses/old-licenses/lgpl-2.1-standalone.html

Ulrich Lukas, 2019-02-02

## Run example on Raspbian Stretch:
### Install wiringpi library:
	sudo apt install python-pip
	sudo pip install wiringpi
### Activate SPI bus and reboot system:
	sudo sed -E -i s/"(#)(dtparam=spi).*"/"\2=on"/ /boot/config.txt
	sudo reboot
### Run example:
	sudo python example.py

## Example 2 added:
+ Importing configuration files for configuration of more than one ADC
+ Register access via instance properties
+ Reading ADC samples directly into a NumPy Array.


## ADS1256
```
class ADS1256(__builtin__.object)
|  Python class for interfacing the ADS1256 and ADS1255 analog to
|  digital converters with the Raspberry Pi.
|  
|  This is part of module PiPyADC
|  Download: https://github.com/ul-gh/PiPyADC
|  
|  Default pin and settings configuration is for the Open Hardware
|  "Waveshare High-Precision AD/DA Board"
|  
|  See file ADS1256_default_config.py for
|  configuration settings and description.
|  
|  Register read/write access is implemented via Python class/instance
|  properties. Commands are implemented as functions.
|  
|  See help(ADS1256) for usage of the properties and functions herein.
|  
|  See ADS1256_definitions.py for chip registers, flags and commands.
|  
|  Documentation source: Texas Instruments ADS1255/ADS1256
|  datasheet SBAS288: http://www.ti.com/lit/ds/sbas288j/sbas288j.pdf
|  
|  Methods defined here:
|  
|  __init__(self, conf=<module 'PiPyADC.ADS1256_default_config' from 'PiPyADC/ADS1256_default_config.pyc'>)
|      # Constructor for the ADC object: Hardware pin configuration must be
|      # set up at initialization phase and can not be changed later.
|      # Register/Configuration Flag settings are initialized, but these
|      # can be changed during runtime via class properties.
|      # Default config is read from external file (module) import
|  
|  cal_self(self)
|      Perform an input zero and full-scale two-point-calibration
|      using chip-internal circuitry connected to VREFP and VREFN.
|      
|      Sets the ADS1255/ADS1256 OFC and FSC registers.
|  
|  cal_self_gain(self)
|      Perform an input full-scale calibration
|      using chip-internal circuitry connected to VREFP and VREFN.
|      
|      Sets the ADS1255/ADS1256 FSC register.
|  
|  cal_self_offset(self)
|      Perform an input zero calibration using chip-internal
|      reference switches.
|      
|      Sets the ADS1255/ADS1256 OFC register.
|  
|  cal_system_gain(self)
|      Set the ADS1255/ADS1256 FSC register such that the current
|      input voltage corresponds to a full-scale output value.
|      The input multiplexer must be set to the appropriate pins first.
|  
|  cal_system_offset(self)
|      Set the ADS1255/ADS1256 OFC register such that the
|      current input voltage corresponds to a zero output value.
|      The input multiplexer must be set to the appropriate pins first.
|  
|  read_and_next_is(self, diff_channel)
|      Reads ADC data of presently running or already finished
|      conversion, sets and synchronises new input channel config
|      for next sequential read.
|      
|      Arguments:  8-bit code value for differential input channel
|                      (See definitions for the REG_MUX register)
|      Returns:    Signed integer conversion result for present read
|      
|      This enables rapid dycling through different channels and
|      implements the timing sequence outlined in the ADS1256
|      datasheet (Sept.2013) on page 21, figure 19: "Cycling the
|      ADS1256 Input Multiplexer".
|      
|      Note: In most cases, a fixed sequence of input channels is known
|      beforehand. For that case, this module implements the function:
|      
|      read_sequence(ch_sequence)
|          which automates the process for cyclic data acquisition.
|  
|  read_async(self)
|      Read ADC result as soon as possible
|      
|      Arguments:  None
|      Returns:    Signed integer ADC conversion result
|      
|      Issue this command to read a single conversion result for a
|      previously set /and stable/ input channel configuration.
|      
|      For the default, free-running mode of the ADC, this means
|      invalid data is returned when not synchronising acquisition
|      and input channel configuration changes.
|      
|      To avoid this, after changing input channel configuration or
|      with an external hardware multiplexer, use the hardware SYNC
|      input pin or use the sync() method to restart the
|      conversion cycle before calling read_async().
|      
|      Because this function does not implicitly restart a running
|      acquisition, it is faster that the read_oneshot() method.
|  
|  read_oneshot(self, diff_channel)
|      Restart/re-sync ADC and read the specified input pin pair.
|      
|      Arguments:  8-bit code value for differential input channel
|                      (See definitions for the REG_MUX register)
|      Returns:    Signed integer conversion result
|      
|      Use this function after waking up from STANDBY mode.
|      
|      When switching inputs during a running conversion cycle,
|      invalid data is acquired.
|      
|      To prevent this, this function automatically restarts the
|      conversion cycle after configuring the input channels.
|      
|      The resulting delay can be avoided. See functions:
|      
|      read_and_next_is(diff_channel)
|          for cyclic single-channel reads and:
|      
|      read_sequence()
|          for cyclic reads of multiple channels at once.
|  
|  read_reg(self, register)
|      Returns data byte from the specified register
|      
|      Argument: register address
|  
|  read_sequence(self, ch_sequence, ch_buffer=None)
|      Reads a sequence of ADC input channel pin pairs
|      
|      Argument1:  Tuple (list) of 8-bit code values for differential
|                  input channel pins to read sequentially in a cycle.
|                  (See definitions for the REG_MUX register)
|      
|                  This argument is optional. Defaults are read from
|                  configuration file and can be set via property
|                  "ch_sequence".
|      
|      Argument2:  List (array, vector, buffer) of signed integer conversion
|                  results for the sequence of input channels.
|      
|      Returns:    List (array, vector, buffer) of signed integer conversion
|                  results for the sequence of input channels.
|      
|      This implements the timing sequence outlined in the ADS1256
|      datasheet (Sept.2013) on page 21, figure 19: "Cycling the
|      ADS1256 Input Multiplexer" for cyclic data acquisition.
|  
|  reset(self)
|      Reset all registers except CLK0 and CLK1 bits
|      to reset values and Polls for DRDY change / timeout afterwards.
|  
|  standby(self)
|      Put chip in low-power standby mode
|  
|  sync(self)
|      Restart the ADC conversion cycle with a SYNC + WAKEUP
|      command sequence as described in the ADS1256 datasheet.
|      
|      This is useful to restart the acquisition cycle after rapid
|      changes of the input signals, for example when using an
|      external input multiplexer or after changing ADC configuration
|      flags.
|  
|  wait_DRDY(self)
|      Delays until the configured DRDY input pin is pulled to
|      active logic low level by the ADS1256 hardware or until the
|      DRDY_TIMEOUT in seconds has passed.
|      
|      Arguments: None
|      Returns: None
|      
|      The minimum necessary DRDY_TIMEOUT when not using the hardware
|      pin, can be up to approx. one and a half second, see datasheet..
|      
|      Manually invoking this function is necessary when using the
|      automatic calibration feature (ACAL flag). Then, use wait_DRDY()
|      after every access that changes the PGA gain bits in
|      ADCON register, the DRATE register or the BUFFEN flag.
|  
|  wakeup(self)
|      Wake up the chip from standby mode.
|      See datasheet for settling time specifications after wake-up.
|      Data is ready when the DRDY pin becomes active low.
|      
|      You can then use the read_oneshot() function to read a new
|      sample of input data.
|      
|      Call standby() to enter standby mode again.
|  
|  write_reg(self, register, data)
|      Writes data byte to the specified register
|      
|      Arguments: register address, data byte (uint_8)
|  
|  ----------------------------------------------------------------------
|  Data descriptors defined here:
|  
|  __dict__
|      dictionary for instance variables (if defined)
|  
|  __weakref__
|      list of weak references to the object (if defined)
|  
|  adcon
|      Get/Set value of the ADC configuration register, REG_ADCON.
|      Note: When the AUTOCAL flag is enabled, this causes a
|      wait_DRDY() timeout.
|  
|  chip_ID
|      Get the numeric ID from the ADS chip.
|      Useful to check if hardware is connected.
|      
|      Value for the ADS1256 on the Waveshare board seems to be a 3.
|  
|  drate
|      Get/Set value of the ADC output sample data rate by setting
|      the DRATE register (REG_DRATE).
|      This configures the hardware integrated moving average filter.
|      
|      When changing the register during a running acquisition,
|      invalid data is sampled. In this case, call the sync() method
|      to restart the acquisition cycle.
|      
|      The available data rates are defined in ADS1256_definitions.py.
|  
|  fsc
|      Get/Set the three full-scale adjustment registers, OFC0..2.
|      This property is supposed to be a positive integer value.
|      Gets/sets 24-bit unsigned int value in three 8-bit-registers.
|  
|  gpio
|      Get the logic level of the four GPIO pins, returned as
|      a four-bit bitmask or Set the status of the GPIO register,
|      REG_IO, where the most significant four bits represent the
|      pin direction, and the least significant four bits determine
|      the output logic level.
|      A timeout/debounce for the reading is not implemented.
|  
|  mux
|      Get/Set value of ADC analog input multiplexer register,
|      REG_MUX, used for selecting any arbitrary pair of input pins
|      as a differential input channel. For single-ended measurements,
|      choose NEG_AINCOM as the second input pin.
|      
|      The most significant four bits select the positive input pin.
|      The least significant four bits select the negative input pin.
|      
|      Example: ads1256.mux = POS_AIN4 | NEG_AINCOM.
|      
|      IMPORTANT:
|      
|      When switching inputs during a running conversion cycle,
|      invalid data is acquired.
|      
|      To prevent this, you must restart the conversion using the
|      sync() function or the SYNC hardware pin before doing an
|      async_read().
|      
|      The resulting delay can be avoided. See functions:
|      
|      read_and_next_is(diff_channel)
|          for cyclic single-channel reads and:
|      
|      read_sequence()
|          for cyclic reads of multiple channels at once.
|  
|  ofc
|      Get/Set the three offset compensation registers, OFC0..2.
|      This property is supposed to be a signed integer value.
|      Gets/sets 24-bit two's complement value in three 8-bit-registers.
|  
|  pga_gain
|      Get/Set ADC programmable gain amplifier setting.
|      
|      The available options for the ADS1256 are:
|      1, 2, 4, 8, 16, 32 and 64.
|      
|      This function sets the ADCON register with the code values
|      defined in file ADS1256_definitions.py.
|      
|      Note: When changing the gain setting at runtime, with activated
|      ACAL flag (AUTOCAL_ENABLE), this causes a Wait_DRDY() timeout
|      for the calibration process to finish.
|  
|  status
|      Get/Set value of ADC status register, REG_STATUS (8 bit).
|      For available settings flag options, see datasheet and file
|      ADS1256_definitions.py. Note: When enabling the AUTOCAL
|      flag, any subsequent access to the BUFEN flag, DRATE register
|      (drate property) or PGA gain setting (gain property) will cause
|      an additional delay for completion of hardware auto-calibration.
|  
|  v_per_digit
|      Get ADC LSB weight in volts per numeric output digit.
|      Readonly: This is a convenience value calculated from
|      gain and v_ref setting.
|  
|  v_ref
|      Get/Set ADC analog reference input voltage differential.
|      This is only for calculation of output value scale factor.
```
