#!/usr/bin/python
from __future__ import division
import serial
import struct

"""Pololu driver module for motor controllers using pyserial

This module handles the lower level logic of writing to a pololu motor
controller using python.

Serial reference: https://www.pololu.com/docs/0J44/6.2.1
Get Variable reference: https://www.pololu.com/docs/0J44/6.4

Example:
Here is an example of usage. The first Daisy object sets the port and
all the of the rest use the same port.
    ::
        motor0 = Daisy(0, port="/dev/ttyUSB0") # only first one has to be set
        motor1 = Daisy(1) # all following will use the first port
        motor2 = Daisy(2)

        motor1.forward(1600) # drive forward the motor with device number 1

        # All motors are using port "/dev/ttyUSB0" to send commands
        # Each device is addressed by their device number
"""


BAUD_SYNC = chr(0x80)  # Required to sync the baud rate for older devices
PROTOCOL = chr(0xAA)  # First part of command to write using Pololu Protocol
FORWARD = chr(0x05)  # Drive motor forward command
BACKWARD = chr(0x06)  # Drive motor reverse command
START = chr(0x03)  # Exit Safe-Start mode so the motor can run
STOP = chr(0x60)   # Stops the motor and enters Safe-Start mode



class Daisy(object):
    """Represents a single Pololu Simple Motor Controller in a daisy chain

    This class offers an interface to daisy chain several Pololu Simple
    Motor Controllers. The daisy chained modules all share the same
    serial connection.  To target specific devices, every command
    is sent with the device number of the device.  You must configure the
    devices to have different numbers. This must be done using the Simple
    Motor Controller setup softare (https://www.pololu.com/docs/0J44/3).

    Initialize every separate board you want to talk to with its
    set device number.


    Attributes:
        dev_num (int): Device number of Pololu board to be commanded
        flip (bool): Polarity of motor. False if normal configuration
        port (str): Serial port of all devices chained
    """

    count = 0  # static variable to keep track of number of Daisys open
    ser = None  # initialize static variable ser to None
    first_flip = False # first

    def __init__(self, dev_num, flip=False, port="/dev/ttyUSB0", crc_enabled=False, timeout=0.1):
        """Set motor id and open serial connection if not already open"""
        if dev_num < 0 or dev_num > 127:
            raise Exception("Invalid motor id, must set to id of motor (0-127) for daisy chaining")

        Daisy.count += 1  # increment count of controllers
        self.dev_num = dev_num  # set device number to use in other commands
        self.crc_enabled = crc_enabled

        # if serial connection has not been made yet
        if Daisy.ser is None or not Daisy.ser.isOpen():
            Daisy.first_flip = flip
            Daisy.ser = serial.Serial(port, baudrate=115200, timeout=timeout)
            Daisy.ser.write(BAUD_SYNC)  # sync old devices by writing 0x80
        if flip != Daisy.first_flip:
            print("WARN: your motors have different flip (polarity) values")
        self.flip = flip
        self._exit_safe_start()  # make it so pololu reacts to commands

    def __del__(self):
        """Decrement count, stop motor, and close port if it's the last connection"""
        Daisy.count -= 1  # decrement count of controllers
        self._stop_motor()  # safely stop current motor
        # if this is the last controller open
        if Daisy.count <= 0:
            if Daisy.ser is not None:
                Daisy.ser.close()
                print("Serial connection closed")

    def _send_command(self, command, databyte3, databyte4):
        """Sends a two-byte command using the Pololu protocol."""
        cmd = PROTOCOL + chr(self.dev_num) + command + chr(databyte3) + chr(databyte4)
        if self.crc_enabled:
            cmd = self.crc7(cmd) # Calculate and append Cyclic Redundancy Check byte

        Daisy.ser.write(cmd)

    def _send_command_single(self, command):
        """Sends a one-byte command using the Pololu protocol."""
        cmd = PROTOCOL + chr(self.dev_num) + command
        if self.crc_enabled:
            cmd = self.crc7(cmd) # Calculate and append Cyclic Redundancy Check byte

        Daisy.ser.write(cmd)


    def crc7(self,cmd_str):
        """
        Calculates and appends the Cyclic Redundancy Check (CRC7) byte for error checking
        """
        l = len(cmd_str)

        int_tuple = struct.unpack('B'*len(cmd_str), cmd_str)
        divd = self.__bitrev(int_tuple)

        if(l>4):
            print " This CRC function currently does not support strings > 4 chars"
            return 0

        divd = self.__bitrev(ord(cmd_str[0]))

            # put the chars in an integer
        for i in range(1,l):
              new = self.__bitrev(ord(cmd_str[i]))
              divd <<= 8
              divd = divd | new

                #crc = 0b10001001<<(8*(l-1))
              #hex instead
              crc = int('10001001',2)<<(8*(l-1)) #J binary literals don't work in python 2.5
              lsbcheck = 0x80 << (8*(l-1))

              for i in range(0,8*l):
                  if(divd & lsbcheck == lsbcheck):
                      divd = divd ^ crc
                      divd = divd << 1
                  else:
                      divd = divd<<1

              divd = divd>>(8*(l-1))
              divd = self.__bitrev(divd)
              s = chr(divd & 0xff)
        return cmd_str + s

    def __bitrev(self,bytes):
        """
        Creates a lookup table of reversed bit orders

        Input:
           bytes -- tuple -- tuple of 1 byte values to be reversed
        Output:
           bitrev_table -- dict
        """
        bytes = sum(bytes)         # Sums the bytes
        bin_repr = bin(bytes)[2:]  # Convert to binary string, remove "0b" at the beginning of the string
        bin_repr = bin_repr[::-1]  # Reverse all digits
        bin_repr = "0b%s" % bin_repr

        return int(bin_repr,2)     # Convert back to int, and return

    def _exit_safe_start(self):
        """Exit safe start so you can freely send commands to Pololu.
        This must be run before run other commands."""
        self._send_command_single(START)

    def _stop_motor(self):
        """Immediately stops the motor and enters safe start mode"""
        self._send_command_single(STOP)

    def _get_variable(self, variable_id):
        """Get variable from pololu device"""
        cmd = PROTOCOL + chr(self.dev_num) + chr(0x21) + chr(variable_id)
        Daisy.ser.write(cmd)

        low_byte = Daisy.ser.read()
        if (low_byte == ''): # if the board didn't write anything to serial, bail
            Daisy.ser.flushInput()
            return 0
        low_byte = ord(low_byte) # variable low byte
        high_byte = 256 * ord(Daisy.ser.read())
        return high_byte + low_byte




    # USER METHODS

    # VARIABLE GETTERS
    def get_error_status(self):
        """Get the error status of the device. Return string `OK` if all good,
        else return the string of the exact error."""
        var_id = 0
        val = self._get_variable(var_id)
        if val == 0:
            return "OK"
        else:
            error_msg = "ERROR STATUS: "
            error_msg += "safe start mode, " if val & 2**0 else ""
            error_msg += "request channel invalid, " if val & 2**1 else ""
            error_msg += "serial error, " if val & 2**2 else ""
            error_msg += "command timeout, " if val &  2**3 else ""
            error_msg += "limit/kill switch, " if val & 2**4 else ""
            error_msg += "low vin, " if val & 2**5 else ""
            error_msg += "high vin, " if val & 2**6 else ""
            error_msg += "over temperature, " if val & 2**7 else ""
            error_msg += "motor drive error, " if val & 2**8 else ""
            error_msg += "err line high, " if val & 2**9 else ""
            error_msg = error_msg[:-2]
            return error_msg

    def get_serial_errors(self):
        """Check if there were any serial errors, if none return "OK, else
        give return a listing the error that occured, such as a FRAME error"""
        var_id = 2
        val = self._get_variable(var_id)
        if val == 0:
            return "OK" # No errors
        else:
            # Create an error message. If any of the bits are set, their
            # error message gets appended, else an empty string is appended
            error_msg = "SERIAL ERROR: "
            error_msg += "frame, " if val & 2**1 else ""
            error_msg += "noise, " if val & 2**2 else ""
            error_msg += "rx overrun, " if val & 2**3 else ""
            error_msg += "format, " if val & 2**4 else ""
            error_msg += "crc, " if val & 2**5 else ""
            error_msg = error_msg[:-2] # remove the last comma and space
            return error_msg

    def get_limit_statuses(self):
        """Return a list of two bools, which are true if the limit switches
        (AN1 and AN2) are active"""
        var_id = 3
        val = self._get_variable(var_id)
        an1_active = bool(val & 2**7) # bit mask to check if 7th bit is set
        an2_active = bool(val & 2**8) # bit mask to check if 8th bit is set
        return an1_active, an2_active

    def get_target_speed(self):
        """Motor target speed (-3200 to +3200) requested by the controlling interface."""
        var_id = 20
        unsigned = self._get_variable(var_id)
        if (unsigned > 3200):
            signed = unsigned - 2**16
        else:
            signed = unsigned
        return signed

    def get_speed(self):
        """Current speed of the motor (-3200 to +3200), 16 bit"""
        var_id = 21
        unsigned = self._get_variable(var_id)
        if (unsigned > 3200):
            signed = unsigned - 2**16
        else:
            signed = unsigned
        return unsigned

    def get_input_voltage(self):
        """Get the measured voltage on the VIN pin"""
        var_id = 23
        return self._get_variable(var_id) / 994.0 # some arbitrary constant from the boards

    def get_board_temperature(self):
        """Board temperature as measured by a temperature sensor near
        the motor driver. Returned in degrees Celsius"""
        var_id = 24
        val = self._get_variable(var_id)
        return val * 10 # scale up 0.1C to 1C

    # USER CONTROL
    def forward(self, speed):
        """Drive motor forward at specified speed (0 to 3200)"""
        speed = max(min(3200, speed), 0)  # enforce bounds

        self._exit_safe_start()
        # This is how the documentation recommends doing it.
        # The 1st byte will be from 0 to 31
        # The 2nd byte will be from 0 to 100
        cmd1 = speed % 32
        cmd2 = speed // 32
        if self.flip:
            self._send_command(BACKWARD, cmd1, cmd2)  # low bytes, high bytes
        else:
            self._send_command(FORWARD, cmd1, cmd2)  # low bytes, high bytes

    def backward(self, speed):
        """Drive motor backward (reverse) at specified speed (0 to 3200)"""
        speed = max(min(3200, speed), 0)  # enforce bounds
        self._exit_safe_start()
        cmd1 = speed % 32
        cmd2 = speed // 32
        if self.flip:
            self._send_command(FORWARD, cmd1, cmd2)  # low bytes, high bytes
        else:
            self._send_command(BACKWARD, cmd1, cmd2)  # low bytes, high bytes

    def drive(self, speed):
        """Drive motor in direction based on speed (-3200, 3200)"""
        if speed < 0:
            self.backward(-speed)
        else:
            self.forward(speed)

    def stop(self):
        """Stop the motor"""
        self._stop_motor()
