#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

__author__ = 'Rodrigo Muñoz'

import struct
import time
from array import array

class Message(object):
    """Base class for messages"""
    def serialize(self):
        pass
    
    def deserialize(self, data):
        pass

    def check(self):
        pass

    def __str__(self):
        return ''

class MessageException(Exception):
    """Base exception type for errors"""

class DeserializationError(MessageException):
    """Message deserialization error"""

class SerializationError(MessageException):
    """Message serialization error"""

class CheckError(MessageException):
    """Error on message fields"""

class DuckietownCommand(Message):
    # Struct type
    # Little endian
    # DuckietownCommand [uint8_t, int16_t, int16_t] 
    _struct = struct.Struct('<Bhh')
    PWM_MIN = -255
    PWM_MAX = 255

    def __init__(self):
        self.led = False
        self.pwm_ch1 = 0
        self.pwm_ch2 = 0
        self.timestamp = time.time()

    def check(self):
        # Check led
        if not isinstance(self.led, bool):
            raise CheckError('Field message.led is not a bool')
        # Check PWM channel 1
        if isinstance(self.pwm_ch1, int):
            # Limit value
            self.pwm_ch1 = max(min(self.pwm_ch1, DuckietownCommand.PWM_MAX), DuckietownCommand.PWM_MIN)
        else:
            raise CheckError('Field message.pwm_ch1 is not a int')
        # Check PWM channel 2
        if isinstance(self.pwm_ch2, int):
            # Limit value
            self.pwm_ch2 = max(min(self.pwm_ch2, DuckietownCommand.PWM_MAX), DuckietownCommand.PWM_MIN)
        else:
            raise CheckError('Field message.pwm_ch2 is not a int')


    def serialize(self):
        try:
            self.check()
            return  [ord(a) for a in list(DuckietownCommand._struct.pack(int(self.led), self.pwm_ch1, self.pwm_ch2))]
        except struct.error:
            raise SerializationError('Error in serialization {}'.format(self.__str__))
        
    def deserialize(self, data):
        pass

    def __str__(self):
        return 'led: {}\npwm_ch1: {:d}\npwm_ch2: {:d}'.format(self.led, self.pwm_ch1, self.pwm_ch2)


class DuckietownStatus(Message):
    # Struct type
    # Little endian
    # DuckietownStatus [uint8_t, int16_t, int16_t] 
    _struct = struct.Struct('<Bhh')

    def __init__(self):
        self.led = False
        self.pwm_ch1 = 0
        self.pwm_ch2 = 0
        self.stamp = 0.0

    def check(self):
        pass

    def serialize(self):
        pass
        
    def deserialize(self, data):
        data_tuple = DuckietownStatus._struct.unpack(array('B', data).tostring())
        self.led = bool(data_tuple[0])
        self.pwm_ch1 = data_tuple[1]
        self.pwm_ch2 = data_tuple[2]
        self.stamp = time.time()

    def __str__(self):
        return 'led: {}\npwm_ch1: {:d}\npwm_ch2: {:d}\nstamp: {:f}'.format(self.led, self.pwm_ch1, self.pwm_ch2, self.stamp)


def to_hex(data):
    return ":".join("{:02x}".format(ord(c)) for c in data)

def test():
    cmd = DuckietownCommand()
    cmd.led = True
    print(cmd.serialize())
    cmd.pwm_ch1 = -200
    cmd.pwm_ch2 = 100
    print(cmd.serialize())
    cmd.pwm_ch1 = -300
    cmd.pwm_ch2 = 100
    print(cmd.serialize())
    print(cmd)

if __name__ == '__main__':
    test()
