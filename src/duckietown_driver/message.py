#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division

__author__ = 'Rodrigo Muñoz'

import struct

class Message(object):
    """Base class for messages"""
    def serialize(self, buff):
        pass
    
    def deserialize(self, buff):
    	pass

    def __str__(self):
        return ''

class MessageException(Exception):
    """Base exception type for errors"""
    pass

class DeserializationError(MessageException):
    """Message deserialization error"""
    pass

class SerializationError(MessageException):
    """Message serialization error"""
    pass

class DuckietownCommand(Message):
    # Struct type
    # Little endian
    # DuckietownCommand [uint8_t, int16_t, int16_t] 
    _struct = struct.Struct('<Bhh')

    def __init__(self):
        self.led = False
        self.pwm_ch1 = 0
        self.pwm_ch2 = 0

    def serialize(self):
        try:
            return  [ord(a) for a in list(DuckietownCommand._struct.pack(int(self.led), self.pwm_ch1, self.pwm_ch2))]
        except struct.error as se:
            raise SerializationError('Error in serialization %s' % (self.__str__))
        
    def deserialize(self, buff):
        pass

    def __str__(self):
        return 'led: {}\npwm_ch1: {:d}\npwm_ch2: {:d}'.format(self.led, self.pwm_ch1, self.pwm_ch2)

def to_hex(data):
    return ":".join("{:02x}".format(ord(c)) for c in data)

def test():
    cmd = DuckietownCommand()
    cmd.led = True
    print(cmd.serialize())
    cmd.pwm_ch1 = -200
    cmd.pwm_ch2 = 100
    print(cmd.serialize())

if __name__ == '__main__':
    test()
