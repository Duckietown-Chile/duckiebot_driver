#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = 'Rodrigo Munoz'

import time
import serial
from array import array
from threading import Lock

# Instruction Set
DXL_PING = 1
DXL_READ_DATA = 2
DXL_WRITE_DATA = 3
DXL_REG_WRITE = 4
DXL_ACTION = 5
DXL_RESET = 6
DXL_SYNC_WRITE = 131

class DuckietownSerial(object):
    """
    Provides low level control for Arduino based Ducktown robots
    """

    def __init__(self, port, baudrate):
        try:
            self.serial_mutex = Lock()
            self.ser = None
            self.ser = serial.Serial(port)
            self.ser.setTimeout(0.015)
            self.ser.baudrate = baudrate
            self.port_name = port
        except serial.SerialException:
            raise

    def __del__(self):
        self.close()

    def close(self):
        if self.ser:
            self.ser.flushInput()
            self.ser.flushOutput()
            self.ser.close()

    def __write_serial(self, data):
        self.ser.flushInput()
        self.ser.flushOutput()
        self.ser.write(data)

    def __read_response(self, servo_id):
        data = []

        try:
            data.extend(self.ser.read(4))
            if not data[0:2] == ['\xff', '\xff']: raise Exception('Wrong packet prefix %s' % data[0:2])
            data.extend(self.ser.read(ord(data[3])))
            data = array('B', ''.join(data)).tolist()
        except Exception as e:
            raise DroppedPacketError('Invalid response received from Duckietown %d. %s' % (servo_id, e))

        # verify checksum
        checksum = 255 - sum(data[2:-1]) % 256
        if not checksum == data[-1]: raise ChecksumError(servo_id, data, checksum)

        return data

    def read(self, servo_id, address, size):
        """ Read "size" bytes of data from servo with "servo_id" starting at the
        register with "address". "address" is an integer between 0 and 57. It is
        recommended to use the constants in module dynamixel_const for readability.

        To read the position from servo with id 1, the method should be called
        like:
            read(1, DXL_GOAL_POSITION_L, 2)
        """
        # Number of bytes following standard header (0xFF, 0xFF, id, length)
        length = 4  # instruction, address, size, checksum

        # directly from AX-12 manual:
        # Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
        # If the calculated value is > 255, the lower byte is the check sum.
        checksum = 255 - ( (servo_id + length + DXL_READ_DATA + address + size) % 256 )

        # packet: FF  FF  ID LENGTH INSTRUCTION PARAM_1 ... CHECKSUM
        packet = [0xFF, 0xFF, servo_id, length, DXL_READ_DATA, address, size, checksum]
        packetStr = array('B', packet).tostring() # same as: packetStr = ''.join([chr(byte) for byte in packet])

        with self.serial_mutex:
            self.__write_serial(packetStr)

            # wait for response packet from the Duckietown driver
            timestamp = time.time()
            time.sleep(0.0013)#0.00235)

            # read response
            data = self.__read_response(servo_id)
            data.append(timestamp)

        return data

    def write(self, servo_id, address, data):
        # Number of bytes following standard header (0xFF, 0xFF, id, length)
        length = 3 + len(data)  # instruction, address, len(data), checksum

        # directly from AX-12 manual:
        # Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
        # If the calculated value is > 255, the lower byte is the check sum.
        checksum = 255 - ((servo_id + length + DXL_WRITE_DATA + address + sum(data)) % 256)

        # packet: FF  FF  ID LENGTH INSTRUCTION PARAM_1 ... CHECKSUM
        packet = [0xFF, 0xFF, servo_id, length, DXL_WRITE_DATA, address]
        packet.extend(data)
        packet.append(checksum)

        packetStr = array('B', packet).tostring() # packetStr = ''.join([chr(byte) for byte in packet])

        with self.serial_mutex:
            self.__write_serial(packetStr)

            # wait for response packet from the motor
            timestamp = time.time()
            time.sleep(0.0013)

            # read response
            data = self.__read_response(servo_id)
            data.append(timestamp)

        return data

    def ping(self, servo_id):
        """
        Ping
        """
        # Number of bytes following standard header (0xFF, 0xFF, id, length)
        length = 2  # instruction, checksum

        # directly from AX-12 manual:
        # Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
        # If the calculated value is > 255, the lower byte is the check sum.
        checksum = 255 - ((servo_id + length + DXL_PING) % 256)

        # packet: FF  FF  ID LENGTH INSTRUCTION CHECKSUM
        packet = [0xFF, 0xFF, servo_id, length, DXL_PING, checksum]
        packetStr = array('B', packet).tostring()

        with self.serial_mutex:
            self.__write_serial(packetStr)

            # wait for response packet from the motor
            timestamp = time.time()
            time.sleep(0.0013)

            # read response
            try:
                response = self.__read_response(servo_id)
                response.append(timestamp)
            except Exception:
                response = []
        return response

    def set_led(self, state):
        """
        Set LED state
        """
        LED_REG = 7
        response = self.write(1, LED_REG, [state])
        return response


class ChecksumError(Exception):
    def __init__(self, servo_id, response, checksum):
        Exception.__init__(self)
        self.message = 'Checksum received from Duckietown robot %d does not match the expected one (%d != %d)' \
                       %(servo_id, response[-1], checksum)
        self.response_data = response
        self.expected_checksum = checksum
    def __str__(self):
        return self.message

class DroppedPacketError(Exception):
    def __init__(self, message):
        Exception.__init__(self)
        self.message = message
    def __str__(self):
        return self.message


