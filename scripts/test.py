#!/usr/bin/python

import sys
import time
from duckietown_driver.serial_interface import DuckietownSerial
import struct
_struct = struct.Struct('<l')

def test_ping(duck, dev_id):
    result = None
    try:
        result = duck.ping(dev_id)
    except Exception as ex:
        print 'Exception thrown while pinging device %d - %s' % (dev_id, ex)
    
    if result:
        print 'Found %d' % (dev_id)
    return result

def test_read(duck, dev_id, address):
    result = None
    try:
        result = duck.read(dev_id, address,1)
    except Exception as ex:
        print 'Exception thrown while reading addres %d' % (address)

    if result:
        print 'Address %d = %d' % (address, result[5])
    return result

def test_write(duck, dev_id, address, data):
    result = None
    try:
        result = duck.write(dev_id, address,[data])
    except Exception as ex:
        print 'Exception thrown while writing addres %d' % (address)

    if result:
        pass
        #print 'Response ' + str(result[5])
    return result

def test_blink(duck):
    for i in range(10):
        print duck.write(1, 6, [0])
        print duck.read(1, 0,8)[5:-2]
        time.sleep(0.05)
        duck.write(1, 6, [1])
        print duck.read(1, 0, 8)[5:-2]
        time.sleep(0.05)

def test_change_id(duck):
    for i in range(5):
        print duck.ping(1)
        #print "[0FF, 0FF, ID, LEN, ERR]"
        print duck.read(1, 0, 7)[5:-2]
        print duck.write(1,3,[2])
        time.sleep(0.5)
        #print duck.read(1, 0, 8)[:13]
        print ''
        print duck.read(2, 0, 7)[5:-2]
        print duck.write(2,3,[1])
        time.sleep(0.5)

def test_ping(duck):
    for i in range(5):
        print duck.ping(1)

def serialize_int32(data):
    return  [ord(a) for a in list(struct.pack('<l',int(data)))]

          
def main():
    duck = DuckietownSerial('/dev/ttyUSB0', baudrate = 115200)
    test_ping(duck)
            
if __name__ == '__main__':
    main()
