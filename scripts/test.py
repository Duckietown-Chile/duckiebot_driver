#!/usr/bin/python

import time
from duckietown_driver.serial_interface import DuckietownSerial
from duckietown_driver.message import DuckietownCommand, DuckietownStatus

def test_blink(duck):
    cmd = DuckietownCommand()
    status = DuckietownStatus()
    for i in range(20):
        cmd.led = False
        cmd.pwm_ch2 = -200+i*20
        duck.send_command(cmd)
        duck.get_status(status)
        print status
        time.sleep(0.1)
        cmd.led = True
        duck.send_command(cmd)
        duck.get_status(status)
        print status
        time.sleep(0.1)
    
def main():
<<<<<<< HEAD
    duck = DuckietownSerial('/dev/ttyAMA0', baudrate = 57600)
    for i in xrange(10):
        print duck.ping(1)
        time.sleep(1.0) # for arduino bootloader
    test_blink(duck)
            
if __name__ == '__main__':
    main()
