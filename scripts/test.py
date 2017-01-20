#!/usr/bin/python

import time
from duckietown_driver.serial_interface import DuckietownSerial
from duckietown_driver.message import DuckietownCommand, DuckietownStatus

def test_blink(duck):
    cmd = DuckietownCommand()
    status = DuckietownStatus()
    for i in range(20):
        cmd.led = not cmd.led
        cmd.pwm_ch1 = -200+i*20
        duck.send_command(cmd)
        print cmd.pwm_ch1
        # duck.get_status(status)
        # print status
        # time.sleep(0.1)
        # cmd.led = True
        # duck.send_command(cmd)
        # duck.get_status(status)
        # print status
        time.sleep(0.1)
    
def main():
    duck = DuckietownSerial('/dev/ttyUSB0', baudrate = 57600)
    time.sleep(2.0) # for arduino bootloader
    for i in xrange(10):
        print duck.ping(1)
        time.sleep(1.0) # for arduino bootloader
    test_blink(duck)
            
if __name__ == '__main__':
    main()
