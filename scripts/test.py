#!/usr/bin/python

import time
from duckietown_driver.serial_interface import DuckietownSerial
from duckietown_driver.message import DuckietownCommand, DuckietownStatus

def test_blink(duck):
    cmd = DuckietownCommand()
    status = DuckietownStatus()
    for i in range(5):
        cmd.led = False
        duck.send_command(cmd)
        duck.get_status(status)
        print status
        time.sleep(0.5)
        cmd.led = True
        duck.send_command(cmd)
        duck.get_status(status)
        print status
        time.sleep(0.5)
    
def main():
    duck = DuckietownSerial('/dev/ttyUSB0', baudrate = 115200)
    time.sleep(2.0) # for arduino bootloader
    test_blink(duck)
            
if __name__ == '__main__':
    main()
