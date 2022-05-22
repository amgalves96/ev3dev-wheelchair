#!/usr/bin/env python3

from time import sleep

from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, MoveTank, SpeedRPM, MoveSteering
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.led import Leds
from ev3dev.core import RemoteControl

import os
import sys
import time

#test

#m = LargeMotor(OUTPUT_A)
drive = MoveSteering(OUTPUT_A, OUTPUT_B)
#ts = TouchSensor()
rc = RemoteControl()

def debug_print(*args, **kwargs):
    '''Print debug messages to stderr.

    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)

# print something to the output panel in VS Code
debug_print('Code uploaded!')

print("You can see this!")

while True:
    if rc.red_up:
        #m.on_for_seconds(SpeedRPM(150), 5)
        drive.on(0, 20)
        # drive in a different turn for 3 seconds
        #tank_drive.on_for_seconds(SpeedPercent(60), SpeedPercent(30), 3)
    elif rc.red_down:
        drive.on(0, -20)
    elif rc.blue_up:
        drive.on(-100, 20)
    elif rc.blue_down:
        drive.on(100, 20)
    else:
        drive.on(0, 0)
    # don't let this loop use 100% CPU
    #sleep(0.01)
