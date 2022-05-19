#!/usr/bin/env python3

from time import sleep

from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, MoveTank, SpeedRPM
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.led import Leds

import os
import sys
import time

m = LargeMotor(OUTPUT_A)
tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
ts = TouchSensor()

def debug_print(*args, **kwargs):
    '''Print debug messages to stderr.

    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)

# print something to the output panel in VS Code
debug_print('Code uploaded!')

print("You can see this!")

while True:
    if ts.is_pressed:
        #m.on_for_seconds(SpeedRPM(150), 5)
        tank_drive.on_for_rotations(SpeedPercent(75), SpeedPercent(75), 10)
        # drive in a different turn for 3 seconds
        #tank_drive.on_for_seconds(SpeedPercent(60), SpeedPercent(30), 3)
    #else:
     #   m.stop()
    # don't let this loop use 100% CPU
    sleep(0.01)
