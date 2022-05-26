#!/usr/bin/env python3

from time import sleep

from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, SpeedPercent, MoveTank, SpeedRPM, MoveSteering, MediumMotor, OUTPUT_D
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.led import Leds
from ev3dev.core import RemoteControl

import os
import sys
import time

POS_MAX_MED_MOTOR = 205

def debug_print(*args, **kwargs):
    '''Print debug messages to stderr.

    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)


if __name__ == "__main__":
    drive = MoveSteering(OUTPUT_A, OUTPUT_B)
    #ts = TouchSensor()
    rc = RemoteControl()
    rc_motors = RemoteControl(channel=2)
    medium_motor = MediumMotor(OUTPUT_C)
    rc_large_motor = RemoteControl(channel=3)
    large_motor = LargeMotor(OUTPUT_D)

    # print something to the output panel in VS Code
    debug_print('Code uploaded!')

    print("Code uploaded!")

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
        elif rc_motors.red_up:
            if medium_motor.position >= -POS_MAX_MED_MOTOR:
                medium_motor.on_to_position(-20, -POS_MAX_MED_MOTOR)
                debug_print("State:", medium_motor.state)
                debug_print("Position:", medium_motor.position)
        elif rc_motors.red_down:
            if medium_motor.position <= 0:
                medium_motor.on_to_position(20, 0)
                debug_print("State:", medium_motor.state)
                debug_print("Position:", medium_motor.position)
        elif rc_motors.blue_up:
            if medium_motor.position >= -(POS_MAX_MED_MOTOR-10):
                medium_motor.on(-20)
                debug_print("State:", medium_motor.state)
                debug_print("Position:", medium_motor.position)
        elif rc_motors.blue_down:
            if medium_motor.position <= 0:
                medium_motor.on(20)
                debug_print("State:", medium_motor.state)
                debug_print("Position:", medium_motor.position)
        elif rc_large_motor.blue_up:
            #if medium_motor.position >= -210:
            large_motor.on(-10)
            debug_print("State:", large_motor.state)
            debug_print("Position:", large_motor.position)
        elif rc_large_motor.blue_down:
            #if medium_motor.position <= 0:
            large_motor.on(10)
            debug_print("State:", large_motor.state)
            debug_print("Position:", large_motor.position)
        else:
            drive.on(0, 0)

            # Force motor to hold
            large_motor.stop(stop_action=LargeMotor.STOP_ACTION_HOLD)
            medium_motor.stop(stop_action=MediumMotor.STOP_ACTION_HOLD)

            #large_motor.on(0)
            #medium_motor.on(0)
