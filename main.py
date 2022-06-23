#!/usr/bin/env python3

from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, SpeedPercent, MoveTank, SpeedRPM, MoveSteering, MediumMotor, OUTPUT_D
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.led import Leds
from ev3dev.core import RemoteControl

import os
import sys
import time
import paho.mqtt.client as mqtt
import threading

POS_MAX_MED_MOTOR = 215

UP_DOWN_SPEED = 10

med_motor_pos = 0
up_down_motor_speed = 0
last_position = 0

def debug_print(*args, **kwargs):
    '''Print debug messages to stderr.

    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)


def publish_speed_up_down_motor(client):
    while True:
        global med_motor_pos
        med_motor_pos = medium_motor.position
        global up_down_motor_speed
        time.sleep(0.001)
        up_down_motor_speed = medium_motor.speed_sp
        global last_position

        if last_position != med_motor_pos:
            if up_down_motor_speed > 0:
                client.publish("/up_down_motor_pos", "down")
            elif up_down_motor_speed < 0:
                client.publish("/up_down_motor_pos", "up")
        else:
            client.publish("/up_down_motor_pos", "stop")

        last_position = med_motor_pos


def rc_control(drive, medium_motor, large_motor, rc, rc_motors, rc_large_motor):
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
            start_time = time.time()
            if med_motor_pos >= -POS_MAX_MED_MOTOR:
                medium_motor.on_to_position(-UP_DOWN_SPEED, -POS_MAX_MED_MOTOR)
                #debug_print("State:", medium_motor.state)
                debug_print("Position:", med_motor_pos)
                #debug_print("--- %s seconds ---" % (time.time() - start_time))
        elif rc_motors.red_down:
            if med_motor_pos <= 0:
                medium_motor.on_to_position(UP_DOWN_SPEED, 0)
                #debug_print("State:", medium_motor.state)
                debug_print("Position:", med_motor_pos)
        elif rc_motors.blue_up:
            if med_motor_pos >= -(POS_MAX_MED_MOTOR-7):
                medium_motor.on(-UP_DOWN_SPEED)
                #debug_print("State:", medium_motor.state)
                debug_print("Position:", med_motor_pos)
        elif rc_motors.blue_down:
            if med_motor_pos <= 0:
                medium_motor.on(UP_DOWN_SPEED)
                #debug_print("State:", medium_motor.state)
                debug_print("Position:", med_motor_pos)
        elif rc_large_motor.blue_up:
            if large_motor.position > -95:
                large_motor.on(-7)
                #debug_print("State:", large_motor.state)
                debug_print("Position:", large_motor.position)
        elif rc_large_motor.blue_down:
            if large_motor.position < 0:
                large_motor.on(7)
                #debug_print("State:", large_motor.state)
                debug_print("Position:", large_motor.position)
        else:
            drive.on(0, 0)

            # Force motor to hold
            large_motor.stop(stop_action=LargeMotor.STOP_ACTION_HOLD)
            medium_motor.stop(stop_action=MediumMotor.STOP_ACTION_HOLD)

            #large_motor.on(0)
            #medium_motor.on(0)


if __name__ == "__main__":

    client = mqtt.Client("EV3")
    # Connect MQTT broker
    try:
        client.connect("192.168.0.107", 1883)
    except:
        print("Couldn't connect to MQTT broker.")

    # Init motors
    drive = MoveSteering(OUTPUT_A, OUTPUT_B)
    medium_motor = MediumMotor(OUTPUT_C)
    large_motor = LargeMotor(OUTPUT_D)

    # Assign channel controls
    rc = RemoteControl() # channel 1 for driving
    rc_motors = RemoteControl(channel=2) # channel 2 for up and down motor
    rc_large_motor = RemoteControl(channel=3) # channel 3 for back motor

    medium_motor.position = 0
    #medium_motor.speed_sp = 0
    large_motor.position = 0

    # print something to the output panel in VS Code
    debug_print('Code uploaded!')

    # EV3 print
    print("Code uploaded!")

    t1 = threading.Thread(target=publish_speed_up_down_motor, args=(client, ))
    t1.start()

    t2 = threading.Thread(target=rc_control, args=(drive, medium_motor, large_motor, rc, rc_motors, rc_large_motor))
    t2.start()

