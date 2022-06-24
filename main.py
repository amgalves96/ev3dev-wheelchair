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

POS_MAX_UP_DOWN_MOTOR = 215
UP_DOWN_SPEED = 10

POS_MAX_BACK_MOTOR = 95
BACK_SPEED = 5

up_down_motor_pos = 0
up_down_motor_speed = 0
up_down_last_position = 0

back_motor_pos = 0
back_motor_speed = 0
back_last_position = 0

top_down_topic = "/up_down_motor_pos"
back_topic = "/back_motor_pos"

def debug_print(*args, **kwargs):
    '''Print debug messages to stderr.

    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)


def publish_speed_up_down_motor(client):
    while True:
        global up_down_motor_pos
        up_down_motor_pos = medium_motor.position
        global up_down_motor_speed
        time.sleep(0.001)
        up_down_motor_speed = medium_motor.speed_sp
        global up_down_last_position
        #time.sleep(0.001)

        if up_down_last_position != up_down_motor_pos:
            if up_down_motor_speed > 0:
                client.publish(top_down_topic, "d")
            elif up_down_motor_speed < 0:
                client.publish(top_down_topic, "u")
        else:
            client.publish(top_down_topic, "s_ud")

        up_down_last_position = up_down_motor_pos


def publish_speed_back_motor(client):
    while True:
        global back_motor_pos
        back_motor_pos = large_motor.position
        global back_motor_speed
        time.sleep(0.001)
        back_motor_speed = large_motor.speed_sp
        global back_last_position

        if back_last_position != back_motor_pos:
            if back_motor_speed > 0:
                client.publish(back_topic, "b")
            elif back_motor_speed < 0:
                client.publish(back_topic, "f")
        else:
            client.publish(back_topic, "s_b")

        back_last_position = back_motor_pos


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
            if up_down_motor_pos >= -POS_MAX_UP_DOWN_MOTOR:
                medium_motor.on_to_position(-UP_DOWN_SPEED, -POS_MAX_UP_DOWN_MOTOR)
                #debug_print("State:", medium_motor.state)
                debug_print("Position:", up_down_motor_pos)
                #debug_print("--- %s seconds ---" % (time.time() - start_time))
        elif rc_motors.red_down:
            if up_down_motor_pos <= 0:
                medium_motor.on_to_position(UP_DOWN_SPEED, 0)
                #debug_print("State:", medium_motor.state)
                debug_print("Position:", up_down_motor_pos)
        elif rc_motors.blue_up:
            if up_down_motor_pos >= -(POS_MAX_UP_DOWN_MOTOR-7):
                medium_motor.on(-UP_DOWN_SPEED)
                #debug_print("State:", medium_motor.state)
                debug_print("Position:", up_down_motor_pos)
        elif rc_motors.blue_down:
            if up_down_motor_pos <= 0:
                medium_motor.on(UP_DOWN_SPEED)
                #debug_print("State:", medium_motor.state)
                debug_print("Position:", up_down_motor_pos)
        elif rc_large_motor.red_up:
            if back_motor_pos >= -POS_MAX_BACK_MOTOR:
                large_motor.on_to_position(-BACK_SPEED, -POS_MAX_BACK_MOTOR)
                #debug_print("State:", medium_motor.state)
                debug_print("Position:", back_motor_pos)
                #debug_print("--- %s seconds ---" % (time.time() - start_time))
        elif rc_large_motor.red_down:
            if back_motor_pos <= 0:
                large_motor.on_to_position(BACK_SPEED, 0)
                #debug_print("State:", medium_motor.state)
                debug_print("Position:", back_motor_pos)
        elif rc_large_motor.blue_up:
            if back_motor_pos > -POS_MAX_BACK_MOTOR:
                large_motor.on(-BACK_SPEED)
                #debug_print("State:", large_motor.state)
                debug_print("Position:", back_motor_pos)
        elif rc_large_motor.blue_down:
            if back_motor_pos <= 0:
                large_motor.on(BACK_SPEED)
                #debug_print("State:", large_motor.state)
                debug_print("Position:", back_motor_pos)
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
    large_motor.on_to_position(BACK_SPEED, -47)
    large_motor.position = -47

    # print something to the output panel in VS Code
    debug_print('Code uploaded!')

    # EV3 print
    print("Code uploaded!")

    t1 = threading.Thread(target=publish_speed_up_down_motor, args=(client, ))
    t1.start()

    t2 = threading.Thread(target=rc_control, args=(drive, medium_motor, large_motor, rc, rc_motors, rc_large_motor))
    t2.start()

    t3 = threading.Thread(target=publish_speed_back_motor, args=(client, ))
    t3.start()

