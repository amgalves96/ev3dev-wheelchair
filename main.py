#!/usr/bin/env python3

from pydoc import cli

from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C,  OUTPUT_D, MoveSteering, MediumMotor
from ev3dev.core import RemoteControl # old version of ev3dev
from ev3dev2.sound import Sound

import os
import sys
import time
import paho.mqtt.client as mqtt
import threading

STEERING_LINEAR_SPEED = 20
STEERING_ANGULAR_SPEED = 9

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

left_wheel_pos = 0
right_wheel_pos = 0
right_wheel_speed = 0
left_wheel_speed = 0
last_left_wheel_pos = 0
last_right_wheel_pos = 0

top_down_topic = "/up_down_motor_pos"
back_topic = "/back_motor_pos"
steering_topic = "/steering"

def debug_print(*args, **kwargs):
    '''Print debug messages to stderr.

    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)


def publish_speed_up_down_motor(client, medium_motor):
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
                client.publish(top_down_topic, "down")
            elif up_down_motor_speed < 0:
                client.publish(top_down_topic, "up")
        else:
            client.publish(top_down_topic, "stop_up_down")

        up_down_last_position = up_down_motor_pos


def publish_speed_back_motor(client, large_motor):
    while True:
        global back_motor_pos
        back_motor_pos = large_motor.position
        global back_motor_speed
        time.sleep(0.001)
        back_motor_speed = large_motor.speed_sp
        global back_last_position

        if back_last_position != back_motor_pos:
            if back_motor_speed > 0:
                client.publish(back_topic, "back")
            elif back_motor_speed < 0:
                client.publish(back_topic, "forward")
        else:
            client.publish(back_topic, "stop_back")

        back_last_position = back_motor_pos


def publish_wheels_motors(client, left_wheel, right_wheel, drive):
    while True:
        global left_wheel_pos
        left_wheel_pos = left_wheel.position
        global right_wheel_pos
        right_wheel_pos = right_wheel.position
        global right_wheel_speed
        right_wheel_speed = right_wheel.speed_sp
        global left_wheel_speed
        left_wheel_speed = left_wheel.speed_sp
        global last_left_wheel_pos
        global last_right_wheel_pos

        time.sleep(0.001)

        if last_left_wheel_pos < left_wheel_pos and last_right_wheel_pos < right_wheel_pos:
            client.publish(steering_topic, "steer_front" + str(right_wheel_speed) + '.' + str(left_wheel_speed))
        elif last_left_wheel_pos > left_wheel_pos and last_right_wheel_pos > right_wheel_pos:
            client.publish(steering_topic, "steer_back"+ str(right_wheel_speed) + '.' + str(left_wheel_speed))
        elif last_left_wheel_pos < left_wheel_pos and last_right_wheel_pos > right_wheel_pos:
            client.publish(steering_topic, "steer_right"+ str(right_wheel_speed) + '.' + str(left_wheel_speed))
        elif last_left_wheel_pos > left_wheel_pos and last_right_wheel_pos < right_wheel_pos:
            client.publish(steering_topic, "steer_left"+ str(right_wheel_speed) + '.' + str(left_wheel_speed))
        else:
            client.publish(steering_topic, "steer_stop")

        last_left_wheel_pos = left_wheel_pos
        last_right_wheel_pos = right_wheel_pos


def rc_control(drive, medium_motor, large_motor, rc_driving, rc_up_down_motor, rc_back_motor):

    while True:

        # Channel #1
        if rc_driving.red_up:
            drive.on(0, STEERING_LINEAR_SPEED) # (steering, speed)
            debug_print("Posicao Roda Esquerda:", left_wheel_pos)
            debug_print("Posicao Roda Direita:", right_wheel_pos)
            debug_print("Velocidade Roda Esquerda:", left_wheel_speed)
            debug_print("Velocidade Roda Direita:", right_wheel_speed)
        elif rc_driving.red_down:
            drive.on(0, -STEERING_LINEAR_SPEED)
            debug_print("Posicao Roda Esquerda:", left_wheel_pos)
            debug_print("Posicao Roda Direita:", right_wheel_pos)
            debug_print("Velocidade Roda Esquerda:", left_wheel_speed)
            debug_print("Velocidade Roda Direita:", right_wheel_speed)
        elif rc_driving.blue_up:
            drive.on(-100, STEERING_ANGULAR_SPEED)
            debug_print("Posicao Roda Esquerda:", left_wheel_pos)
            debug_print("Posicao Roda Direita:", right_wheel_pos)
            debug_print("Velocidade Roda Esquerda:", left_wheel_speed)
            debug_print("Velocidade Roda Direita:", right_wheel_speed)
        elif rc_driving.blue_down:
            drive.on(100, STEERING_ANGULAR_SPEED)
            debug_print("Posicao Roda Esquerda:", left_wheel_pos)
            debug_print("Posicao Roda Direita:", right_wheel_pos)
            debug_print("Velocidade Roda Esquerda:", left_wheel_speed)
            debug_print("Velocidade Roda Direita:", right_wheel_speed)

        # Channel #2
        elif rc_up_down_motor.red_up:
            # start_time = time.time()
            if up_down_motor_pos >= -POS_MAX_UP_DOWN_MOTOR:
                medium_motor.on_to_position(-UP_DOWN_SPEED, -POS_MAX_UP_DOWN_MOTOR)
                debug_print("Estado do Motor do Assento:", medium_motor.state)
                debug_print("Posicao do Motor do Assento:", up_down_motor_pos)
                #debug_print("--- %s seconds ---" % (time.time() - start_time))
        elif rc_up_down_motor.red_down:
            if up_down_motor_pos <= 0:
                medium_motor.on_to_position(UP_DOWN_SPEED, 0)
                debug_print("Estado do Motor do Assento:", medium_motor.state)
                debug_print("Posicao do Motor do Assento:", up_down_motor_pos)
        elif rc_up_down_motor.blue_up:
            if up_down_motor_pos >= -(POS_MAX_UP_DOWN_MOTOR-7):
                medium_motor.on(-UP_DOWN_SPEED)
                debug_print("Estado do Motor do Assento:", medium_motor.state)
                debug_print("Posicao do Motor do Assento:", up_down_motor_pos)
        elif rc_up_down_motor.blue_down:
            if up_down_motor_pos <= 0:
                medium_motor.on(UP_DOWN_SPEED)
                debug_print("Estado do Motor do Assento:", medium_motor.state)
                debug_print("Posicao do Motor do Assento:", up_down_motor_pos)

        # Channel #3
        elif rc_back_motor.red_up:
            if back_motor_pos >= -POS_MAX_BACK_MOTOR:
                large_motor.on_to_position(-BACK_SPEED, -POS_MAX_BACK_MOTOR)
                debug_print("Estado do Motor das Costas:", large_motor.state)
                debug_print("Posicao do Motor das Costas:", back_motor_pos)
                #debug_print("--- %s seconds ---" % (time.time() - start_time))
        elif rc_back_motor.red_down:
            if back_motor_pos <= 0:
                large_motor.on_to_position(BACK_SPEED, 0)
                debug_print("Estado do Motor das Costas:", large_motor.state)
                debug_print("Posicao do Motor das Costas:", back_motor_pos)
        elif rc_back_motor.blue_up:
            if back_motor_pos > -POS_MAX_BACK_MOTOR:
                large_motor.on(-BACK_SPEED)
                debug_print("Estado do Motor das Costas:", large_motor.state)
                debug_print("Posicao do Motor das Costas:", back_motor_pos)
        elif rc_back_motor.blue_down:
            if back_motor_pos <= 0:
                large_motor.on(BACK_SPEED)
                debug_print("Estado do Motor das Costas:", large_motor.state)
                debug_print("Posicao do Motor das Costas:", back_motor_pos)

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

    # Initialize Components
    drive = MoveSteering(OUTPUT_A, OUTPUT_B)
    left_wheel = LargeMotor(OUTPUT_A)
    right_wheel = LargeMotor(OUTPUT_B)
    medium_motor = MediumMotor(OUTPUT_C)
    large_motor = LargeMotor(OUTPUT_D)
    sound = Sound()

    # Assign channel controls for remote control
    rc_driving = RemoteControl(channel=1) # channel 1 for driving
    rc_up_down_motor = RemoteControl(channel=2) # channel 2 for up and down motor
    rc_back_motor = RemoteControl(channel=3) # channel 3 for back motor

    # Initialize motor positions
    medium_motor.position = 0
    large_motor.position = 0
    large_motor.on_to_position(BACK_SPEED, -47) # sync with digital twin in ROS
    large_motor.position = -47 # make sure motor is in position
    debug_print("Posicao do Motor Costas:", large_motor.position)
    left_wheel.position = 0
    right_wheel.position = 0

    # broadcast message
    sound.speak('Code uploaded successfully!')

    # debug print in console
    # debug_print('Code uploaded!')

    # EV3 display print
    print("Code uploaded!")

    # Multithreading

    # up_down motor
    t1 = threading.Thread(target=publish_speed_up_down_motor, args=(client, medium_motor))
    t1.start()

    # remote control
    t2 = threading.Thread(target=rc_control, args=(drive, medium_motor, large_motor, rc_driving, rc_up_down_motor, rc_back_motor))
    t2.start()

    # back motor
    t3 = threading.Thread(target=publish_speed_back_motor, args=(client, large_motor))
    t3.start()

    # wheels motors
    t4 = threading.Thread(target=publish_wheels_motors, args=(client, left_wheel, right_wheel, drive))
    t4.start()
