#!/usr/bin/env python3
# Robot Set Up and Driver Routines
# For  Commands (v,W) recd through a PS3 Controller
# Manish Mahajan
# June 2019

import sys
import time
import RPi.GPIO as GPIO
import robots
import sensors
import motors
import numpy as np
from yaml import load, Loader
import signal
from sys import argv
import socket
import random
import threading
from approxeng.input.selectbinder import ControllerResource
from approxeng.input.dualshock3 import DualShock3


def ps3drive(robot):
    while True:
            try:
                with ControllerResource() as joystick:
                    print('Found a joystick and connected')
                    while joystick.connected:
                        # Get a corrected value for the left stick x-axis
                        # This does not really work! PS3 contraller gives erratic values - not sure of the range
                        print('linear : ', joystick.ly, ' angular ' , joystick.lx)
                        w = (2*joystick.lx +1)*robot.params['max_omega']
                        v = (2*joystick.ly/100 -1)*robot.params['max_vel']
                        #robot.command_vel([v,w])
                        print (v,w)
                        time.sleep(1)
                # Joystick disconnected...
                print('Connection to joystick lost')
            except IOError:
                # No joystick found, wait for a bit before trying again
                print('Unable to find any joysticks')
                time.sleep(1.0)

def signal_handler(signum, frame):
        print('You pressed Ctrl+C!')
        GPIO.cleanup()
        sys.exit(0)


def main():
    #test and drive the the Robot with PS3 Controller
    #set up Ctrl C interrupt
    signal.signal(signal.SIGINT, signal_handler)

    #Initialise the Board
    params = load(open('params.yaml').read(), Loader=Loader)
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    StdByPin = params['StdByPin']  # this is the common pin
    leftMotorPins = params['leftMotorPins'] # fill up with GPIO pins, PWMA, AIn1, AIn2
    rightMotorPins = params['rightMotorPins'] # same as above
    leftMotorPins.append(StdByPin)
    rightMotorPins.append(StdByPin)
    #set up motors and sensors
    Motors = [motors.motor(leftMotorPins,'Left'),motors.motor(rightMotorPins,'Right')]
    Sensors = [sensors.ultrasonic_sensor([params['trig'],params['echo_left']]), sensors.ultrasonic_sensor([params['trig'],params['echo_fwd']]), sensors.ultrasonic_sensor([params['trig'],params['echo_right']])]

    #set up robot
    PiOde = robots.RobotOne(Motors,Sensors)
    #PiOde.test()

    #Buttons
    button_pin = params['button_pin']
    GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    #set up button handling
    #GPIO.add_event_detect(button_pin, GPIO.RISING,callback=lambda x: PiOde.toggle_roaming(), bouncetime = 200)
    #GPIO.add_event_callback(button_pin, button_handler)

    ps3drive(PiOde)

if __name__ == "__main__":
    main()


    # .... do stuff, the controller is bound to 'joystick' which is a DualShock3 instance....
