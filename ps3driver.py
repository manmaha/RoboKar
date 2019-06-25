import sys
import time
#sys.path.append('/usr/local/lib/python2.7/dist-packages')
from approxeng.input.selectbinder import ControllerResource
from approxeng.input.dualshock3 import DualShock3
import RPi.GPIO as GPIO
import robots
import sensors
import motors
import numpy as np
from yaml import load, Loader
import signal
from sys import argv
import robots


def main():
    #test and drive the the Robot with PS3 Controller
    #motor GPIO Pins
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    StdByPin = 22  # this is the common pin
    leftMotorPins = [12,23,24] # fill up with GPIO pins, PWMA, AIn1, AIn2
    rightMotorPins = [13,25,5] # same as above
    leftMotorPins.append(StdByPin)
    rightMotorPins.append(StdByPin)

    #Sensor GPIO Pins
    trig = 19    # common trig pin
    echo_left = 17 #left sensor echo pin
    echo_fwd = 4 #forward sensor echo pin
    echo_right = 16   #right sensor echo pin


    #set up motors and sensors
    Motors = [motors.motor(leftMotorPins,'Left'),motors.motor(rightMotorPins,'Right')]
    Sensors = [sensors.ultrasonic_sensor([trig,echo_left]), sensors.ultrasonic_sensor([trig,echo_fwd]), sensors.ultrasonic_sensor([trig,echo_right])]

    #set up robot
    PiOde = robots.RobotOne(Motors,Sensors)

    #set up Ctrl C interrupt
    signal.signal(signal.SIGINT, robots.signal_handler)

    #button pins
    button_pin = 18
    GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    #set up button handling
    #GPIO.add_event_detect(button_pin, GPIO.RISING,callback=lambda x: PiOde.toggle_roaming(), bouncetime = 200)
    #GPIO.add_event_callback(button_pin, button_handler)

    PiOde.ps3drive()



if __name__ == "__main__":
    main()


    # .... do stuff, the controller is bound to 'joystick' which is a DualShock3 instance....
