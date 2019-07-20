# Robot Threading Routines
# Manish Mahajan
#23 Oct 2017

import RPi.GPIO as GPIO
import time
import sensors
import motors
import robots
import signal
import sys
import threading
import socket
from yaml import load, Loader
from sys import argv

#Class that drives the robot
class Driver(object):

#initialises robot and Driver
    def __init__(self,robot,HOST,PORT):
        self.robot = robot
        self.v = 0.0
        self.w = 0.0
        self.HOST = HOST
        self.PORT = PORT
        self._lock=threading.RLock()
        pass

#reads sensors
    def sense(self):
        while True:
            with self._lock:
                self.robot.sense()
            time.sleep(1)
            print ('sensor values are: ', self.robot.readings)
        pass

#Drives the robot with commanded velocities
    def drive(self):
        while True:
            with self._lock:
                vel=[self.v,self.w]
            self.robot.command_vel(vel)
            print('commanded velocities',vel)
            time.sleep(0.5)
        pass

#receives commands
    def rx_commands(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((self.HOST, self.PORT))
            s.listen()
            while True:
                conn, addr = s.accept()
                with conn:
                    print('Connected by', addr)
                    # generate v,w = data
                    while True:
                        try:
                            data = conn.recv(1024).decode()
                        except:
                            break
                        else:
                            if data:
                                try:
                                    v,w = [float(val) for val in data.split(',')]
                                except:
                                    print('couldnt form float')
                                else:
                                    with self._lock:
                                        self.v,self.w = v,w
                                    print('Received',v,w)
                            else:
                                break
        pass

def signal_handler(signum, frame):
        print('You pressed Ctrl+C!')
        GPIO.cleanup()
        sys.exit(0)

def main(HOST='',PORT=65432):
    #HOST is localhost
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
    PiOde.test()

    #Buttons
    button_pin = params['button_pin']
    GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    #set up button handling
    GPIO.add_event_detect(button_pin, GPIO.RISING, bouncetime = 200)
    GPIO.add_event_callback(button_pin, PiOde.toggle_roaming)

    d = Driver(PiOde,HOST,PORT)
    sense = threading.Thread(target=d.sense)
    rx_commands = threading.Thread(target=d.rx_commands)
    drive = threading.Thread(target=d.drive)

    sense.start()
    rx_commands.start()
    drive.start()

if __name__=="__main__":
    if len(argv) >3 :
        print('too many args')
    elif len(argv)==3:
        main(argv[1],int(argv[2]))
    elif len(argv)==2:
        main(argv[1])
    else:
        main()
