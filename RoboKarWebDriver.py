#!/usr/bin/env python3
# Robot Set Up and Driver Routines
# For  Commands (v,W) recd through an a Web GUI
# Manish Mahajan
# 30 July 2019

import logging
import time
import argparse
import signal
import sys
import threading
from yaml import load, Loader
from sys import argv
from flask import Flask
from flask import request
import atexit




class Driver(object):

#initialises robot and Driver
    def __init__(self,robot,args):
        logging.basicConfig(level=logging.DEBUG)
        self.robot = robot
        self.args = args
        self._lock=threading.RLock()
        pass

    #@app.route("/")
    def web_interface(self):
        # html file is in RoboKar.html
        html = open("RoboKar.html")
        response = html.read().replace('\n', '')
        html.close()
        return response

    #@app.route("/read_vel")
    def read_vel(self):
        # Clean up v,w; convert to float; send cmd Velocity
        try:
            v = float(request.args.get("linear"))
            w = float(request.args.get("angular"))
        except:
            return "Bad Input"
            logging.info('Bad v,w input')
        else:
            print('commanded velocities',[v,w])
            if self.args.testing != 'True':
                self.robot.command_vel([v,w])
            return "Received v: "+str(v)+"w: "+str(w)


    #reads sensors
    def sense(self):
        while True:
            with self._lock:
                if self.args.testing != 'True':
                    self.robot.sense()
            time.sleep(1)
            if self.args.testing != 'True':
                print ('sensor values are: ', self.robot.readings)
            else:
                print('Sensing')
        pass

    def stop(self):
        print('stopping')
        if self.args.testing != 'True':
            self.robot.stop()
        return self.web_interface()



def main():
    parser = argparse.ArgumentParser(description='Web Driver for RoboKar')
    parser.add_argument('--hostname', default='0.0.0.0')
    parser.add_argument('--port', default=5000)
    parser.add_argument('--testing',default=False)
    args = parser.parse_args()
    # Cleanup done at exit

    @atexit.register
    def cleanup_robot():
        if args.testing != 'True':
            PiOde.stop()
            GPIO.cleanup()
        pass

    def signal_handler(signum, frame):
            print('You pressed Ctrl+C!')
            if args.testing != 'True':
                GPIO.cleanup()
            sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    if args.testing != 'True':
        # Rpi Sepcific Commands - if not testing
        import RPi.GPIO as GPIO
        import sensors
        import motors
        import robots
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
    else:
        PiOde = 1

    # driver instance
    d = Driver(PiOde,args)

    #start the threaded processes
    threading.Thread(target=d.sense).start()

    #start the flask server
    app = Flask(__name__)
    #app.add_url_rule('/','web_interface',d.web_interface)
    #app.add_url_rule('/read_vel','read_vel',d.read_vel)

    app.route("/")(d.web_interface)
    app.route("/read_vel")(d.read_vel)
    app.route("/stop_robot")(d.stop)
    app.route("/stop")(d.stop)

    app.run(host= args.hostname,port=args.port, debug=True)

if __name__=="__main__":
        main()
