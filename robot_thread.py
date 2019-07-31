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
from sys import argv


class sense(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    signal.signal(signal.SIGINT, signal_handler)
    pass

  def run(self):
    global PiOde
    while True:
      if PiOde.roaming:
        print ('sensing')
        print ('sensor values are: ', PiOde.readings)
        with lock:
          PiOde.sense()
        time.sleep(1)
      else:
        print ('not sensing')
    pass

class roam(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    pass

  def run(self):
    global PiOde
    while True:
     with lock:
      if PiOde.is_roaming():
        print ('roaming')
        PiOde.roam()
      else:
        PiOde.stop()
        print ('stopped')


def signal_handler(signum, frame):
        print('You pressed Ctrl+C!')
        GPIO.cleanup()
        sys.exit(0)

def button_handler(pin):
	global PiOde
	PiOde.toggle_roaming()


#test and run the the Robot
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

#button pins
button_pin = 18
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

#set up motors and sensors
Motors = [motors.motor(leftMotorPins,'Left'),motors.motor(rightMotorPins,'Right')]
Sensors = [sensors.ultrasonic_sensor([trig,echo_left]), sensors.ultrasonic_sensor([trig,echo_fwd]), sensors.ultrasonic_sensor([trig,echo_right])]

#set up robot
PiOde = robots.RobotOne(Motors,Sensors)

#set up Ctrl C interrupt
signal.signal(signal.SIGINT, signal_handler)

#set up button handling
GPIO.add_event_detect(button_pin, GPIO.RISING, bouncetime = 200)
GPIO.add_event_callback(button_pin, button_handler)
time.sleep(1)
PiOde.sense()
print ('sensor_readings: ',PiOde.sensor_readings())
lock = threading.RLock()

def main():
  s = sense()
  r = roam()
  time.sleep(1)
  s.start()
  r.start()

if __name__ == "__main__":
    main()
