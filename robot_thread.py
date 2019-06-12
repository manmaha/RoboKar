# Robot Threading Routines
# Manish Mahajan
#23 Oct 2017

import RPi.GPIO as GPIO
import time
from sys import argv
import sensors
import motors
import signal
import sys
import threading
import numpy as np
from yaml import load, loader


Obstruction_Type = ['LEFT', 'STRAIGHT', 'RIGHT']
Obstruction_Tolerance = 20.00 # in cm
Backup_Time = 0.5 # in seconds
Backup_Speed =  15

# Robot is an object comprising of a list of sensors and actuators
# methods are actions that a robot is capable of

class robot(object):
  def __init__(self,motors, sensors):
    self.motors = motors
    self.sensors = sensors
    self.readings = [1.0]*len(sensors) #initialise sensor readings
    self.roaming = False # is the robot roaming or avoiding
    self.stopped = True # is the robot stopped
    self.readings = self.sense() # sensor values
    pass

  #roaming methods
  def set_roaming_on(self):
    self.roaming = True

  def set_roaming_off(self):
    self.roaming = False

  def toggle_roaming(self):
    self.roaming = not self.roaming

  #motion methods
  def stop(self):
    for m in self.motors:
      m.brake()
      m.stop()
      self.stopped = True

  #forward/backward motion
  def move_straight(self,speed,direction, runTime):
    self.stopped = False
    motors.timed_move(self.motors,speed,direction,runTime)
    if runTime != motors.FOREVER:
      self.stopped = True
    pass

  def move_back(self,speed,direction,runTime):
    self.stopped = False
    motors.timed_move(self.motors,speed,False,runTime)
    if runTime != motors.FOREVER:
      self.stopped = True
    pass

  #Sensor methods
  def sense(self):
    #populates readings with a  list of sensor readings,
    #in this case distance readings
    # of left, front and right sensor
    self.readings = [sensor.sense() for sensor in self.sensors]
    #print self.readings

  def sensor_readings(self):
    return self.readings

class RobotOne(robot): # simple 2 wheeled robot
# has two motors
# has three ultrasonic sensors
  def __init__(self,motors,sensors):
    super(RobotOne,self).__init__(motors,sensors) #do all the usual set up
    self.motors = motors
    self.sensors = sensors
    self.leftMotor=motors[0]
    self.rightMotor=motors[1]
    self.leftSensor = sensors[0]
    self.frontSensor = sensors[1]
    self.rightSensor = sensors[2]

    from yaml import load, Loader
        # intitalisse parameters
    self.params = load(open('params.yaml').read(), Loader=Loader)

    #Calculate Max Left and Right motor speeds
    U_l_max = self.params['MAXGAIN']/self.params['left_ratio']
    U_r_max = self.params['MAXGAIN']/self.params['right_ratio']
    self.lin_vel_max = self.params['R']/2*(U_l_max+U_r_max)
    self.ang_vel_max = self.params['R']/self.params['L']*(U_l_max-U_r_max)

    pass

#unicycle model
  def command_vel(self,(v,w),runTime):
# input is a linear velocity and angular velocity tuple
# moves the motors by left_gain and right_gain

  #constrain velocities
   lin_vel = np.sign(v)*min(self.lin_vel_max,abs(v))
   ang_vel = np.sign(w)*min(self.ang_vel_max,abs(w))

  #unicycle Model
   left_gain = self.params['left_ratio']*(lin_vel-ang_vel*self.params['L']/2)/self.params['R']
   left_direction = left_gain > 0
   right_gain = self.params['right_ratio']*(lin_vel+ang_vel*self.params['L']/2)/self.params['R']
   right_direction = right_gain > 0
  #move command
   self.leftMotor.move(abs(left_gain),left_direction)
   self.rightMotor.move(abs(right_gain),right_direction)

   if runTime != self.params['FOREVER']:
    start_time = time.time()
    end_time = start_time
    while end_time - start_time < runTime:
        end_time = time.time()
    self.stop()

   pass

  #motion methods
  def move_left(self,speed,direction,runTime):
  # move back if direction False and put time for short move
    self.stopped = False
    self.leftMotor.stop()
    self.rightMotor.forward(speed,direction)
    if runTime != motors.FOREVER:
      start_time = time.time()
      end_time = start_time
      while end_time - start_time < runTime:
        end_time = time.time()
      for m in self.motors : m.stop()
      self.stopped = True

  def avoid_right(self):
    self.move_left(Backup_Speed*0.5,True,Backup_Time*0.5)
    pass

  def move_right(self,speed, direction,runTime):
  # move back if direction False and put time for short move
    self.stopped = False
    self.rightMotor.stop()
    self.leftMotor.forward(speed,direction)
    if runTime != motors.FOREVER:
      start_time = time.time()
      end_time = start_time
      while end_time - start_time < runTime:
        end_time = time.time()
      for m in self.motors : m.stop()
      self.stopped = True

  def avoid_left(self):
    self.move_right(Backup_Speed*0.5,True,Backup_Time*0.5)
    pass

  def avoid_front(self):
  # avoid obstacle to the front
   self.move_back(Backup_Speed*3,True,Backup_Time*3)# go back far away
   self.stop()
   self.sense()
   if self.readings[0]>self.readings[2]: #right obstacle is closer
     self.avoid_right()
   else:
     self.avoid_left()
   pass


  def obstruction(self):
  # returns, False for no obstruction, 'LEFT','FORWARD','RIGHT' for left, forward and right
    self.sense()
    print 'sensor values are : ', self.readings
    if min(self.readings) > Obstruction_Tolerance:
      print ('no obstruction found')
      return False
    else:
      print ('obstruction type', Obstruction_Type[self.readings.index(min(self.readings))])
      return Obstruction_Type[self.readings.index(min(self.readings))]
    pass

  def back_up(self, obstruction_type):
  # back up if an obstruction of obstruction_type is found
    if obstruction_type: # not False
      self.stop()
      chooser = {
        'LEFT':self.avoid_left,
        'RIGHT':self.avoid_right,
        'STRAIGHT':self.avoid_front
        }
      chooser[obstruction_type]()
      self.back_up(self.obstruction())
      pass

  def roam(self):
  # robot roams and avoids obstacles
    self.back_up(self.obstruction())
    self.move_straight(self.params['MAXGAIN']*0.4,True,1.5)

class sense(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    signal.signal(signal.SIGINT, signal_handler)
    pass

  def run(self):
    global PiOde
    while True:
      if PiOde.roaming:
        print 'sensing'
        print 'sensor values are: ', PiOde.readings
        with lock:
          PiOde.sense()
        time.sleep(1)
      else:
        print 'not sensing'
    pass

class roam(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    pass

  def run(self):
    global PiOde
    while True:
     with lock:
      if PiOde.roaming:
        print 'roaming'
        obstacle = PiOde.obstruction()
        if not obstacle:
          for m in PiOde.motors:
            m.forward(speed=motors.self.params['MAXGAIN']*0.4)
        else:
          PiOde.back_up(obstacle)
      else:
        PiOde.stop()
        print 'stopped'


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
PiOde = RobotOne(Motors,Sensors)

#set up Ctrl C interrupt
signal.signal(signal.SIGINT, signal_handler)

#set up button handling
GPIO.add_event_detect(button_pin, GPIO.RISING, bouncetime = 200)
GPIO.add_event_callback(button_pin, button_handler)
time.sleep(1)
PiOde.sense()
print PiOde.sensor_readings()
lock = threading.RLock()

def main():
  s = sense()
  r = roam()
  time.sleep(1)
  s.start()
  r.start()

if __name__ == "__main__":
    main()
