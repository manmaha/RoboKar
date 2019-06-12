# Motor Driver Routines
# Manish Mahajan
# 1 Sep 2017
# TB6612FNG Motor Driver

import RPi.GPIO as GPIO
import time
from sys import argv

# Motor is an object, setup defined as list of gpiopins and motor_type (A or B)
# TB6612FNG Motor Driver controls two motors, Motor A and Motor B
class motor(object):
  def __init__(self,gpiopins, motor_type = True):
    #Get Parameters
    from yaml import load, Loader
    # intitalisse parameters
    self.params = load(open('params.yaml').read(), Loader=Loader)


    GPIO.setmode(GPIO.BCM)

    #gpiopins is a list of pins in this order:
    #PWM,In1,In2,StdBy
    #StdBy has to be pulled to High for motor to be working
    #All pins have to designated as Output
    #motor type - to be used later, 'LEFT', 'RIGHT'

    GPIO.setup(gpiopins,GPIO.OUT)
    self.PWM = GPIO.PWM(gpiopins[0],self.params['PWM_FREQ'])
    #self.PWM = gpiopins[0]
    self.In1 = gpiopins[1]
    self.In2 = gpiopins[2]
    self.stdby = gpiopins[3]
    self.motor_type=motor_type
    pass

  def standby(self, level = True):
  #puts the motor in stdby mode
    #GPIO.output(self.PWM,level)
    GPIO.output(self.stdby,level)
    pass

  def move(self, speed , direction = True): #True for forward, False for backwards
  #simple move command
    self.standby()
    self.PWM.start(min(speed,self.params['MAXGAIN']))
    print('speed: ',speed,'direction: ',direction)
    if direction :
        GPIO.output((self.In1,self.In2),(GPIO.HIGH,GPIO.LOW))
    else:
        GPIO.output((self.In1,self.In2),(GPIO.LOW,GPIO.HIGH))
    #GPIO.output((self.In1,self.In2),(direction,not direction))
    pass

  def forward(self,speed, direction=True):
  #moves the motor forward
    self.standby()
    self.PWM.start(speed) #duty cycle = speed
    print('moving forward - direction:',direction)
    GPIO.output((self.In1,self.In2),(direction,not direction))
    pass

  def back(self,speed):
  # moves the motor backwards
    self.standby()
    self.PWM.start(speed) #duty cycle = speed
    print ('moving back')
    GPIO.output((self.In1,self.In2),(GPIO.LOW,GPIO.HIGH))
    pass

  def brake(self):
  # brakes the motor
    self.PWM.stop
    GPIO.output([self.In1,self.In2], GPIO.HIGH)
    pass

  def stop(self):
  # stops the motor
    GPIO.output(self.In1,GPIO.LOW)
    GPIO.output(self.In2,GPIO.HIGH)
    self.PWM.start(self.params['MAX_DC']) #duty cycle Max
    self.standby(False)
    pass


def timed_move(motorList,speed,direction,runTime):
#move a list of motors for fixed time in seconds
  for m in motorList:
     m.move(speed,direction) # move motors forward
  start_time = time.time()
  end_time = start_time
  if runTime != m.params['FOREVER']:
       while end_time - start_time < runTime:
         end_time = time.time()
       for m in motorList : m.stop()
  pass

def main():

  GPIO.setmode(GPIO.BCM)
  StdByPin = 22  # this is the common pin
  leftMotorPins = [12,23,24] # fill up with GPIO pins, PWMA, AIn1, AIn2
  rightMotorPins = [13,25,5] # same as above
  leftMotorPins.append(StdByPin)
  rightMotorPins.append(StdByPin)
  l = motor(leftMotorPins)
  r = motor(rightMotorPins)
  speed = l.params['MAXGAIN']*0.5
  l.back(speed)
  time.sleep(3)
  print('left moved forward')
  l.stop()
  r.back(speed)
  time.sleep(1)
  r.stop()
  time.sleep(3)
  print('right moved forward')

  for m in (l,r): m.forward(speed)
  time.sleep(2)
  for m in (l,r): m.stop()
  GPIO.cleanup()


if __name__ == "__main__":
    main()
