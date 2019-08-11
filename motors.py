# Motor Driver Routines
# Manish Mahajan
# 1 Sep 2017
# TB6612FNG Motor Driver

import RPi.GPIO as GPIO
import time
import argparse
import atexit

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

  def move(self, gain , direction = True): #True for forward, False for backwards
  #simple move command
    self.standby()
    self.PWM.start(min(gain,self.params['MAXGAIN']))
    #print('gain: ',gain,'direction: ',direction)
    if direction :
        GPIO.output((self.In1,self.In2),(GPIO.HIGH,GPIO.LOW))
    else:
        GPIO.output((self.In1,self.In2),(GPIO.LOW,GPIO.HIGH))
    #GPIO.output((self.In1,self.In2),(direction,not direction))
    pass

  def forward(self,gain, direction=True):
  #moves the motor forward
    self.standby()
    self.PWM.start(gain) #duty cycle = gain
    print('moving forward - direction:',direction,' gain: ', gain)
    GPIO.output((self.In1,self.In2),(direction,not direction))
    pass

  def back(self,gain):
  # moves the motor backwards
    self.standby()
    self.PWM.start(gain) #duty cycle = gain
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

    #GPIO.output(self.In1,GPIO.LOW)
    #GPIO.output(self.In2,GPIO.HIGH)
    #self.PWM.start(self.params['MAXGAIN']) #duty cycle Max
    #self.standby(False)
    self.PWM.stop
    GPIO.output([self.In1,self.In2],GPIO.HIGH)
    pass


def timed_move(motorList,gain,direction,runTime):
#move a list of motors for fixed time in seconds
  for m in motorList:
     m.move(gain,direction) # move motors forward
  start_time = time.time()
  end_time = start_time
  if runTime != m.params['FOREVER']:
       while end_time - start_time < runTime:
         end_time = time.time()
       for m in motorList : m.stop()
  pass

def main():
      parser = argparse.ArgumentParser(description='Motor Driver for RoboKar')
      parser.add_argument('--motor', default='both')
      parser.add_argument('--gain', default=100)
      parser.add_argument('--testing',default=False)
      parser.add_argument('--time', default=10)
      args = parser.parse_args()

      # Cleanup done at exit
#      @atexit.register
#      def cleanup_robot():
#          if args.testing != 'True':
#              print('EXITING')
#              for m in (l,r): m.stop()
#              GPIO.cleanup()
#              pass

      StdByPin = 22  # this is the common pin
      leftMotorPins = [12,23,24] # fill up with GPIO pins, PWMA, AIn1, AIn2
      rightMotorPins = [13,25,5] # same as above
      leftMotorPins.append(StdByPin)
      rightMotorPins.append(StdByPin)
      l = motor(leftMotorPins)
      r = motor(rightMotorPins)
      if args.motor == 'both':
          for m in (l,r):m.forward(float(args.gain))
      else:
          if args.motor == 'left':
              l.forward(float(args.gain))
          else:
              r.forward(float(args.gain))

      time.sleep(float(args.time))
      for m in (lr,r): m.brake()
      GPIO.cleanup()	




if __name__ == "__main__":
    main()
