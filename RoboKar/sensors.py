# Sensor Driver Routines
# Manish Mahajan
# 1 Sep 2017


import RPi.GPIO as GPIO
import time
from sys import argv

# Sensor is an object, setup defined as list of gpiopins

class sensor(object):
  def __init__(self,gpiopins):
    GPIO.setmode(GPIO.BCM)
    pass
    
  def sense(self,mode=1):
  #senses and returns depending on mode
    pass
  
  def trigger(self,mode=1):
  # trigger the sensor to start sensing
    pass
    
#ultrasonic sensor HC-SR04
class ultrasonic_sensor(sensor):
  def __init__(self,gpiopins):
    super(ultrasonic_sensor,self).__init__(gpiopins) #do all the usual set up
    self.trig=gpiopins[0]
    self.echo=gpiopins[1]
    GPIO.setup(self.trig,GPIO.OUT)
    GPIO.setup(self.echo,GPIO.IN)
    pass

  def trigger(self,mode=1):
    GPIO.output(self.trig,GPIO.LOW)
    time.sleep(0.5) #wait for sensor to settle
    GPIO.output(self.trig,GPIO.HIGH)
    time.sleep(0.0001)
    GPIO.output(self.trig,GPIO.LOW)
    pass

  def sense(self):
    self.trigger() #trigger the sensor
    while GPIO.input(self.echo)==GPIO.LOW:
      pulse_start = time.time()
      pulse_end = pulse_start
      while not GPIO.input(self.echo):
        pulse_start = time.time()
        if pulse_start - pulse_end > 0.02: #Echo pin still low
          return 101
      while GPIO.input(self.echo):
        pulse_end = time.time()
        t = pulse_end - pulse_start
        if t > 0.02 : #pulse too long
          return 102
      return round(t*17150/2,2)
    pass

# IR Sensor MH Sensor Flying Fish
class ir_sensor(sensor):
  def __init__(self,gpiopins):
    super(ir_sensor,self).__init__(gpiopins) #do all the usual set up
    self.proximity = gpiopins[0]
    GPIO.setup(self.proximity,GPIO.IN)
    pass


  def sense(self):
    return not GPIO.input(self.proximity)
    pass



def main():
  
  GPIO.cleanup()


if __name__ == "__main__":
    main()
