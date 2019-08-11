import RPi.GPIO as GPIO
StdByPin = 14
GPIO.setmode(GPIO.BCM)
GPIO.setup(StdByPin,GPIO.OUT)
GPIO.output(StdByPin,False)