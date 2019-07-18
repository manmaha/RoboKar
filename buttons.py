import RPi.GPIO as GPIO
import time


GPIO.setmode(GPIO.BCM)
button_pin = 4
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#button_pin is pulled down
#connect the other pin to 3.3v

def button_pressed(button_pin):
#returns (button state) for button connected to GND on button press
  #GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)  
  state = not GPIO.input(button_pin)
  time.sleep(0.2)
  return (state)
  
#def button_handler(pin):
#   roaming = not roaming
#   pass

def main():
  previous_state = False
  while True:
    button,previous_state = button_pressed(button_pin,previous_state)
    if button:
        print('Button Pressed')
        time.sleep(0.2)
        #put your code here)




if __name__ == "__main__":
    main()