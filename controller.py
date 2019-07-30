
from yaml import load, Loader
from sys import argv
import sys
import time
#sys.path.append('/usr/local/lib/python2.7/dist-packages')
from approxeng.input.selectbinder import ControllerResource
from approxeng.input.dualshock3 import DualShock3

class PS3Controller(object):

    def __init__(self):
        self.params = load(open('params.yaml').read(), Loader=Loader)
        pass

    def get_values(self):
        """Get Linear and Angular Velocity"""
        try:
            with ControllerResource() as joystick:
                print('Found a joystick and connected')
                while joystick.connected:
                    # Get a corrected value for the left stick x-axis
                    w,v = joystick['lx'], joystick['ly']
                    print(v,w)
                    print(joystick.lx,joystick.ly)
                    linear_vel = v*self.params['linear_vel_controller_calib']
                    angular_vel = w*self.params['angular_vel_controller_calib']
                    return linear_vel,angular_vel
                else:
                    # Joystick disconnected...
                    print('Connection to joystick lost')
        except IOError:
                # No joystick found, wait for a bit before trying again
            print('Unable to find any joysticks')
            time.sleep(1.0)
        return 0,0



if __name__ == "__main__":
    ps3 = PS3Controller()
    while True:
        v,w = ps3.get_values()
        print (v, w)
