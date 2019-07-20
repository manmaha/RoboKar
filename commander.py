# Manish Mahajan
# voice Commander for RoboKar
# Needs to run on Google AI Voice Kit
#!/usr/bin/env python3
from sys import argv
import socket
import random
import time
import threading

def get_v():
    #rturns v as a .2f string
    v_max = 5
    return(random.uniform(-v_max,v_max))

def get_w():
    #returns w as .2f string
    w_max = 3.14
    return(random.uniform(-w_max,w_max))

#class that generates commands for the robot and transmits them
class Commander(object):

#initiliases commander
    def __init__(self,HOST,PORT):
        self.v = 0.0
        self.w = 0.0
        self.HOST = HOST
        self.PORT = PORT
        self._lock=threading.Lock()
        pass

#generates v,w commands
    def get_commands(self):
        #this is where the voice module will get the commands
        return get_v(),get_w() #both are floats
        pass

#gets and sets the commands, this needs to be a threaded call with lock
    def set_commands(self):
        while True:
            v,w=self.get_commands()
            with self._lock:
                self.v = v
                self.w = w


#transmits the commands, this needs to be a threaded call with lock
    def tx_commands(self):
            old_v,old_w = 0,0
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((self.HOST, self.PORT))
                while True:
                    with self._lock:
                    #construct the data string and send it
                        if not (old_v == self.v and old_v == self.w):
                            data = '%.2f'%(self.v)+','+'%.2f'%(self.w)
                            s.sendall(data.encode())
                            old_v,old_w = self.v,self.w
                    #data = s.recv(1024)
                    print('sent ', data)
                    time.sleep(1)


def main(HOST='',PORT=65432):
    #HOST is robot hostname
    print(HOST,PORT)
    c = Commander(HOST,PORT)
    set_commands = threading.Thread(target=c.set_commands)
    tx_commands = threading.Thread(target=c.tx_commands)
    set_commands.start()
    tx_commands.start()



if __name__=="__main__":
    if len(argv) >3 :
        print('too many args')
    elif len(argv)==3:
        main(argv[1],int(argv[2]))
    elif len(argv)==2:
        main(argv[1])
    else:
        main()
