#!/usr/bin/env python3
# Manish Mahajan 23 July 2019
# RoboKar voice commander using Google CloudSpeechClient
# options --language --hostname RoboKar --port

"""Voice Commander for the RoboKar"""
import argparse
import locale
import logging
from yaml import load, Loader
import signal
import time
import socket
import random
import threading

from aiy.board import Board, Led
from aiy.cloudspeech import CloudSpeechClient
import aiy.voice.tts

path = '/home/pi/RoboKar'


class RoboKarVoiceCommander(object):
    def __init__(self,args):
        # get commands
        # intitalise commands
        self.commands = load(open(path+'/'+'commands.yaml').read(), Loader=Loader)

        #initialise voice server
        self.args = args
        logging.basicConfig(level=logging.DEBUG)
        logging.info('Initializing for language %s...', args.language)
        self.hints = tuple(self.commands.keys())
        self.client = CloudSpeechClient()

        # initialise transmitted commands
        self.v = 0.0
        self.w = 0.0

        #initialise lock
        self._lock=threading.RLock()
        pass

    def get_v(self):
        with self._lock:
            v = self.v
        return v

    def get_w(self):
        with self._lock:
            w = self.w
        return w

    def set_v(self,v):
        with self._lock:
            self.v =v
        logging.info('set v "%.2f"' % v)


    def set_w(self,w):
        with self._lock:
            self.w =w
        logging.info('set w "%.2f"' % w)

    def set_both_commands(self,cmd):
        # sets both v and c, cmd is a list of commands
        self.set_v(cmd[0])
        self.set_w(cmd[1])


#transmits the commands, this needs to be a threaded call with lock
    def tx_commands(self):
            old_v,old_w = self.get_v(),self.get_w()
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((self.args.hostname, self.args.port))
                logging.info('connected')
                while True:
                    v,w = self.get_v(), self.get_w()
                    #construct the data string and send it
                    #only send it if it has changed
                    if not (v == old_v and v == old_w):
                        data = '%.2f'%(v)+','+'%.2f'%(w)
                        s.sendall(data.encode())
                        old_v,old_w = v,w
                        logging.info('sent "%s" '% data)
                    time.sleep(1)



    def set_commands(self):

        with Board() as board:
            while True:
                if self.hints:
                    logging.info('Say something, e.g. %s.' % ', '.join(self.hints))
                else:
                    logging.info('Say something.')

                text = self.client.recognize(language_code=self.args.language,
                                    hint_phrases=self.hints)
                if text is None:
                    logging.info('You said nothing.')
                    continue

                logging.info('You said: "%s"' % text)
                text = text.lower()

                if 'goodbye' in text:
                    self.set_both_commands([0,0])
                    break
                if 'linear' in text:
                    # Remove "linear" from the text to be repeated
                    cmd = text.replace('linear', '').replace(' ','')
                    #print('You said, Linear:',cmd)
                    self.set_v(float(cmd))
                    aiy.voice.tts.say(cmd)
                elif 'angular' in text:
                    cmd = text.replace('angular', '').replace(' ','')
                    #print('You said, Angular:',cmd)
                    self.set_w(float(cmd))
                    aiy.voice.tts.say(cmd)
                elif text in self.commands: #search commands dict
                    #print('You said:',text)
                    self.set_both_commands(self.commands[text])
                    aiy.voice.tts.say(text)



def main():
    language, _ = locale.getdefaultlocale()
    parser = argparse.ArgumentParser(description='Assistant service example.')
    parser.add_argument('--language', default=language)
    parser.add_argument('--hostname', default='')
    parser.add_argument('--port', default=65432)
    args = parser.parse_args()



    c = RoboKarVoiceCommander(args)
    set_commands = threading.Thread(target=c.set_commands)
    tx_commands = threading.Thread(target=c.tx_commands)
    set_commands.start()
    tx_commands.start()



if __name__=="__main__":
        main()
