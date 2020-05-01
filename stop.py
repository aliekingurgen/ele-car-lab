from a_star import AStar
a_star = AStar()
path = []

import time

import pixy 
from ctypes import *
from pixy import *

def stop(t):
    pixy.init()
    pixy.set_lamp(0, 0)
    a_star.motors(0, 0)
    time.sleep(t)

if __name__ == "__main__":
    stop(10)