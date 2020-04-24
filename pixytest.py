#!/usr/bin/env python3

# use blocks of color to indicate "obstacles"

from __future__ import print_function
import pixy 
from ctypes import *
from pixy import *

import time

from a_star import AStar
a_star = AStar()
path = []

print("Test PixyCam")

pixy.init ()
pixy.change_prog ("color_connected_components")

class Blocks (Structure):
  _fields_ = [ ("m_signature", c_uint),
    ("m_x", c_uint),
    ("m_y", c_uint),
    ("m_width", c_uint),
    ("m_height", c_uint),
    ("m_angle", c_uint),
    ("m_index", c_uint),
    ("m_age", c_uint) ]

blocks = BlockArray(100)
frame = 0

pixy.set_lamp(0, 0)
a_star.motors(70, 70) # move straight forward, need to use pi control!!!
print("begin obstacle search")

while 1:
    count = pixy.ccc_get_blocks (100, blocks) # 100 blocks probably overkill...

    if (count > 0):
        # make sure block is large enough to be relevant
        if (blocks[0].m_width > 50 and blocks[0].m_height > 50):
            a_star.motors(0, 0) # stops if a block is detected
            pixy.set_lamp(1, 1) # turns on lamp if a block is detected
            time.sleep(2)
            pixy.set_lamp(0, 0)
            print("obstacle detected, motor stopped")
        
