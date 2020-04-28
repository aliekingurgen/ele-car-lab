#!/usr/bin/env python3

# Copyright Pololu Corporation.  For more information, see https://www.pololu.com/
from flask import Flask, make_response, request
from flask import render_template
from flask import redirect
from subprocess import call
from planner import calculatePath
import time
import argparse
app = Flask(__name__)
app.debug = True

from a_star import AStar
a_star = AStar()
path = []

import json

import pixy 
from ctypes import *
from pixy import *


led0_state = False
led1_state = False
led2_state = False

currentLineCount = -1

# ---------------------------------------------------------------------------------

# block type for pixycam
class Blocks (Structure):
  _fields_ = [ ("m_signature", c_uint),
    ("m_x", c_uint),
    ("m_y", c_uint),
    ("m_width", c_uint),
    ("m_height", c_uint),
    ("m_angle", c_uint),
    ("m_index", c_uint),
    ("m_age", c_uint) ]

# ---------------------------------------------------------------------------------

# Get the configurations 
parser = argparse.ArgumentParser(description='201_indep_project')
parser.add_argument('--config_file', type=str)
arg_con = parser.parse_args()
config_file = arg_con.config_file
args = json.load(open(config_file))

rWheel = args['rWheel']
lWheel = args['lWheel']
turnLeftTime = args['turnLeftTime']
turnRightTime = args['turnRightTime']
convFactor = args['convFactor']

# ---------------------------------------------------------------------------------

@app.route("/", methods=['GET'])
@app.route("/index", methods=['GET'])
def hello():
    html = render_template("index.html")
    response = make_response(html)
    return response

# ---------------------------------------------------------------------------------

@app.route("/running", methods=['GET'])
def running():
    xlength = request.args.get("x")
    ylength = request.args.get("y")
    width = request.args.get("w")

    if xlength == str(-1) and ylength == str(-1) and width == str(-1):
        stop(30000)
        return "Stopped"

    if xlength == "" or ylength == "" or width == "":
        return "Please enter valid inputs."
    else:
        runPath(xlength, ylength, width)
        return "Success"
    
# ---------------------------------------------------------------------------------

@app.route("/drawfield", methods=['GET'])
def drawField():
    global currentLineCount

    xLength = request.args.get("x")
    yLength = request.args.get("y")
    width = request.args.get("w")

    if (xLength is None) or (xLength.strip() == ''): return ""
    if (yLength is None) or (yLength.strip() == ''): return ""
    if (width is None) or (width.strip() == ''): width = 0

    html = '<svg width=' + xLength + ' height=' + yLength + '> ' + '<g>' + \
            '<rect x=0 y=0 width=' + xLength + ' height=' + yLength + ' style="fill:white;stroke-width:3;stroke:black" />'
    
    if width != 0:
        lineX = 0
        lineCount = 0
        while (lineX <= int(xLength)):
            if lineCount < currentLineCount:
                html += '<line x1='+str(lineX)+' y1=15 x2='+str(lineX)+' y2=' + str(int(yLength) - 15) + ' stroke="green" stroke-width="4"></line>'
            elif lineCount == currentLineCount:
                html += '<line x1='+str(lineX)+' y1=15 x2='+str(lineX)+' y2=' + str(int(yLength) - 15) + ' stroke="orange" stroke-width="4"></line>'
            else:
                html += '<line x1='+str(lineX)+' y1=15 x2='+str(lineX)+' y2=' + str(int(yLength) - 15) + ' stroke="black" stroke-width="4"></line>'
            lineX += int(width)
            lineCount += 1

    html += 'Sorry, your browser does not support inline SVG.' + '<\g> ' + '</svg>'

    return html

# ---------------------------------------------------------------------------------

@app.route("/detectfield", methods=['GET'])
def detectField():

    x1_count = forwardForDetection()
    turnLeft()
    y1_count = forwardForDetection()
    turnLeft()
    x2_count = forwardForDetection()
    turnLeft()
    y2_count = forwardForDetection()

    #x = x1_time/convFactor
    #y = y1_time/convFactor
    # convert counts to cm (22cm/)
    x = x1_count/65.45
    y = y1_count/65.45
    
    return "X: " + str(x) + " "  + "Y: " + str(y)

# ---------------------------------------------------------------------------------

def piControl(error, accError, lr):
    if (lr == 0):
        kp = 0.9 # proportional constant
        ki = 0.05 # integral constant
    else:
        kp = 0.9 # proportional constant
        ki = 0.05 # integral constant

    pterm = kp*error
    iterm = ki*accError

    result = int(pterm + iterm)
    if (result < 0):
        result = 0

    return result

def forward(eCount):
    a_star.motors(50, 50)
    leftCount = 0 # use left count to determine when to stop
    rightCount = 0
    leftAcc = 0
    rightAcc = 0
    countAcc = 0
    startTime = time.perf_counter()
    encoders1 = a_star.read_encoders()
    print (encoders1)
    time2 = time.perf_counter()
    while (leftCount < eCount): # left encoder count is less than goal count
        if ((time.perf_counter() - time2) > 0.05): # look at encoder values every second
            encoders2 = a_star.read_encoders()
            # goal: 313 counts/second
            righten = encoders2[1] - encoders1[1]
            leften = encoders2[0] - encoders1[0]

            if (righten > 0):
                rightError = 35 - righten
                rightCount += righten
            if (leften > 0):
                leftError = 35 - leften
                leftCount += leften
            print (str(leftCount) + " " + str(rightCount))

            if (countAcc > 5):
                leftAcc += leftError
                rightAcc += rightError

            left = piControl(leftError, leftAcc, 0)
            right = piControl(rightError, rightAcc, 1)
            
            # angle correction and motor control
            if (leftCount - rightCount > 10):
                a_star.motors(left, right + 40)
                print ("adjusting right")
            if (rightCount - leftCount > 10):
                a_star.motors(left + 40, right)
                print ("adjusting left")
            else:
                a_star.motors(left, right)

            # reset time 2 /encoders
            time2 = time.perf_counter()
            encoders1 = encoders2
            countAcc += 1
    stop(1)

def forwardForDetection():
    #initialize camera and block array
    pixy.init ()
    pixy.change_prog ("color_connected_components")
    blocks = BlockArray(100)
    pixy.set_lamp(0, 0)

    #start pi control
    a_star.motors(70, 70)
    leftCount = 0 # use leftCount as goal encoder count
    rightCount = 0
    leftAcc = 0
    rightAcc = 0
    countAcc = 0
    startTime = time.perf_counter()
    encoders1 = a_star.read_encoders()
    print (encoders1)
    time2 = time.perf_counter()
    while 1:
        if ((time.perf_counter() - time2) > 0.05): # look at encoder values every second
            encoders2 = a_star.read_encoders()
            # goal: 313 counts/second
            righten = encoders2[1] - encoders1[1]
            leften = encoders2[0] - encoders1[0]

            if (righten > 0):
                rightError = 36 - righten
            if (leften > 0):
                leftError = 36 - leften

            if (countAcc > 5):
                leftAcc += leftError
                rightAcc += rightError

            # calculate new motor parameters and apply to motor
            left = piControl(leftError, leftAcc, 0)
            right = piControl(rightError, rightAcc, 1)

            # angle correction and motor control
            if (leftCount - rightCount > 10):
                a_star.motors(left, right + 40)
            if (rightCount - leftCount > 10):
                a_star.motors(left + 40, right)
            else:
                a_star.motors(left, right)

            # reset time 2 /encoders
            time2 = time.perf_counter()
            encoders1 = encoders2
            countAcc += 1

            # look at the camera input
            count = pixy.ccc_get_blocks (100, blocks)
            if (count > 0):
                # make sure block is large enough to be relevant
                if (blocks[0].m_width > 50 and blocks[0].m_height > 50):
                    a_star.motors(0, 0) # stops if a block is detected
                    pixy.set_lamp(1, 1) # turns on lamp if a block is detected
                    pixy.set_lamp(0, 0)
                    print("detected a corner")
                    break
    endTime = time.perf_counter()
    print(str(endTime))
    stop(2)
    return leftCount


def stop(t):
    a_star.motors(0, 0)
    time.sleep(t)

def forwardOld(t):
    a_star.motors(lWheel, rWheel)
    time.sleep(t)
    stop(1)

def backward(t):
    a_star.motors(-lWheel, -rWheel)
    time.sleep(t)
    stop(1)

def turnLeft():
    rightmotorInitial = a_star.read_encoders()[1]
    print(rightmotorInitial)
    a_star.motors(0, 60)
    go = True

    while go:
        rightmotor = a_star.read_encoders()[1]
        go = (abs(rightmotor - rightmotorInitial) < 1460)
    stop(1)

def turnRight():
    leftmotorInitial = a_star.read_encoders()[0]
    print(leftmotorInitial)
    a_star.motors(60, 0)
    go = True

    while go:
        leftmotor = a_star.read_encoders()[0]
        go = (abs(leftmotor - leftmotorInitial) < 1460)
    stop(1)

def runPath (x, y, w): # arguments passed are encoder counts
    global currentLineCount
    countPlant = int(int(y)*65.45) # CHANGED from timePlant, now total counts
    maxCount = int(x)/int(w) # number of planting lines, can still be in cm
    widthCount = int(int(w)*65.45)
    # (IF WIDTH < SOME VALUE NEED TO THROW EXCEPTION)
    if maxCount % 10 == 0:
        maxCount += 1
    leftOrRight = False # False: initially turn left, True: initially turn right

    currentLineCount = 0
    count = 0
    while count <= maxCount:
        forward(countPlant)
        # width is length of romi (15cm*65.45) + a count num
        if leftOrRight:
            turnRight()
            forward(widthCount - int(15*65.45))
            turnRight()
            leftOrRight = False
        else:
            turnLeft()
            forward(widthCount - int(15*65.45))
            turnLeft()
            leftOrRight = True
        count += 1
        currentLineCount += 1

    currentLineCount = -1

# ---------------------------------------------------------------------------------

@app.route("/status.json")
def status():
    
    buttons = a_star.read_buttons()
    analog = a_star.read_analog()
    battery_millivolts = a_star.read_battery_millivolts()
    encoders = a_star.read_encoders()
    data = {
        "buttons": buttons,
        "battery_millivolts": battery_millivolts,
        "analog": analog,
        "encoders": encoders
    }
    return json.dumps(data)

@app.route("/motors/<left>,<right>")
def motors(left, right):
    a_star.motors(int(left), int(right))
    return ""

@app.route("/leds/<int:led0>,<int:led1>,<int:led2>")
def leds(led0, led1, led2):
    a_star.leds(led0, led1, led2)
    global led0_state
    global led1_state
    global led2_state
    led0_state = led0
    led1_state = led1
    led2_state = led2
    return ""


@app.route("/heartbeat/<int:state>")
def hearbeat(state):
    if state == 0:
        a_star.leds(led0_state, led1_state, led2_state)
    else:
        a_star.leds(not led0_state, not led1_state, not led2_state)
    return ""


@app.route("/play_notes/<notes>")
def play_notes(notes):
    a_star.play_notes(notes)
    return ""

@app.route("/halt")
def halt():
    call(["bash", "-c", "(sleep 2; sudo halt)&"])
    return redirect("/shutting-down")

@app.route("/shutting-down")
def shutting_down():
    return "Shutting down in 2 seconds! You can remove power when the green LED stops flashing."

if __name__ == "__main__":
    app.run(host = "0.0.0.0")
