#!/usr/bin/env python3

from flask import Flask, make_response, request
from flask import render_template
from flask import redirect
from subprocess import call
from planner import calculatePath
import time
import argparse
app = Flask(__name__)
app.debug = True

# for I2C control board commands
from a_star import AStar
a_star = AStar()
path = []

import json

# for Pixy2 camera
import pixy 
from ctypes import *
from pixy import *


led0_state = False
led1_state = False
led2_state = False

# initial parameters before program runs
currentLineCount = -1
yLength = 0
goingDown = True
obstacles = []
distAlongY = 0

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
countToCM = args['countToCM'] # only configuration used in final version of project

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
    global obstacles
    global distAlongY
    global goingDown

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

        # Display planting lines
        while (lineX <= int(xLength)):
            if lineCount < currentLineCount:
                html += '<line x1='+str(lineX)+' y1=15 x2='+str(lineX)+' y2=' + str(int(yLength) - 15) + ' stroke="green" stroke-width="4"></line>'
            
            elif lineCount == currentLineCount:
                if goingDown:
                    if distAlongY < 15:
                        html += '<line x1='+str(lineX)+' y1=15 x2='+str(lineX)+' y2=' + str(int(yLength) - 15) + ' stroke="orange" stroke-width="4"></line>'
                    elif distAlongY >  (int(yLength) - 15):
                        html += '<line x1='+str(lineX)+' y1=15 x2='+str(lineX)+' y2=' + str(int(yLength) - 15) + ' stroke="green" stroke-width="4"></line>'
                    else:
                        html += '<line x1='+str(lineX)+' y1=15 x2='+str(lineX)+' y2='+str(distAlongY)+' stroke="green" stroke-width="4"></line>'
                        html += '<line x1='+str(lineX)+' y1='+str(distAlongY)+' x2='+str(lineX)+' y2=' + str(int(yLength) - 15) + ' stroke="orange" stroke-width="4"></line>'
                if not goingDown:
                    if distAlongY < 15:
                        html += '<line x1='+str(lineX)+' y1=15 x2='+str(lineX)+' y2=' + str(int(yLength) - 15) + ' stroke="green" stroke-width="4"></line>'
                    elif distAlongY >  (int(yLength) - 15):
                        html += '<line x1='+str(lineX)+' y1=15 x2='+str(lineX)+' y2=' + str(int(yLength) - 15) + ' stroke="orange" stroke-width="4"></line>'
                    else:
                        html += '<line x1='+str(lineX)+' y1=15 x2='+str(lineX)+' y2='+str(distAlongY)+' stroke="orange" stroke-width="4"></line>'
                        html += '<line x1='+str(lineX)+' y1='+str(distAlongY)+' x2='+str(lineX)+' y2=' + str(int(yLength) - 15) + ' stroke="green" stroke-width="4"></line>'
             
            else:
                html += '<line x1='+str(lineX)+' y1=15 x2='+str(lineX)+' y2=' + str(int(yLength) - 15) + ' stroke="black" stroke-width="4"></line>'
            lineX += int(width)
            lineCount += 1
        
        # Display obstacles (if any)
        for obstacle in obstacles:
            obsX = str(int(obstacle[0])*int(width))
            obsY = str(obstacle[1])
            html += '<circle cx='+obsX+' cy='+obsY+' r=10 stroke="black" stroke-width=2 fill="red" />'

    html += 'Sorry, your browser does not support inline SVG.' + '<\g> ' + '</svg>'

    return html

# ---------------------------------------------------------------------------------

@app.route("/detectfield", methods=['GET'])
def detectField():

    #initialize camera
    pixy.init()
    pixy.set_lamp(1, 1)
    pixy.change_prog ("color_connected_components")

    # field detection instructions
    x1_count = forwardForDetection()
    turnLeft()
    y1_count = forwardForDetection()
    turnLeft()
    forwardObstacle(x1_count)
    turnLeft()
    forwardObstacle(y1_count)
    turnLeft()
    pixy.set_lamp(0, 0)

    y = int(x1_count/countToCM) # length in cm
    x = int(y1_count/countToCM) # width in cm
    
    return str(x) + " " + str(y)

# ---------------------------------------------------------------------------------

# PI speed control
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

# forward method for runPath
def forward(eCount):
    global distAlongY
    global yLength
    global distAlongY

    blocks = BlockArray(100)

    a_star.motors(50, 50)
    leftCount = 0 # use left count to determine when to stop
    rightCount = 0
    leftAcc = 0
    rightAcc = 0
    countAcc = 0
    encoders1 = a_star.read_encoders()
    time2 = time.perf_counter()
    while (leftCount < eCount): # left encoder count is less than goal count
        if ((time.perf_counter() - time2) > 0.05): # look at encoder values every 0.05 second

            encoders2 = a_star.read_encoders()
            righten = encoders2[1] - encoders1[1]
            leften = encoders2[0] - encoders1[0]

            # only update if difference is positive (not wraparound case)
            if (righten > 0):
                rightError = 35 - righten
                rightCount += righten
            if (leften > 0):
                leftError = 35 - leften
                leftCount += leften
                # Update distAlongY
                distAlongY = int(leftCount/countToCM)
                if not goingDown:
                    distAlongY = int(yLength) - int(distAlongY)

            # only apply integral control on fifth iteration
            if (countAcc > 5):
                leftAcc += leftError
                rightAcc += rightError

            # get motor parameters from PI method
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


            # look at the camera input
            count = pixy.ccc_get_blocks (100, blocks)
            if (count > 0):
                sig = int(blocks[0].m_signature)
                if (sig == 2):
                    print(sig)
            # make sure block is large enough to be relevant and signature is red (1)
                else:
                    if (blocks[0].m_width > 50 and blocks[0].m_height > 50):
                        a_star.motors(0, 0) # stops if a block is detected
                        encInitL, encInitR = a_star.read_encoders()
                        print("detected an obstacle")
                        print(blocks[0].m_signature)
                        avoidObstacle()
                        encFinL, encFinR = a_star.read_encoders()

                        # adjust right and left counts to account for extra distance
                        leftCount -= (encFinL - encInitL)
                        rightCount -= (encFinR - encInitR)
                        leftCount += int(35*countToCM)
                        rightCount += int(35*countToCM)

            # reset time 2 /encoders
            time2 = time.perf_counter()
            encoders1 = encoders2
            countAcc += 1
    stop(1)

# forward method for x edges (no obstacle detection)
def forwardOnX(eCount):
    global distAlongY
    global yLength
    global distAlongY

    blocks = BlockArray(100)

    a_star.motors(50, 50)
    leftCount = 0 # use left count to determine when to stop
    rightCount = 0
    leftAcc = 0
    rightAcc = 0
    countAcc = 0
    encoders1 = a_star.read_encoders()
    time2 = time.perf_counter()
    while (leftCount < eCount): # left encoder count is less than goal count
        if ((time.perf_counter() - time2) > 0.05): # look at encoder values every second

            encoders2 = a_star.read_encoders()
            righten = encoders2[1] - encoders1[1]
            leften = encoders2[0] - encoders1[0]

            if (righten > 0):
                rightError = 35 - righten
                rightCount += righten
            if (leften > 0):
                leftError = 35 - leften
                leftCount += leften

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

# subroutine for avoiding obstacles once detected
def avoidObstacle():
    global currentLineCount
    global obstacles
    global distAlongY
    
    obstacles.append([currentLineCount, distAlongY])

    stop(1)
    turnRight()
    forwardObstacle(int(5*countToCM))
    turnLeft()
    forwardObstacle(int(10*countToCM))
    turnLeft()
    forwardObstacle(int(5*countToCM))
    turnRight()
    a_star.motors(50, 50)
    print("finished avoiding obstacle")

# forward method for obstacle detection, stops looking for additional obstacles
def forwardObstacle(eCount):
    a_star.motors(50, 50)
    leftCount = 0 # use left count to determine when to stop
    rightCount = 0
    leftAcc = 0
    rightAcc = 0
    countAcc = 0
    encoders1 = a_star.read_encoders()
    time2 = time.perf_counter()
    while (leftCount < eCount): # left encoder count is less than goal count
        if ((time.perf_counter() - time2) > 0.05): # look at encoder values every 0.05 second

            encoders2 = a_star.read_encoders()
            righten = encoders2[1] - encoders1[1]
            leften = encoders2[0] - encoders1[0]

            if (righten > 0):
                rightError = 35 - righten
                rightCount += righten
            if (leften > 0):
                leftError = 35 - leften
                leftCount += leften

            if (countAcc > 5):
                leftAcc += leftError
                rightAcc += rightError

            left = piControl(leftError, leftAcc, 0)
            right = piControl(rightError, rightAcc, 1)
            
            # angle correction and motor control
            if (leftCount - rightCount > 10):
                a_star.motors(left, right + 40)
                #print ("adjusting right")
            if (rightCount - leftCount > 10):
                a_star.motors(left + 40, right)
                #print ("adjusting left")
            else:
                a_star.motors(left, right)

            # reset time 2 /encoders
            time2 = time.perf_counter()
            encoders1 = encoders2
            countAcc += 1             
    stop(1)

# forward method for field detection, looks for green corners
def forwardForDetection():
    blocks = BlockArray(100)

    #start pi control
    a_star.motors(70, 70)
    leftCount = 0 # use leftCount as goal encoder count
    rightCount = 0
    leftAcc = 0
    rightAcc = 0
    countAcc = 0
    startTime = time.perf_counter()
    encoders1 = a_star.read_encoders()
    time2 = time.perf_counter()
    while 1:
        if ((time.perf_counter() - time2) > 0.05): # look at encoder values every 0.05 second

            encoders2 = a_star.read_encoders()
            righten = encoders2[1] - encoders1[1]
            leften = encoders2[0] - encoders1[0]

            if (righten > 0):
                rightError = 35 - righten
                rightCount += righten
            if (leften > 0):
                leftError = 35 - leften
                leftCount += leften

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
            sig = int(blocks[0].m_signature)
            # make sure block is large enough to be relevant and signature is green
            if (sig == 1):
                print(sig)
            else:
                if (blocks[0].m_width > 50 and blocks[0].m_height > 50):
                    a_star.motors(0, 0) # stops if a block is detected
                    print("detected a corner")
                    print(blocks[0].m_signature)
                    break
    stop(2)
    return leftCount

# stops motors after moving
def stop(t):
    a_star.motors(0, 0)
    time.sleep(t)

# config file controlled forward
def forwardOld(t):
    a_star.motors(lWheel, rWheel)
    time.sleep(t)
    stop(1)

# config file controlled backward
def backward(t):
    a_star.motors(-lWheel, -rWheel)
    time.sleep(t)
    stop(1)

# 90 degree left turn
def turnLeft():
    rightmotorInitial = a_star.read_encoders()[1]
    a_star.motors(0, 60)
    go = True

    while go:
        rightmotor = a_star.read_encoders()[1]
        go = (abs(rightmotor - rightmotorInitial) < 1410)
    stop(1)

# 90 degree right turn
def turnRight():
    leftmotorInitial = a_star.read_encoders()[0]
    a_star.motors(60, 0)
    go = True

    while go:
        leftmotor = a_star.read_encoders()[0]
        go = (abs(leftmotor - leftmotorInitial) < 1410)
    stop(1)

# executes a planting path with field parameters
def runPath (x, y, w): # arguments passed are encoder counts
    global currentLineCount
    global yLength
    global goingDown
    global distAlongY
    
    distAlongY = 0
    yLength = y
    goingDown = True
    obstacles = []

    countPlant = int(int(y)*countToCM)
    maxCount = int(x)/int(w)
    widthCount = int(int(w)*countToCM)
    if maxCount % 10 == 0:
        maxCount += 1
    leftOrRight = False # False: initially turn left, True: initially turn right

    currentLineCount = 0
    count = 0

    #initialize camera
    pixy.init()
    pixy.set_lamp(1, 1)
    pixy.change_prog ("color_connected_components")

    while count <= maxCount:
        forward(countPlant)
        currentLineCount += 1
        goingDown = not goingDown
        # width is length of romi (15cm*countToCM) + a count num
        if leftOrRight:
            turnRight()
            forwardOnX(widthCount - int(15*countToCM))
            turnRight()
            leftOrRight = False
        else:
            turnLeft()
            forwardOnX(widthCount - int(15*countToCM))
            turnLeft()
            leftOrRight = True
        count += 1

    pixy.set_lamp(0, 0)


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
