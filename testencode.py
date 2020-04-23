# test reading romi encoders
# encoders have 12 counts per motor shaft rotation --> 1440 counts per revolution of wheel

from a_star import AStar
a_star = AStar()
path = []

import time

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

def forward(t):
    a_star.motors(70, 70)
    leftAcc = 0
    rightAcc = 0
    countAcc = 0
    startTime = time.perf_counter()
    encoders1 = a_star.read_encoders()
    print (encoders1)
    time2 = time.perf_counter()
    while ((time.perf_counter() - startTime) < t):
        if ((time.perf_counter() - time2) > 0.1): # look at encoder values every second
            encoders2 = a_star.read_encoders()
            # goal: 313 counts/second
            righten = encoders2[1] - encoders1[1]
            leften = encoders2[0] - encoders1[0]

            if (righten > 0):
                rightError = 70 - righten
            if (leften > 0):
                leftError = 70 - leften
            print (str(leftError) + " " + str(rightError))

            if (countAcc > 3):
                leftAcc += leftError
                rightAcc += rightError

            # calculate new motor parameters and apply to motor
            left = piControl(leftError, leftAcc, 0)
            right = piControl(rightError, rightAcc, 1)
            a_star.motors(left, right)

            # reset time 2 /encoders
            time2 = time.perf_counter()
            encoders1 = encoders2
            countAcc += 1
        
    stop(1)

def forwardOld(t):
    a_star.motors(60, 57)
    encoders = a_star.read_encoders() #tuple type
    print (encoders[0])
    print (encoders[1])
    time.sleep(t)
    stop(1)

def turnLeft():
    rightmotorInitial = a_star.read_encoders()[1]
    print(rightmotorInitial)
    a_star.motors(0, 60)
    go = True

    while go:
        rightmotor = a_star.read_encoders()[1]
        go = (abs(rightmotor - rightmotorInitial) < 1500)

    stop(1)

def turnRight():
    leftmotorInitial = a_star.read_encoders()[0]
    print(leftmotorInitial)
    a_star.motors(60, 0)
    go = True

    while go:
        leftmotor = a_star.read_encoders()[0]
        go = (abs(leftmotor - leftmotorInitial) < 1500)

    stop(1)

def stop(t):
    a_star.motors(0, 0)
    time.sleep(t)

if __name__ == "__main__":
    # implement PI control
    # target speed = 5cm/s --> 313 counts/sec

    # test printing of encoder data
    forward(15)

    # make wheel turn 90 degrees
    # wheel r = 3.5cm, dist = 2*pi*r = 22cm
    # distance for 90 degrees = 2*pi*15cm/4 (quarter circle) = 23.6cm
    # counts for one turn = 1440counts/22cm * 23.6cm = 1545 counts
    #1545 seems slightly too much, change to 1500
    #turnLeft()
    #turnRight()

