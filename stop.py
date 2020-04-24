from a_star import AStar
a_star = AStar()
path = []

import time

def stop(t):
    a_star.motors(0, 0)
    time.sleep(t)

if __name__ == "__main__":
    stop(10)