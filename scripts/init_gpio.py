# test code
# controls direction by up, down, left, and right arrows and speed by < and > keys

# reading keyboard - readchar, readkey
import sys
import tty
import termios
import numpy as np
# import controling speeds and movements
import move
import time
import math
#import gyro

up = 0
down = 1
right = 2
left = 3

def readchar():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno()) # sets the tty in raw mode - do not have to press enter after each key
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings) # returns tty to old settings - takes out of raw mode
    if ch == '0x03':
        raise KeyboardInterrupt
    return ch

def readkey(getchar_fn = None):
    getchar = getchar_fn or readchar
    c1 = getchar()
    if ord(c1) != 0x1b:
        return c1
    c2 = getchar()
    if ord(c2) != 0x5b:
        return c1
    c3 = getchar()
    return ord(c3) - 65

# initializing robot movement and speed
speed = 40
move.init()
# main body of code

print("Use the up (w), down (s), left (a), or right (d) keys to move!")
print("Press < to decrease speed, > to increase speed, and the space bar to stop!\n")

while True:
    try:
        keyp = readkey()
        if keyp == up or keyp == 'w':
            move.forward(speed)
            print('Forward', speed, "\n")
        elif keyp == down or keyp == 's':
            move.reverse(speed), "\n"
            print('Backward', speed)
        elif keyp == right or keyp == 'd':
            move.spinRight(speed)
            print('Spin Right', speed ,"\n")
        elif keyp == left or keyp == 'a':
            move.spinLeft(speed),"\n"
            print('Spin Left', speed)
        elif keyp == '.' or keyp == '>':
            speed = min(100, speed + 10) # keeps speed from exceeding 100
            print('Speed+', speed,"\n")
        elif keyp == ',' or keyp == '<':
            speed = max(0, speed - 10) # keeps speed at or above 0
            print('Speed-', speed, "\n")
        elif keyp == ' ':
            move.stop()
            print('Stop', "\n")
        elif ord(keyp) == 3: # end of text
            break

    except KeyboardInterrupt: # press ctrl+c to exit - this allows user to kill multiple threads
        move.cleanup()
fd.close()
