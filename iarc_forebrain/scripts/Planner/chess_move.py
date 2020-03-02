#!/usr/bin/env python2

''' Move a piece from one square to another '''

import rospy
import math
from mode import Mode

import numpy as np

# TODO: capturing pieces

TOL = 0.1 # Position tolerance in meters

class ChessMove(Mode):

    def __init__(self, drone):
        Mode.__init__(self, drone)

    def enable(self, start, end, reset=False):
        defaultPos = self.drone.get_pos("board").pose.position
        self.defaultPos = [defaultPos.x, defaultPos.y]
        self.defaultLand = False
        self.startPos = get_coordinates(start)
        self.endPos = get_coordinates(end)
        self.active = True
        self.state = 0
        self.reset = reset
        self.t0 = rospy.get_time()
        print("Moving piece from " + start + " to " + end)

    def update(self, look_direction=0, obstacles=[]):
        # Land on piece
        if self.state == 0:
            if self.drone.is_flying():
                if self.drone.move_towards(self.startPos[0], self.startPos[1], "board", tol=TOL):
                    self.state += 1
                    self.t0 = rospy.get_time()
                    self.drone.land()
                    print("Landing on target piece")
                    print(self.drone.get_pos("board"))
            else:
                self.drone.takeoff()
                self.defaultLand = True
                print("Taking off")
        # Pick up piece
        elif self.state == 1:
            if rospy.get_time() - self.t0 > 2.0: # wait for landing to finish
                # TODO: trigger magnet
                print("Engaging electromagnet")
                self.state += 1
                self.t0 = rospy.get_time()
                self.drone.takeoff()
                print("Taking off")
        # Land on destination square
        elif self.state == 2:
            if self.drone.move_towards(self.endPos[0], self.endPos[1], "board", tol=TOL):
                self.state += 1
                self.t0 = rospy.get_time()
                self.drone.land()
                print("Landing on destination square")
                print(self.drone.get_pos("board"))
        # Release piece
        elif self.state == 3:
            if rospy.get_time() - self.t0 > 2.0: # wait for landing to finish
                # TODO: release magnet
                print("Releasing electromagnet")
                self.state += 1
                self.t0 = rospy.get_time()
                self.drone.takeoff()
                print("Taking off")
                if not self.reset:
                    print("Move completed")
                    self.drone.hover()
        # Return to starting position
        elif self.state == 4 and self.reset:
            if self.drone.move_towards(self.defaultPos[0], self.defaultPos[1], "board", tol=TOL):
                if self.defaultLand:
                    self.drone.land()
                self.active = False
                print("Move completed")
                self.drone.hover()

def get_coordinates(square):
    ''' Converts chess coordinates to world coordinates in board frame '''
    SQUARE_WIDTH = 1 # width of a grid square in meters
    column = ord(square[0].lower())-96
    row = int(square[1])
    return ((column-0.5)*SQUARE_WIDTH, (row-0.5)*SQUARE_WIDTH)
    # TODO: rotation if necessary


if __name__ == '__main__':
    pass