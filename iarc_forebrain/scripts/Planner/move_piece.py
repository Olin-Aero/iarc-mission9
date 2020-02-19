#!/usr/bin/env python2

''' Move a piece from one square to another '''

import rospy
import math
from mode import Mode

import numpy as np

# TODO: simulator add board frame, fix height, test code

class MovePiece(Mode):

    def __init__(self, drone):
        Mode.__init__(self, drone)

    def enable(self, start, end):
        self.defaultPos = self.drone.get_pos("board").pose.position
        self.defaultLand = False
        self.startPos = get_coordinates(start)
        self.endPos = get_coordinates(end)
        self.active = True
        self.state = 0
        self.t0 = rospy.get_time()
        print("Moving piece from " + start + " to " + end)

    def update(self, look_direction=0, obstacles=[]):
        # Land on piece
        if self.state == 0:
            if self.drone.is_flying():
                if self.drone.move_towards(startPos[0], startPos[1], "board"):
                    self.state += 1
                    self.t0 = rospy.get_time()
                    self.drone.land()
                    print("Landing on target piece")
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
            if self.drone.move_towards(endPos[0], endPos[1], "board"):
                self.state += 1
                self.t0 = rospy.get_time()
                self.drone.land()
                print("Landing on destination square")
        # Release piece
        elif self.state == 3:
            if rospy.get_time() - self.t0 > 2.0: # wait for landing to finish
                # TODO: release magnet
                print("Releasing electromagnet")
                self.state += 1
                self.t0 = rospy.get_time()
                self.drone.takeoff()
                print("Taking off")
        # Return to starting position
        elif self.state == 4:
            if self.drone.move_towards(defaultPos[0], defaultPos[1], "board"):
                if self.defaultLand:
                    self.drone.land()
                self.active = False
                print("Move completed")

def get_coordinates(square):
    ''' Converts chess coordinates to world coordinates in board frame '''
    SQUARE_WIDTH = 1 # width of a grid square in meters
    column = ord(square[0].lower())-96
    row = int(square[1])
    return ((column-0.5)*SQUARE_WIDTH, (row-0.5)*SQUARE_WIDTH)
    # TODO: rotation if necessary


if __name__ == '__main__':
    pass