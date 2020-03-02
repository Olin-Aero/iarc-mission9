#!/usr/bin/env python2

import rospy
from std_msgs.msg import String
import re
from iarc_forebrain.msg import MovePiece,Piece

class key2Square():
	"""
	Takes input from a keyboard in the form of "A6 to B7" or "d2 -> e5" and 
	converts it to an inital and destination of the requested move.
	Publisher:  /req_initial -> square of piece to be moved
				/req_destination -> square for piece to move to
	"""
	def __init__(self):
		rospy.init_node('keyboardInput', anonymous=True)
		self.requestPub = rospy.Publisher('/req_square',MovePiece,queue_size=10)

	def run(self):
		while not rospy.is_shutdown():
			text = raw_input("Make a move (e.g. A5 to B6):")
			squares = re.findall("[A-F]+[1-8]",text.upper())
			if(len(squares) == 2):
				request = MovePiece()
				request.init_square = squares[0]
				request.dest_square = squares[1]
				self.requestPub.publish(request)
			elif(len(squares) < 2):
				rospy.loginfo("Not enough squares input or mistyped square")
			elif(len(squares > 2)):
				rospy.loginfo("Too many squares input")

if __name__ == '__main__':
	key = key2Square()
	key.run()