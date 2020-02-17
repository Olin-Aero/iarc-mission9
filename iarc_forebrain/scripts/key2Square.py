#!/usr/bin/env python2

import rospy
from std_msgs.msg import String
import re

class keyboardInput():
	def __init__(self):
		rospy.init_node('keyboardInput', anonymous=True)
		self.initPub = rospy.Publisher('/req_initial',String,queue_size=10)
		self.destPub = rospy.Publisher('/req_destination',String,queue_size=10)

	def run(self):
		while not rospy.is_shutdown():
			text = raw_input("Make a move (e.g. A5 to B6):")
			squares = re.findall("[A-F]+[1-8]",text.upper())
			if(len(squares) == 2):
				self.initPub.publish(String(squares[0]))
				self.destPub.publish(String(squares[1]))
			elif(len(squares) < 2):
				rospy.loginfo("Not enough squares input or mistyped square")
			elif(len(squares > 2)):
				rospy.loginfo("Too many squares input")

if __name__ == '__main__':
	key = keyboardInput()
	key.run()