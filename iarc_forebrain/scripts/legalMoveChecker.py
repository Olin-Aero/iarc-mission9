#!/usr/bin/env python2

import rospy
from std_msgs.msg import String

class legalMoveChecker(object):
	"""docstring for legalMoveChecker"""
	def __init__(self):
		rospy.init_node('legalMoveChecker', anonymous=True)
		rospy.Subscriber("/req_initial",String,self.req_initialCB)
		rospy.Subscriber("/req_destination",String,self.req_destinationCB)
		rospy.Subscriber("/piece_initial",String,self.piece_initialCB)
		rospy.Subscriber("/piece_destination",String,self.piece_destinationCB)
		self.moveInitPub = rospy.Publisher("/move_initial",String,queue_size=10)
		self.moveDestPub = rospy.Publisher("/move_destination",String,queue_size=10)
		self.req_initial = None
		self.req_destination = None
		self.piece_initial = None
		self.piece_destination = None
		self.rate = rospy.Rate(10)
		
	def req_initalCB(self,msg):
		self.req_initial = msg.data

	def req_destinationCB(self,msg):
		self.req_destination = msg.data

	def piece_initialCB(self,msg):
		self.piece_initial = msg.data

	def piece_destinationCB(self,msg):
		self.piece_destination = msg.data

	def checkMove(self):
		chess.Move.from_uci

	def main(self):
		while not rospy.is_shutdown():
			self.checkMove()
			self.rate.sleep()


if __name__ == '__main__':
	somenn = legalMoveChecker()
	somenn.main()

	

	

	
		