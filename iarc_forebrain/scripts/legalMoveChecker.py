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
		board = chess.Board("8/8/8/8/8/8/8/8")
		executive = "board.set_piece_at(chess.{},chess.Piece(chess.{},chess.{}))".format(self.req_initial,self.piece_initial.type,self.piece_initial.color)
		exec(executive)
		executive = "board.set_piece_at(chess.{},chess.Piece(chess.{},chess.{}))".format(self.req_destination,self.piece_destination.type,self.piece_destination.color)
		if(chess.Move.from_uci((req_initial+req_destination).lower()) in board.legal_moves):
			if(piece_destination == None):
				self.move_initial.publish(self.req_initial)
				self.move_destination.publish(self.req_destination)
			else:
				self.move_initial.publish(self.req_destination)
				self.move_destination.publish("TRASH")
				self.move_initial.publish(self.req_initial)
				self.move_destination.publish(self.req_destination))


	def main(self):
		while not rospy.is_shutdown():
			self.checkMove()
			self.rate.sleep()


if __name__ == '__main__':
	somenn = legalMoveChecker()
	somenn.main()

	

	

	
		