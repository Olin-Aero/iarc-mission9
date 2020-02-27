#!/usr/bin/env python2

import rospy
from std_msgs.msg import String
from iarc_forebrain.msg import MovePiece,Piece
import chess


class legalMoveChecker(object):
	"""docstring for legalMoveChecker"""
	def __init__(self):
		rospy.init_node('legalMoveChecker', anonymous=True)
		rospy.Subscriber("/req_move",MovePiece,self.requestedMoveCB)
		self.moveSquarePub = rospy.Publisher("/move_square",MovePiece,queue_size=10)
		self.move = None
		self.rate = rospy.Rate(10)
		
	def requestedMoveCB(self,msg):
		self.move = msg

	def checkMove(self):
		board = chess.Board("8/8/8/8/8/8/8/8")
		executive = "board.set_piece_at(chess.{},chess.Piece(chess.{},chess.{}))".format(self.move.init_square.data,self.move.init_piece.type.data,self.move.init_piece.color.data)
		exec(executive)
		executive = "board.set_piece_at(chess.{},chess.Piece(chess.{},chess.{}))".format(self.move.dest_square.data,self.move.dest_piece.type.data,self.move.dest_piece.color.data)
		if(chess.Move.from_uci((req_initial+req_destination).lower()) in board.legal_moves):
			self.moveSquarePub.publish(self.move)
		else:
			rospy.logerror("Illegal Move, try again.")
			
		self.move = None


	def main(self):
		while not rospy.is_shutdown():
			if(self.move != None):
				self.checkMove()
			self.rate.sleep()


if __name__ == '__main__':
	somenn = legalMoveChecker()
	somenn.main()

	

	

	
		