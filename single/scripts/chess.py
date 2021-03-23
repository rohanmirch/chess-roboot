#!/usr/bin/env python
#
#   pypickup.py
#
#
import sys
import rospy
import math
import rosbag
import time
import numpy as np
import cv2
import pickle
import os
import pyimage


## Python-chess library
import chess 
import chess.engine
import chess.uci
print(chess.__version__)

from sensor_msgs.msg import JointState
from std_msgs.msg    import Float64, String
from opencv_apps.msg import FaceArrayStamped
from single.msg import Num
from threading import Lock


M = None

class Game:
    def __init__(self):
        self.board = chess.Board()
        self.move_number = 0
        path = "/usr/local/Cellar/stockfish/11/bin/stockfish"
        self.engine = chess.uci.popen_engine(path)
        
    def find_move_made(self, board1, board2):
        '''Takes in two board fen strings and finds the move that was made. '''
        if board1.board_fen() == board2.board_fen():
            print("Board hasn't changed")
            return None
        fro = -1
        to = -1
        b1 = board1.piece_map()
        b2 = board2.piece_map()
        print(b1.keys())
        print(b2.keys())
        b2s = set(b2)
        for pos in b1:
            # If we are in a positon that becomes empty, we move from that
            if pos not in b2s:
                fro = int(pos)
                break     
        for pos in b2:
            # If we are in a positon that was previously empty
            # or filled with another chess piece, it was moved to
            if b1.get(pos, "-1") == "-1" or b1.get(pos, "-1") != b2.get(pos, "-1"):
                to = int(pos)
                break   
        print("from {}, to {}".format(fro,to))
        if fro == -1 or to == -1:
            return None
        
        move = chess.Move(fro, to)
        if move not in board1.legal_moves:
            print("illegal move")
            return None
        return move.uci()
    

    def make_user_move(self, image = None, move = None, fen = None):
        '''Makes a user move from the image detection (or inputted move).
           Returns 1 if a user has made a move and 0 if not. '''
        ### Including custom Fen strings for testing purposes
        if image != None:
            result, corners = pyimage.get_board()
            detect_board = chess.Board(result)
            move = self.find_move_made(self.board, detect_board)
        elif fen != None:
            detect_board = chess.Board()
            detect_board.set_board_fen(fen)
            move = self.find_move_made(self.board, detect_board)

        if move != None:
            self.board.push_uci(move)
            ###### NEED MOVE FUNCTION THAT MOVES TEH ACTUAL ROBOT
            if self.board.is_game_over():
                print("game over")
            return 1 
                
        return 0


    def make_cardi_move(self):
        '''Plays a move from the AI and returns the integer positional representation'''
        self.engine.position(self.board)
        result = self.engine.go(movetime = 500)
        move = result.bestmove
        self.board.push(move)
        print("cardi Move: ", move.uci())
        if self.board.is_game_over():
            print("game over - cardi wins")
        return (move.from_square, move.to_square)

    def is_game_over(self):
        return self.board.is_game_over()
    
    def start_game_loop(self):
        print("starting game loop")
        while True:
            self.make_cardi_move()
            print("cardi making move...")
            time.sleep(5)
            while(self.make_user_move(image = 1) == 0):
                print("no user move detected...")
                time.sleep(3)
            if self.board.is_game_over():
                break



#### NEED MOE FUNCTION THAT WILL INCORPORATE CHATED"S code and actually send cardi commands
# def move_cardi(p1, p2, corners)

if __name__ == "__main__":
    rospy.init_node('pychess')
    

    #Load Map array object 
    pickle_in = open("mapping_arr.pickle","rb")
    M = pickle.load(pickle_in)

    #create publisher for move cfunctrion
    pub = rospy.Publisher('/goal', Num, queue_size=5)
    print("sending real first message")
    pub.publish(.2125,0, .3)
    time.sleep(1) 
    

   

    

    

    

    
