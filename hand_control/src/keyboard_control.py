#!/usr/bin/python 

'''
----------------------------
Author: Avishai Sintov
        Rutgers University
Date: October 2018
----------------------------
'''


import rospy
import numpy as np 
from std_msgs.msg import Char
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from openhand.srv import MoveServos
from std_srvs.srv import Empty, EmptyResponse
# import tty, termios, sys
from std_srvs.srv import Empty, EmptyResponse
from hand_control.srv import observation, IsDropped, TargetAngles, RegraspObject, close, MovePrim
import curses, time

def key_listener(stdscr):
    """checking for keypress"""
    stdscr.nodelay(True)  # do not wait for input when calling getch
    return stdscr.getch()

class keyboard_control():

    A = np.array([[1.0,1.0],[-1.,-1.],[-1.,1.],[1.,-1.],[1.5,0.],[-1.5,0.],[0.,-1.5],[0.,1.5],[0.,0.]])

    def __init__(self):
        rospy.init_node('keyboard_control', anonymous=True)

        pub_action = rospy.Publisher('/keyboard/desired_action', Float32MultiArray, queue_size=10)
        rospy.Subscriber('/gripper/pos', Float32MultiArray, self.ActPosCallback)
        rospy.Service('/ResetKeyboard', Empty, self.ResetKeyboard)
        move_srv = rospy.ServiceProxy('/MovePrimitive', MovePrim)

        rate = rospy.Rate(5)
        self.ch = 's'
        while not rospy.is_shutdown():
            c = curses.wrapper(key_listener)#self.getchar()
            if c!=-1:
                self.ch = chr(c)

            if ord(self.ch) == 27:
                break

            if np.any(self.ch == np.array(['o','p','n','m'])):
                move_srv(self.ch)
                self.ch = 's'
                rospy.sleep(1.0)
            else:
                move_srv(self.ch)

            rate.sleep()

    def ActPosCallback(self, msg):
        self.act_angles = np.array(msg.data)

    def Move(self, ch):

        if chr(ch) == 's': # Don't move
            return self.A[8]
        if chr(ch) == 'x': # Down
            return self.A[0]            
        if chr(ch) == 'w': # Up
            return self.A[1]
        if chr(ch) == 'a': # Left
            return self.A[2]
        if chr(ch) == 'd': # Right
            return self.A[3]
        if chr(ch) == 'c': # Down-Right
            return self.A[4]
        if chr(ch) == 'z': # Down-left
            return self.A[7]
        if chr(ch) == 'e': # Up-Right
            return self.A[6]
        if chr(ch) == 'q': # Up-Left
            return self.A[5]
        
        if chr(ch) == '[': # Close
            return np.array([100,100])
        if chr(ch) == ']': # Open
            return np.array([-100, -100])

    def ResetKeyboard(self, msg):
        self.ch = ord('s')

        return EmptyResponse()


if __name__ == '__main__':

    try:
        keyboard_control()
    except rospy.ROSInterruptException:
        pass

    
