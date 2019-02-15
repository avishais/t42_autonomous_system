#!/usr/bin/env python

import rospy
import numpy as np
import time
import random
from transition_experience import *
from std_msgs.msg import Float64MultiArray, Float32MultiArray, String, Bool
from std_srvs.srv import Empty, EmptyResponse



class actorPubRec():
    discrete_actions = True

    gripper_pos = np.array([0., 0.])
    gripper_load = np.array([0., 0.])
    obj_pos = np.array([0., 0.])
    drop = True
    Done = False
    running = False
    action = np.array([0.,0.])
    state = np.array([0.,0., 0., 0.])
    n = 0
    
    texp = transition_experience(Load=True, discrete = discrete_actions, postfix = 'bu')

    def __init__(self):
        rospy.init_node('actor_pub_record', anonymous=True)

        rospy.Subscriber('/gripper/load', Float32MultiArray, self.callbackGripperLoad)
        rospy.Subscriber('/hand_control/obj_pos_mm', Float32MultiArray, self.callbackObj)
        rospy.Subscriber('/hand_control/drop', Bool, self.callbackDrop)
        rospy.Subscriber('/collect/action', Float32MultiArray, self.callbackAction)

        rospy.Service('/actor/trigger', Empty, self.callbackTrigger)

        rate = rospy.Rate(15)
        count = 1
        while not rospy.is_shutdown():

            if self.running:
                count += 1

                self.state = np.concatenate((self.obj_pos, self.gripper_load), axis=0)
                self.texp.add(self.state, self.action, self.state, self.drop)

                if self.drop:
                    print('[actor_record] Episode ended (%d points so far).' % self.texp.getSize())
                    self.running = False
                    if not (count % 3):
                        self.texp.save()

            rate.sleep()

    def callbackGripperLoad(self, msg):
        self.gripper_load = np.array(msg.data)

    def callbackObj(self, msg):
        self.obj_pos = np.array(msg.data)

    def callbackDrop(self, msg):
        self.drop = msg.data

    def callbackAction(self, msg):
        self.action = np.array(msg.data)

    def callbackTrigger(self, msg):
        self.running = not self.running

        return EmptyResponse()


if __name__ == '__main__':
    
    try:
        actorPubRec()
    except rospy.ROSInterruptException:
        pass
