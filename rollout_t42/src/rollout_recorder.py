#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32MultiArray, Bool
from std_srvs.srv import Empty, EmptyResponse, SetBool
from rollout_t42.srv import rolloutReq, rolloutReqFile, plotReq, observation, IsDropped, TargetAngles, gets
from hand_control.srv import RegraspObject, close
import numpy as np
import matplotlib.pyplot as plt
import pickle

class rolloutRecorder():

    gripper_load = np.array([0., 0.])
    obj_pos = np.array([0., 0.])
    drop = True
    running = False
    action = np.array([0.,0.])
    S = A = []
    suc = True
    drop_counter = 0
    fail = False
    

    def __init__(self):
        rospy.init_node('rollout_recorder', anonymous=True)

        rospy.Subscriber('/gripper/load', Float32MultiArray, self.callbackGripperLoad)
        rospy.Subscriber('/hand_control/obj_pos_mm', Float32MultiArray, self.callbackObj)
        # rospy.Subscriber('/cylinder_drop', Bool, self.callbackObjectDrop)
        rospy.Subscriber('/rollout/action', Float32MultiArray, self.callbackAction)
        # rospy.Subscriber('/rollout/move_success', Bool, self.callbackSuccess)
        rospy.Subscriber('/rollout/fail', Bool, self.callbacFail)
        
        rospy.Service('/rollout/record_trigger', SetBool, self.callbackTrigger)
        rospy.Service('/rollout/get_states', gets, self.get_states)

        self.rate = rospy.Rate(10) # 15hz
        while not rospy.is_shutdown():

            if self.running:
                self.state = np.concatenate((self.obj_pos, self.gripper_load), axis=0)

                self.S.append(self.state)
                self.A.append(self.action)

                if self.fail:
                    print('[rollout_recorder] Episode ended.')
                    self.running = False
                    # File = self.path + self.planning_algorithm + '_goal' + str(goal[0]) + '_' + str(goal[1]) + '_n' + self.id + '_rollout.txt'
                    # np.savetxt(File, S, delimiter = ', ')

            self.rate.sleep()

    def callbackGripperLoad(self, msg):
        self.gripper_load = np.array(msg.data)

    def callbackObj(self, msg):
        self.obj_pos = np.array(msg.data)*1000

    def callbackAction(self, msg):
        self.action = np.array(msg.data)

    def callbacFail(self, msg):
        self.fail = msg.data

    def callbackTrigger(self, msg):
        self.running = msg.data
        if self.running:
            self.S = []
            self.A = []
            print('[rollout_recorder] Recording started.')

        return {'success': True, 'message': ''}

    def get_states(self, msg):
        # S = self.medfilter(np.array(self.S), 20)

        return {'states': np.array(self.S).reshape((-1,)), 'actions': np.array(self.A).reshape((-1,))}

    def medfilter(self, x, W):
        print('[rollout_recorder] Smoothing data...')
        w = int(W/2)
        x_new = np.copy(x)
        for i in range(0, x.shape[0]):
            if i < w:
                x_new[i] = np.mean(x[:i+w])
            elif i > x.shape[0]-w:
                x_new[i] = np.mean(x[i-w:])
            else:
                x_new[i] = np.mean(x[i-w:i+w])
        return x_new



if __name__ == '__main__':
    try:
        rolloutRecorder()
    except rospy.ROSInterruptException:
        pass