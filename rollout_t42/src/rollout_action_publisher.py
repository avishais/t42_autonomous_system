#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32MultiArray, Bool
from std_srvs.srv import Empty, EmptyResponse, SetBool
from rollout_t42.srv import rolloutReq, rolloutReqFile, plotReq, observation, IsDropped, TargetAngles, gets
from hand_control.srv import RegraspObject, close
import numpy as np
import matplotlib.pyplot as plt
import pickle


class rolloutPublisher():

    states = []
    plot_num = 0
    arm_status = ' '
    trigger = False # Enable collection
    drop = True

    def __init__(self):
        rospy.init_node('rollout_action_publisher', anonymous=True)

        rospy.Service('/rollout/rollout', rolloutReq, self.CallbackRollout)
        rospy.Service('/rollout/rollout_from_file', rolloutReqFile, self.CallbackRolloutFile)
        self.run_srv = rospy.ServiceProxy('/rollout/run_trigger', SetBool)
        self.record_srv = rospy.ServiceProxy('/rollout/record_trigger', SetBool)

        self.action_pub = rospy.Publisher('/rollout/action', Float32MultiArray, queue_size = 10)

        self.arm_reset_srv = rospy.ServiceProxy('/RegraspObject', RegraspObject)
        rospy.Subscriber('/ObjectIsReset', String, self.callbackTrigger)
        rospy.Subscriber('/cylinder_drop', Bool, self.callbackObjectDrop)
        self.drop_srv = rospy.ServiceProxy('/IsObjDropped', IsDropped)
        self.gets_srv = rospy.ServiceProxy('/rollout/get_states', gets)

        self.state_dim = 4
        self.action_dim = 2
        self.stepSize = 1 

        self.rate = rospy.Rate(1) # 15hz
        rospy.spin()

    def ResetArm(self):
        while 1:
            if not self.trigger and self.arm_status == 'waiting':
                print('[rollout_action_publisher] Waiting for arm to grasp object...')
                self.arm_reset_srv()
                rospy.sleep(1.0)
            self.rate.sleep()
            if self.arm_status != 'moving' and self.trigger:
                self.rate.sleep()
                if self.drop_srv().dropped: # Check if really grasped
                    self.trigger = False
                    print('[rollout_action_publisher] Grasp failed. Restarting')
                    continue
                else:
                    break
        self.trigger = False

    def run_rollout(self, A):
        self.rollout_transition = []
        self.trigger = False
        self.ResetArm()        

        msg = Float32MultiArray()    

        print("[rollout_action_publisher] Rolling-out actions...")
        self.run_srv(True)
        self.record_srv(True)
        
        # Publish episode actions
        success = True
        for action in A:
            msg.data = action
            self.action_pub.publish(msg)
            print action

            if self.drop:
                print("[rollout_action_publisher] Fail.")
                break

            self.rate.sleep()

        self.run_srv(False)
        self.record_srv(False)

        print("[rollout_action_publisher] Rollout done.")

    def callbackObjectDrop(self, msg):
        self.drop = msg.data

    def callbackTrigger(self, msg):
        self.arm_status = msg.data
        if not self.trigger and self.arm_status == 'finished':
            self.trigger = True

    def CallbackRollout(self, req):

        print('[rollout_action_publisher] Rollout request received.')
        
        actions = np.array(req.actions).reshape(-1, self.action_dim)
        self.run_rollout(actions)

        SA = self.gets_srv()
        states = np.array(SA.states)
        # actions = SA.actions 

        return {'states': states.reshape((-1,)), 'success' : True}

    def CallbackRolloutFile(self, req):

        file_name = req.file

        actions = np.loadtxt(file_name, delimiter=',', dtype=float)[:,:2]
        self.run_rollout(actions)

        return {'states': actions.reshape((-1,)), 'success' : True}


if __name__ == '__main__':
    try:
        rolloutPublisher()
    except rospy.ROSInterruptException:
        pass