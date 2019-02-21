#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32MultiArray, Bool
from std_srvs.srv import Empty, EmptyResponse, SetBool
from rollout_t42.srv import rolloutReq, rolloutReqFile, plotReq, observation, IsDropped, TargetAngles
from hand_control.srv import RegraspObject, close
import numpy as np
import matplotlib.pyplot as plt
import pickle


class rollout():

    states = []
    plot_num = 0
    arm_status = ' '
    drop = True
    running = False
    action = np.array([0.,0.])

    def __init__(self):
        rospy.init_node('rollout_t42', anonymous=True)

        self.move_srv = rospy.ServiceProxy('/MoveGripper', TargetAngles)
        self.arm_reset_srv = rospy.ServiceProxy('/RegraspObject', RegraspObject)
        rospy.Subscriber('/cylinder_drop', Bool, self.callbackObjectDrop)
        rospy.Subscriber('/rollout/action', Float32MultiArray, self.callbackAction)
        rospy.Service('/rollout/run_trigger', SetBool, self.callbackTrigger)

        print('[rollout] Ready to rollout...')

        self.rate = rospy.Rate(1) # 15hz
        while not rospy.is_shutdown():

            if self.running:
                suc = self.move_srv(self.action).success

                # next_state = np.array(self.obs_srv().state)

                if not suc or self.drop:
                    print("[rollout] Fail")
                    self.running = False

            self.rate.sleep()

    def callbackAction(self, msg):
        self.action = np.array(msg.data)

    def callbackObjectDrop(self, msg):
        self.drop = msg.data

    def callbackTrigger(self, msg):
        self.running = msg.data

        return {'success': True, 'message': ''}


if __name__ == '__main__':
    try:
        rollout()
    except rospy.ROSInterruptException:
        pass