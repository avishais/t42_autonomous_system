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
    suc = True
    drop_counter = 0

    def __init__(self):
        rospy.init_node('rollout_t42', anonymous=True)

        self.move_srv = rospy.ServiceProxy('/MoveGripper', TargetAngles)
        self.arm_reset_srv = rospy.ServiceProxy('/RegraspObject', RegraspObject)
        rospy.Subscriber('/cylinder_drop', Bool, self.callbackObjectDrop)
        rospy.Subscriber('/rollout/action', Float32MultiArray, self.callbackAction)
        rospy.Service('/rollout/run_trigger', SetBool, self.callbackTrigger)
        # suc_pub = rospy.Publisher('/rollout/move_success', Bool, queue_size=10)
        fail_pub = rospy.Publisher('/rollout/fail', Bool, queue_size = 10)

        print('[rollout] Ready to rollout...')

        self.rate = rospy.Rate(2.5) # 15hz
        while not rospy.is_shutdown():
            # suc_pub.publish(self.suc)

            if self.running:
                print self.action
                self.suc = self.move_srv(self.action).success

                fail_pub.publish(not self.suc or self.drop)
               
                if not self.suc:
                    print("[rollout_actor] Load Fail")
                    self.running = False
                elif self.drop:
                    print("[rollout_actor] Drop Fail")
                    self.running = False

            self.rate.sleep()

    def callbackAction(self, msg):
        self.action = np.array(msg.data)

    def callbackObjectDrop(self, msg):
        if (msg.data):
            self.drop_counter += 1
        else:
            self.drop_counter = 0
        self.drop = (self.drop_counter >= 7)

    def callbackTrigger(self, msg):
        self.running = msg.data
        if self.running:
            self.suc = True

        return {'success': True, 'message': ''}


if __name__ == '__main__':
    try:
        rollout()
    except rospy.ROSInterruptException:
        pass