#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32MultiArray
from std_srvs.srv import Empty, EmptyResponse
from rollout_node.srv import rolloutReq, rolloutReqFile, plotReq, observation, IsDropped, TargetAngles
from hand_control.srv import RegraspObject, close
import numpy as np
import matplotlib.pyplot as plt
import pickle


class rollout():

    states = []
    plot_num = 0
    arm_status = ' '
    trigger = False # Enable collection

    def __init__(self):
        rospy.init_node('rollout_node', anonymous=True)

        rospy.Service('/rollout/rollout', rolloutReq, self.CallbackRollout)
        rospy.Service('/rollout/rollout_from_file', rolloutReqFile, self.CallbackRolloutFile)
        rospy.Service('/rollout/plot', plotReq, self.Plot)

        self.obs_srv = rospy.ServiceProxy('/observation', observation)
        self.drop_srv = rospy.ServiceProxy('/IsObjDropped', IsDropped)
        self.move_srv = rospy.ServiceProxy('/MoveGripper', TargetAngles)
        self.arm_reset_srv = rospy.ServiceProxy('/RegraspObject', RegraspObject)
        rospy.Subscriber('/ObjectIsReset', String, self.callbackTrigger)

        close_srv = rospy.ServiceProxy('/CloseGripper', close)
        open_srv = rospy.ServiceProxy('/OpenGripper', Empty) 

        self.state_dim = 4
        self.action_dim = 2
        self.stepSize = 1 

        self.rate = rospy.Rate(15) # 15hz
        while not rospy.is_shutdown():
            rospy.spin()

    def ResetArm(self):
        if not self.drop_srv().dropped:
            return
        while 1:
            if not self.trigger and self.arm_status == 'waiting':
                self.arm_reset_srv()
                print('[collect_data] Waiting for arm to grasp object...')
                rospy.sleep(1.0)
            self.rate.sleep()
            if self.arm_status != 'moving' and self.trigger:
                self.rate.sleep()
                if self.drop_srv().dropped: # Check if really grasped
                    self.trigger = False
                    print('[collect_data] Grasp failed. Restarting')
                    continue
                else:
                    break
        self.trigger = False


    def run_rollout(self, A):
        self.rollout_transition = []
        self.trigger = False
        self.ResetArm()            

        print("[rollout] Rolling-out...")
        
        # Start episode
        success = True
        S = []
        for i in range(A.shape[0]):
            # Get observation and choose action
            state = np.array(self.obs_srv().state)
            action = A[i,:]
            print action

            S.append(state)
            
            suc = True
            state_tmp = state
            for _ in range(self.stepSize):
                suct = self.move_srv(action).success

                next_state = np.array(self.obs_srv().state)
                self.rollout_transition += [(state_tmp, action, next_state, not suct or self.drop_srv().dropped)]
                state_tmp = next_state

                self.rate.sleep()
                if not suct:
                    suc = False

            # Get observation
            next_state = np.array(self.obs_srv().state)

            if suc:
                fail = self.drop_srv().dropped # Check if dropped - end of episode
            else:
                # End episode if overload or angle limits reached
                rospy.logerr('[rollout] Failed to move gripper. Episode declared failed.')
                fail = True

            state = np.copy(next_state)

            if not suc or fail:
                print("[rollout] Fail")
                S.append(state)
                success = False
                break

        file_pi = open('/home/pracsys/catkin_ws/src/hand_control/plans/rollout_output.pkl', 'wb')
        pickle.dump(self.rollout_transition, file_pi)
        file_pi.close()

        print("[rollout] Rollout done.")

        return np.array(S), success

    def callbackTrigger(self, msg):
        self.arm_status = msg.data
        if not self.trigger and self.arm_status == 'finished':
            self.trigger = True

    def CallbackRollout(self, req):
        
        actions = np.array(req.actions).reshape(-1, self.action_dim)
        success = True
        self.states, success = self.run_rollout(actions)

        return {'states': self.states.reshape((-1,)), 'success' : success}

    def CallbackRolloutFile(self, req):

        file_name = req.file

        actions = np.loadtxt(file_name, delimiter=',', dtype=float)[:,:2]
        success = True
        self.states, success = self.run_rollout(actions)

        return {'states': self.states.reshape((-1,)), 'success' : success}

    def Plot(self, req):
        planned = np.array(req.states).reshape(-1, self.state_dim)
        plt.clf()
        plt.plot(self.states[:,0], self.states[:,1],'b', label='Rolled-out path')
        plt.plot(planned[:,0], planned[:,1],'r', label='Planned path')
        # plt.legend()
        if (req.filename):
            plt.savefig(req.filename, bbox_inches='tight')
        else:
            plt.show()

        return EmptyResponse()


if __name__ == '__main__':
    try:
        rollout()
    except rospy.ROSInterruptException:
        pass