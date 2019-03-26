#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse, SetBool
from std_msgs.msg import Bool, String, Float32MultiArray
import numpy as np
import matplotlib.pyplot as plt
import pickle
from rollout_node.srv import observation, IsDropped, TargetAngles
from hand_control.srv import RegraspObject, close, observation, planroll
from rollout_t42.srv import gets
from gpup_gp_node.srv import one_transition
from cl_control.srv import pathTrackReq

import sys
sys.path.insert(0, '/home/pracsys/catkin_ws/src/beliefspaceplanning/gpup_gp_node/src/')
import var

class general_control():

    drop = True
    obj_pos = np.array([0., 0.])
    gripper_load = np.array([0., 0.])
    actionGP = np.array([0., 0.])
    actionVS = np.array([0., 0.])
    tol = 1.5
    goal_tol = 10.0
    horizon = 1
    arm_status = ' '
    trigger = False # Enable collection

    def __init__(self):
        rospy.init_node('cl_control', anonymous=True)

        self.gp = rospy.ServiceProxy('/gp/transitionOneParticle', one_transition)

        rospy.Subscriber('/hand_control/obj_pos_mm', Float32MultiArray, self.callbackObj)
        rospy.Subscriber('/gripper/load', Float32MultiArray, self.callbackGripperLoad)
        rospy.Subscriber('/cylinder_drop', Bool, self.callbackDrop)

        rospy.Service('/control', pathTrackReq, self.CallbackTrack)
        rospy.Subscriber('/gp_controller/action', Float32MultiArray, self.CallbackBestActionGP)
        rospy.Subscriber('/vs_controller/action', Float32MultiArray, self.CallbackBestActionVS)
        self.pub_current_goal = rospy.Publisher('/control/goal', Float32MultiArray, queue_size=10)
        self.pub_horizon = rospy.Publisher('/control/horizon', Float32MultiArray, queue_size=10)
        self.pub_exclude = rospy.Publisher('/control/exclude', Float32MultiArray, queue_size=10)

        self.obs_srv = rospy.ServiceProxy('/observation', observation)
        self.move_srv = rospy.ServiceProxy('/MoveGripper', TargetAngles)
        self.arm_reset_srv = rospy.ServiceProxy('/RegraspObject', RegraspObject)
        rospy.Subscriber('/ObjectIsReset', String, self.callbackTrigger)

        self.trigger_srv = rospy.ServiceProxy('/rollout/record_trigger', SetBool)
        self.gets_srv = rospy.ServiceProxy('/rollout/get_states', gets)

        self.plot_clear_srv = rospy.ServiceProxy('/plot/clear', Empty)
        self.plot_ref_srv = rospy.ServiceProxy('/plot/ref', pathTrackReq)

        self.state_dim = 4
        self.action_dim = 6
        self.stepSize = 1

        print("[control] Ready to track path...")
        self.rate = rospy.Rate(2) 
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
                if self.drop: # Check if really grasped
                    self.trigger = False
                    print('[rollout_action_publisher] Grasp failed. Restarting')
                    continue
                else:
                    break
        self.trigger = False

    def callbackTrigger(self, msg):
        self.arm_status = msg.data
        if not self.trigger and self.arm_status == 'finished':
            self.trigger = True

    def CallbackTrack(self, req):

        path = np.array(req.desired_path).reshape(-1, self.state_dim)
        
        real_path, actions, success = self.run_tracking(path)

        return {'real_path': real_path, 'actions': actions, 'success' : success}

    def weightedL2(self, ds, W = np.diag(np.array([1.,1.,0.2, 0.2]))):
        # return np.sqrt( np.dot(ds.T, np.dot(W, ds)) )
        return np.linalg.norm(ds[:2])

    def run_tracking(self, S):
        
        # Reset gripper
        self.trigger = False
        self.ResetArm()
        rospy.sleep(2.)

        i_path = 1 #S.shape[0]-1#
        msg = Float32MultiArray()
        msg.data = S[i_path,:]
        msge = Float32MultiArray()
        for i in range(5):
            self.pub_current_goal.publish(msg)
            self.rate.sleep()

        rospy.sleep(1.0)
        
        self.plot_clear_srv()
        
        self.plot_ref_srv(S.reshape((-1,)))
        self.trigger_srv(True)
        n = -1
        count = 0
        total_count = 0
        d_prev = 1000
        action = np.array([0.,0.])
        dd_count = 0
        Controller = 'GP'
        
        print("[control] Tracking path...")
        while 1:
            change = False
            state = np.concatenate((self.obj_pos, self.gripper_load), axis=0)
            if i_path == S.shape[0]-1:
                msg.data = S[-1,:]
            elif self.weightedL2(state[:]-S[i_path,:]) < self.tol or (self.weightedL2(state[:]-S[i_path+1,:]) < self.weightedL2(state[:]-S[i_path,:]) and self.weightedL2(state[:]-S[i_path+1,:])):# < self.tol*3):
                i_path += 1
                msg.data = S[i_path,:]
                count = 0
                change = True
                self.tol = 1.5
                dd_count = 0
                Controller = 'GP'
            elif count > 100:# and i_path < S.shape[0]-1:
                self.tol = 2.5
                Controller == 'VS'
            self.pub_current_goal.publish(msg)

            dd = self.weightedL2(state[:]-S[i_path,:]) - d_prev
            dd_count = dd_count + 1 if dd > 0 else 0
            msge.data = action if dd_count > 3 else np.array([0.,0.])
            self.pub_exclude.publish(msge)

            # if n == 0 and not change and dd < 0:
            #     n = 1
            #     print "Extended..."
            if n <= 0:# or dd_count > 5:
                if 0:#Controller == 'GP':
                    action = self.actionGP
                else:
                    action = self.actionVS
                n = self.stepSize
                dd_count = 0

            print total_count, count, i_path, action, self.weightedL2(state[:]-S[i_path,:]), self.weightedL2(state[:]-S[-1,:]), self.weightedL2(state[:]-S[i_path,:]) - d_prev
            
            d_prev =  self.weightedL2(state[:]-S[i_path,:])

            suc = self.move_srv(action).success
            n -= 1

            if not suc or self.drop or count > 1000:
                print("[control] Fail.")
                success = False
                break

            if np.linalg.norm(state[:2]-S[-1,:2]) < self.goal_tol:
                print("[control] Reached GOAL!!!")
                success = True
                break

            # if total_count > 100:
            #     success = True
            #     break

            count += 1
            total_count += 1
            self.rate.sleep()

        Sreal = self.gets_srv().states
        Areal = self.gets_srv().actions

        return Sreal, Areal, success

    def callbackDrop(self, msg):
        self.drop = msg.data

    def callbackObj(self, msg):
        Obj_pos = np.array(msg.data)
        self.obj_pos = Obj_pos[:2] * 1000

    def callbackGripperLoad(self, msg):
        self.gripper_load = np.array(msg.data)

    def CallbackBestActionGP(self, msg):
        self.actionGP = np.array(msg.data)

    def CallbackBestActionVS(self, msg):
        self.actionVS = np.array(msg.data)


if __name__ == '__main__':
    try:
        general_control()
    except rospy.ROSInterruptException:
        pass