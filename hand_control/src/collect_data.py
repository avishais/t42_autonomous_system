#!/usr/bin/env python

import rospy
import numpy as np
import time
import random
from std_msgs.msg import String, Float32MultiArray
from std_srvs.srv import Empty, EmptyResponse
from hand_control.srv import observation, IsDropped, TargetAngles, RegraspObject, close
from transition_experience import *
# from common_msgs_gl.srv import SendBool, SendDoubleArray
import glob
from bowen_pose_estimate.srv import recordHandPose

collect_mode = 'auto' # 'manual' or 'auto' or 'plan'

class collect_data():

    gripper_closed = False
    trigger = False # Enable collection
    discrete_actions = True # Discrete or continuous actions
    arm_status = ' '
    global_trigger = True

    num_episodes = 0
    episode_length = 100000 # !!!
    desired_action = np.array([0.,0.])

    A = np.array([[1.0,1.0],[-1.,-1.],[-1.,1.],[1.,-1.],[1.5,0.],[-1.5,0.],[0.,-1.5],[0.,1.5]])

    texp = transition_experience(Load = True, discrete = discrete_actions)

    def __init__(self):
        rospy.init_node('collect_data', anonymous=True)

        rospy.Subscriber('/gripper/gripper_status', String, self.callbackGripperStatus)
        pub_gripper_action = rospy.Publisher('/gripper_action', Float32MultiArray, queue_size=10)
        rospy.Service('/collect/trigger_episode', Empty, self.callbackManualTrigger)
        obs_srv = rospy.ServiceProxy('/observation', observation)
        drop_srv = rospy.ServiceProxy('/IsObjDropped', IsDropped)
        move_srv = rospy.ServiceProxy('/MoveGripper', TargetAngles)
        rospy.Subscriber('/ObjectIsReset', String, self.callbackTrigger)
        arm_reset_srv = rospy.ServiceProxy('/RegraspObject', RegraspObject)
        record_srv = rospy.ServiceProxy('/record_hand_pose', recordHandPose)
        # allow_motion_srv = rospy.ServiceProxy('/gripper_t42/allow_motion', SendBool)
        # vel_ref_srv = rospy.ServiceProxy('/gripper_t42/vel_ref', SendDoubleArray)
        # reset_motor_pos_srv = rospy.ServiceProxy('/gripper_t42/reset_motor_pos_ref', Empty)

        if collect_mode == 'manual':
            rospy.Subscriber('/keyboard/desired_action', Float32MultiArray, self.callbackDesiredAction)
            ResetKeyboard_srv = rospy.ServiceProxy('/ResetKeyboard', Empty)

        if collect_mode == 'plan':
            filest = glob.glob('/home/pracsys/catkin_ws/src/hand_control/plans/*.txt')
            files = []
            for f in filest:
                if f.find('traj') == -1:
                    files.append(f)
        
        close_srv = rospy.ServiceProxy('/CloseGripper', close)
        open_srv = rospy.ServiceProxy('/OpenGripper', Empty) 

        msg = Float32MultiArray()

        msgd = record_srv()
        print msgd.info
        open_srv()
        time.sleep(2.)

        print('[collect_data] Ready to collect...')

        rate = rospy.Rate(15) # 15hz
        count_fail = 0
        while not rospy.is_shutdown():

            if self.global_trigger:

                if not self.trigger and self.arm_status == 'waiting':
                    
                    if collect_mode == 'manual': 
                        ResetKeyboard_srv()
                    if 1:#drop_srv().dropped:
                        arm_reset_srv()
                        print('[collect_data] Waiting for arm to grasp object...')
                        time.sleep(1.0)
                    else:
                        self.trigger = True
                
                if self.arm_status != 'moving' and self.trigger:

                    print('[collect_data] Verifying grasp...')
                    if drop_srv().dropped: # Check if really grasped
                        self.trigger = False
                        print('[collect_data] Grasp failed. Restarting')
                        count_fail += 1
                        if count_fail == 60:
                            self.global_trigger = False
                        continue

                    count_fail = 0

                    print('[collect_data] Starting episode %d...' % self.num_episodes)

                    self.num_episodes += 1
                    Done = False

                    if collect_mode == 'plan':
                        ia = np.random.randint(len(files))
                        print('[rollout] Rolling out file: ' + files[ia])
                        A = np.loadtxt(files[ia], delimiter = ',', dtype=float)[:,:2]
                        self.episode_length = A.shape[0]

                    # Start episode
                    for ep_step in range(self.episode_length):
                        # Get observation and choose action
                        state = np.array(obs_srv().state)
                        
                        if collect_mode == 'auto':
                            action = self.choose_action()
                            if ep_step==0:#np.random.uniform() > 0.7:
                                n = np.random.randint(100)
                            else:
                                if np.random.uniform() > 0.6:
                                    n = np.random.randint(150)
                                else:
                                    n = np.random.randint(40)
                        elif collect_mode == 'manual':
                            action = self.desired_action
                            n = 1
                        else:
                            n = 1
                            action = A[ep_step, :]                            
                        print action
                        
                        for _ in range( n ):
                            tr = rospy.get_time()
                            
                            # msg.data = action
                            # pub_gripper_action.publish(msg)
                            suc = move_srv(action).success
                            # rospy.sleep(0.05)
                            rate.sleep()

                            # Get observation
                            next_state = np.array(obs_srv().state)

                            if suc:
                                fail = drop_srv().dropped # Check if dropped - end of episode
                            else:
                                # End episode if overload or angle limits reached
                                rospy.logerr('[collect_data] Failed to move gripper. Episode declared failed.')
                                fail = True

                            if not suc or fail:
                                next_state = np.copy(state)

                            self.texp.add(state, action, next_state, not suc or fail, rospy.get_time()-tr)
                            state = np.copy(next_state)

                            if not suc or fail:
                                Done = True
                                break
                        if Done:
                            open_srv()
                            break

                    open_srv()

                    self.trigger = False
                    print('[collect_data] Finished running episode %d with total number of collected points: %d' % (self.num_episodes, self.texp.getSize()))
                    print('[collect_data] Waiting for next episode initialization...')

                    self.texp.save()
                    # if self.num_episodes > 0 and not (self.num_episodes % 3):
                    #     self.texp.save()
                    # if self.num_episodes > 0 and not (self.num_episodes % 10):
                    #     self.texp.process_transition_data(stepSize = 10, plot = False)
                    #     self.texp.process_svm(stepSize = 10)

            rate.sleep()

    def callbackGripperStatus(self, msg):
        self.gripper_closed = msg.data == "closed"

    def callbackTrigger(self, msg):
        self.arm_status = msg.data
        if not self.trigger and self.arm_status == 'finished':
            self.trigger = True

    def callbackManualTrigger(self, msg):
        self.global_trigger = not self.global_trigger

    def choose_action(self):
        if self.discrete_actions:
            a = self.A[np.random.randint(self.A.shape[0])]
            if np.random.uniform(0,1,1) > 0.7:
                if np.random.uniform(0,1,1) > 0.5:
                    a = self.A[0]
                else:
                    a = self.A[1]
        else:
            a = np.random.uniform(-1.,1.,2)
            if np.random.uniform(0,1,1) > 0.35:
                if np.random.uniform(0,1,1) > 0.5:
                    a[0] = np.random.uniform(-1.,-0.8,1)
                    a[1] = np.random.uniform(-1.,-0.8,1)
                else:
                    a[0] = np.random.uniform(0.8,1.,1)
                    a[1] = np.random.uniform(0.8,1.,1)

        return a

    def callbackDesiredAction(self, msg):
        self.desired_action = msg.data

if __name__ == '__main__':
    
    try:
        collect_data()
    except rospy.ROSInterruptException:
        pass
