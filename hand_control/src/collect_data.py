#!/usr/bin/env python

import rospy
import numpy as np
import time
import random
from std_msgs.msg import String, Float32MultiArray, Bool
from std_srvs.srv import Empty, EmptyResponse
from hand_control.srv import observation, IsDropped, TargetAngles, RegraspObject, close
from transition_experience import *
<<<<<<< HEAD

collect_mode = 'manual' # 'manual' or 'auto'

=======
# from common_msgs_gl.srv import SendBool, SendDoubleArray
import glob
from bowen_pose_estimate.srv import recordHandPose
>>>>>>> d8bdba09fb630f5cb93340f38caebb1b23810fd9

class collect_data():

    gripper_closed = False
    trigger = False # Enable collection
    discrete_actions = True # Discrete or continuous actions

    num_episodes = 0
<<<<<<< HEAD
    episode_length = 10000 # !!!
=======
    episode_length = 1000000 # !!!
>>>>>>> d8bdba09fb630f5cb93340f38caebb1b23810fd9
    desired_action = np.array([0.,0.])
    drop = True

<<<<<<< HEAD
    A = np.array([[1.0,1.0],[-1.,-1.],[-1.0,1.],[1.,-1.0],[1.,0.],[-1.,0.],[0.,-1.],[0.,1.]])
=======
    A = np.array([[1.0,1.0],[-1.,-1.],[-1.,1.],[1.,-1.],[1.5,0.],[-1.5,0.],[0.,-1.5],[0.,1.5]])
>>>>>>> d8bdba09fb630f5cb93340f38caebb1b23810fd9

    texp = transition_experience(Load = True, discrete = discrete_actions)

    def __init__(self):
        rospy.init_node('collect_data', anonymous=True)

        rospy.Subscriber('/gripper/gripper_status', String, self.callbackGripperStatus)
<<<<<<< HEAD
        pub_gripper_action = rospy.Publisher('/gripper_action', Float32MultiArray, queue_size=10)
        rospy.Service('/collect/trigger_episode', Empty, self.callbackTrigger)
        obs_srv = rospy.ServiceProxy('/observation', observation)
        drop_srv = rospy.ServiceProxy('/IsObjDropped', IsDropped)
        move_srv = rospy.ServiceProxy('/MoveGripper', TargetAngles)
        # arm_reset_srv = rospy.ServiceProxy('/RegraspObject', RegraspObject)
=======
        pub_gripper_action = rospy.Publisher('/collect/action', Float32MultiArray, queue_size=10)
        rospy.Service('/collect/trigger_episode', Empty, self.callbackManualTrigger)
        rospy.Service('/collect/save', Empty, self.callbackSave)
        self.recorderSave_srv = rospy.ServiceProxy('/actor/save', Empty)
        obs_srv = rospy.ServiceProxy('/observation', observation)
        drop_srv = rospy.ServiceProxy('/IsObjDropped', IsDropped)
        move_srv = rospy.ServiceProxy('/MoveGripper', TargetAngles)
        rospy.Subscriber('/ObjectIsReset', String, self.callbackTrigger)
        arm_reset_srv = rospy.ServiceProxy('/RegraspObject', RegraspObject)
        record_srv = rospy.ServiceProxy('/record_hand_pose', recordHandPose)
        rospy.Subscriber('/hand_control/drop', Bool, self.callbackObjectDrop)
        recorder_srv = rospy.ServiceProxy('/actor/trigger', Empty)
>>>>>>> d8bdba09fb630f5cb93340f38caebb1b23810fd9

        collect_mode = 'auto' # 'manual' or 'auto' or 'plan'
        
        if collect_mode == 'manual':
            rospy.Subscriber('/keyboard/desired_action', Float32MultiArray, self.callbackDesiredAction)
<<<<<<< HEAD
=======
            ResetKeyboard_srv = rospy.ServiceProxy('/ResetKeyboard', Empty)

        # if collect_mode == 'plan':
        # filest = glob.glob('/home/pracsys/catkin_ws/src/hand_control/plans/*.txt')
        # files = []
        # for f in filest:
        #     if f.find('traj') == -1:
        #         files.append(f)
>>>>>>> d8bdba09fb630f5cb93340f38caebb1b23810fd9
        
        close_srv = rospy.ServiceProxy('/CloseGripper', close)
        open_srv = rospy.ServiceProxy('/OpenGripper', Empty)

        msg = Float32MultiArray()

        msgd = record_srv()
        print msgd.info
        open_srv()
        time.sleep(2.)

        print('[collect_data] Ready to collect...')

        rate = rospy.Rate(2.5) # 15hz
        count_fail = 0
        while not rospy.is_shutdown():

            if self.trigger:
                print('[collect_data] Closeing gripper...')
            
                close_srv()
                rospy.sleep(1.0)
                rate.sleep()
                self.trigger = False

                print('[collect_data] Verifying grasp...')
                if drop_srv().dropped: # Check if really grasped
                    self.trigger = False
                    print('[collect_data] Grasp failed.')
                    continue

                print('[collect_data] Starting episode %d...' % self.num_episodes)

<<<<<<< HEAD
                self.num_episodes += 1
                Done = False

                # Start episode
                for ep_step in range(self.episode_length):
                    # Get observation and choose action
                    state = np.array(obs_srv().state)
                    
                    if collect_mode == 'auto':
                        action = self.choose_action()
                        n = np.random.randint(200)
                    else:
                        action = self.desired_action
                        n = 1
                    
                    for _ in range( n ):
                        msg.data = action
                        pub_gripper_action.publish(msg)
                        suc = move_srv(action).success
                        # rospy.sleep(0.05)
                        rate.sleep()
=======
                if collect_mode != 'manual':
                    if np.random.uniform() > 0.7:
                        collect_mode = 'plan'
                        # files = glob.glob('/home/pracsys/catkin_ws/src/t42_control/hand_control/plans/*.txt')
                        # if len(files)==0:
                        #     collect_mode = 'auto'
                    else:
                        collect_mode = 'auto'

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
                        open_srv()
                        count_fail += 1
                        if count_fail == 60:
                            self.global_trigger = False
                        continue

                    count_fail = 0

                    print('[collect_data] Starting episode %d...' % self.num_episodes)

                    self.num_episodes += 1
                    Done = False

                    if collect_mode == 'plan':
                        # ia = np.random.randint(len(files))
                        # print('[collect_data] Rolling out file: ' + files[ia])
                        # Af = np.loadtxt(files[ia], delimiter = ',', dtype=float)[:,:2]
                        if np.random.uniform() > 0.5:
                            Af = np.tile(np.array([-1.,1.]), (np.random.randint(20,100), 1))
                        else:
                            Af = np.tile(np.array([1.,-1.]), (np.random.randint(20,100), 1))
                        print('[collect_data] Rolling out shooting with %d steps.'%Af.shape[0])
                    
                    # Start episode
                    recorder_srv()
                    n = 0
                    action = np.array([0.,0.])
                    state = np.array(obs_srv().state)
                    for ep_step in range(self.episode_length):

                        if collect_mode == 'plan' and Af.shape[0] == ep_step: # Finished planned path and now applying random actions
                            collect_mode = 'auto'
                            n = 0
                            print('[collect_data] Running random actions...')
                        
                        if n == 0:
                            if collect_mode == 'auto':
                                action, n = self.choose_action()
                            elif collect_mode == 'manual':
                                action = self.desired_action
                                n = 1
                            else: # 'plan'
                                n = 1
                                action = Af[ep_step, :]                            
                        print action, ep_step
                        
                        msg.data = action
                        pub_gripper_action.publish(msg)
                        suc = move_srv(action).success
                        n -= 1
>>>>>>> d8bdba09fb630f5cb93340f38caebb1b23810fd9

                        # Get observation
                        next_state = np.array(obs_srv().state)

                        if suc:
<<<<<<< HEAD
                            fail = drop_srv().dropped # Check if dropped - end of episode
=======
                            fail = False#self.drop # drop_srv().dropped # Check if dropped - end of episode
                            c = 0
                            while self.drop:
                                if c == 3:
                                    fail = True
                                    break
                                c += 1
                                rate.sleep()
>>>>>>> d8bdba09fb630f5cb93340f38caebb1b23810fd9
                        else:
                            # End episode if overload or angle limits reached
                            rospy.logerr('[collect_data] Failed to move gripper. Episode declared failed.')
                            fail = True

<<<<<<< HEAD
                        if not suc or fail:
                            next_state = np.copy(state)

                        self.texp.add(state, action, next_state, not suc or fail)
                        state = next_state
=======
                        self.texp.add(state, action, next_state, not suc or fail)
                        state = np.copy(next_state)
>>>>>>> d8bdba09fb630f5cb93340f38caebb1b23810fd9

                        if not suc or fail:
                            Done = True
                            break
                        
                        rate.sleep()
                  

                    if Done:
                        open_srv()
                        break

                self.trigger = False
                print('[collect_data] Finished running episode %d with total number of collected points: %d' % (self.num_episodes, self.texp.getSize()))
                print('[collect_data] Waiting for next episode initialization...')

<<<<<<< HEAD
                self.texp.save()
                # if self.num_episodes > 0 and not (self.num_episodes % 5):
                #     self.texp.save()
                # if self.num_episodes > 0 and not (self.num_episodes % 10):
                #     self.texp.process_transition_data(stepSize = 10, plot = False)
                #     self.texp.process_svm(stepSize = 10)
=======
                    if self.num_episodes > 0 and not (self.num_episodes % 10):
                        open_srv()
                        self.texp.save()
                        self.recorderSave_srv()
                        if (self.num_episodes % 50 == 0):
                            print('[collect_data] Cooling down.')
                            rospy.sleep(180)
>>>>>>> d8bdba09fb630f5cb93340f38caebb1b23810fd9


    def callbackGripperStatus(self, msg):
        self.gripper_closed = msg.data == "closed"

    def callbackTrigger(self, msg):
            self.trigger = True

<<<<<<< HEAD
    def choose_action(self):
        if self.discrete_actions:
            a = self.A[np.random.randint(self.A.shape[0])]
            if np.random.uniform(0,1,1) > 0.5:
=======
    def callbackManualTrigger(self, msg):
        self.global_trigger = not self.global_trigger

    def callbackObjectDrop(self, msg):
        self.drop = msg.data

    def choose_action(self):
        if self.discrete_actions:
            a = self.A[np.random.randint(self.A.shape[0])]
            if np.random.uniform(0,1,1) > 0.85:
>>>>>>> d8bdba09fb630f5cb93340f38caebb1b23810fd9
                if np.random.uniform(0,1,1) > 0.5:
                    a = self.A[0]
                else:
                    a = self.A[1]
            elif np.random.uniform(0,1,1) > 0.7:
                if np.random.uniform(0,1,1) > 0.5:
                    a = self.A[2]
                else:
                    a = self.A[3]

            n = np.random.randint(60)
            if np.all(a == self.A[0]) or np.all(a == self.A[1]):
                n = np.random.randint(70)
            elif np.random.uniform() > 0.7:
                n = np.random.randint(300)
            else:
                n = np.random.randint(100)
            return a, n
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

<<<<<<< HEAD

=======
    def callbackSave(self, msg):
        self.texp.save()
        self.recorderSave_srv()
>>>>>>> d8bdba09fb630f5cb93340f38caebb1b23810fd9

if __name__ == '__main__':
    
    try:
        collect_data()
    except rospy.ROSInterruptException:
        pass
