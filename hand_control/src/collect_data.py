#!/usr/bin/env python

import rospy
import numpy as np
import time
import random
from std_msgs.msg import String, Float32MultiArray
from std_srvs.srv import Empty, EmptyResponse
from hand_control.srv import observation, IsDropped, TargetAngles, RegraspObject, close
from transition_experience import *

collect_mode = 'manual' # 'manual' or 'auto'


class collect_data():

    gripper_closed = False
    trigger = False # Enable collection
    discrete_actions = True # Discrete or continuous actions

    num_episodes = 0
    episode_length = 10000 # !!!
    desired_action = np.array([0.,0.])

    A = np.array([[1.0,1.0],[-1.,-1.],[-1.0,1.],[1.,-1.0],[1.,0.],[-1.,0.],[0.,-1.],[0.,1.]])

    texp = transition_experience(Load = True, discrete = discrete_actions)

    def __init__(self):
        rospy.init_node('collect_data', anonymous=True)

        rospy.Subscriber('/gripper/gripper_status', String, self.callbackGripperStatus)
        pub_gripper_action = rospy.Publisher('/gripper_action', Float32MultiArray, queue_size=10)
        rospy.Service('/collect/trigger_episode', Empty, self.callbackTrigger)
        obs_srv = rospy.ServiceProxy('/observation', observation)
        drop_srv = rospy.ServiceProxy('/IsObjDropped', IsDropped)
        move_srv = rospy.ServiceProxy('/MoveGripper', TargetAngles)
        # arm_reset_srv = rospy.ServiceProxy('/RegraspObject', RegraspObject)

        if collect_mode == 'manual':
            rospy.Subscriber('/keyboard/desired_action', Float32MultiArray, self.callbackDesiredAction)
        
        close_srv = rospy.ServiceProxy('/CloseGripper', close)
        open_srv = rospy.ServiceProxy('/OpenGripper', Empty)

        msg = Float32MultiArray()

        open_srv()

        print('[collect_data] Ready to collect...')

        rate = rospy.Rate(15) # 15hz
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

                        self.texp.add(state, action, next_state, not suc or fail)
                        state = next_state

                        if not suc or fail:
                            Done = True
                            break

                    if Done:
                        open_srv()
                        break

                self.trigger = False
                print('[collect_data] Finished running episode %d with total number of collected points: %d' % (self.num_episodes, self.texp.getSize()))
                print('[collect_data] Waiting for next episode initialization...')

                self.texp.save()
                # if self.num_episodes > 0 and not (self.num_episodes % 5):
                #     self.texp.save()
                # if self.num_episodes > 0 and not (self.num_episodes % 10):
                #     self.texp.process_transition_data(stepSize = 10, plot = False)
                #     self.texp.process_svm(stepSize = 10)

            rate.sleep()

    def callbackGripperStatus(self, msg):
        self.gripper_closed = msg.data == "closed"

    def callbackTrigger(self, msg):
            self.trigger = True

    def choose_action(self):
        if self.discrete_actions:
            a = self.A[np.random.randint(self.A.shape[0])]
            if np.random.uniform(0,1,1) > 0.5:
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
