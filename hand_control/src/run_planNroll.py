#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse
import numpy as np
import matplotlib.pyplot as plt
import pickle
import time
import glob
from hand_control.srv import RegraspObject, close, observation, planroll
from rollout_t42.srv import rolloutReq


pr_srv = rospy.ServiceProxy('/planNroll', planroll)
pr_proc_srv = rospy.ServiceProxy('/planNroll/process', Empty)
rollout_srv = rospy.ServiceProxy('/rollout/rollout', rolloutReq)

open_srv = rospy.ServiceProxy('/OpenGripper', Empty) 
close_srv = rospy.ServiceProxy('/CloseGripper', close) 


rospy.init_node('run_planNroll', anonymous=True)

#goals = np.array([[1.69,80],[-25,65],[68,74],[-28,95],[100,60],[48,75],[-8,113],[-19,66],[22,118],[-25,65],[-49,67],[92,82]])
# goals = np.array([[68,74],[-28,95],[100,60],[-25,65],[48,75],[-8,113],[-19,66],[22,118],[-25,65],[-49,67],[92,82],[1.69,80]])
goals = np.array([[55,97]])
set_modes = ['robust', 'naive']

msg = planroll()

state_dim = 12

def ResetArm():
    open_srv()
    print('[] Press key to insert object...')
    raw_input()
    rospy.sleep(3.)
    print('[] Waiting to grasp object...')
    close_srv()

if 1:
    for goal in goals:
        for set_mode in set_modes:
            print "Running " + set_mode + " with goal " + str(goal) + "..."
            msg.goal = goal
            msg.planning_algorithm = set_mode

            # First rollout is from the known start state
            res = pr_srv(goal, set_mode)
            File = res.file

            # Now, more runs from approximately the start state
            A = np.loadtxt(File + 'txt', delimiter=',', dtype=float)[:,:2]
            with open(File + 'pkl' ,'r') as f:  
                S = pickle.load(f)

            P = []
            P.append(S)

            Af = A.reshape((-1,))
            for j in range(1):
                print("Rollout number " + str(j) + ".")

                ResetArm()
                
                Sro = np.array(rollout_srv(Af).states).reshape(-1,state_dim)

                P.append(Sro)

                with open(File + 'pkl', 'w') as f: 
                    pickle.dump(P, f)
    pr_proc_srv()
else:
    pr_proc_srv()
