#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse
import numpy as np
import matplotlib.pyplot as plt
import pickle
import time
import glob
from hand_control.srv import RegraspObject, close, observation, planroll


pr_srv = rospy.ServiceProxy('/planNroll', planroll)
pr_proc_srv = rospy.ServiceProxy('/planNroll/process', Empty)

rospy.init_node('run_planNroll', anonymous=True)

#goals = np.array([[1.69,80],[-25,65],[68,74],[-28,95],[100,60],[48,75],[-8,113],[-19,66],[22,118],[-25,65],[-49,67],[92,82]])
goals = np.array([[68,74],[-28,95],[100,60],[-25,65],[48,75],[-8,113],[-19,66],[22,118],[-25,65],[-49,67],[92,82],[1.69,80]])
set_modes = ['robust', 'naive']

msg = planroll()

for i in range(10):
    for goal in goals:
        for set_mode in set_modes:
            print "Running " + set_mode + " with goal " + str(goal) + ", iteration " + str(i+1) + "..."
            msg.goal = goal
            msg.planning_algorithm = set_mode

            pr_srv(goal, set_mode)

pr_proc_srv()
