#!/usr/bin/env python

import rospy
import numpy as np
from hand_control.msg import plan_for_goal_request, plan_for_goal_response


class planCall():

    planning_done = False

    path = '/home/pracsys/catkin_ws/src/t42_control/hand_control/plans/'

    goal_counter = 0
    publishToPlanner = True
    goals = np.array([[-40, 70],[-35, 80],[-20, 90],[80, 80],[70, 95],[47, 93],[15, 86],[61, 107],[-30, 96]])

    def __init__(self):
        rospy.init_node('plan_call', anonymous=True)

        self.planning_request_pub = rospy.Publisher('/planning/request', plan_for_goal_request, queue_size=1)
        rospy.Subscriber('/planning/response', plan_for_goal_response, self.planningResponseCallback)

        rate = rospy.Rate(10)
        rospy.sleep(2)
        while not rospy.is_shutdown():
            rate.sleep()
            if (self.publishToPlanner):
                print('[plan_call] Calling planner with goal ' + str(self.goals[self.goal_counter]) + '...')
                self.plan_for_goal(self.goals[self.goal_counter])
            rate.sleep()

            


    def plan_for_goal(self, goal):
        
        msg = plan_for_goal_request()

        msg.goal_state = goal
        msg.goal_radius = 10.0
        msg.time_limit = 600 #seconds
        msg.probability_success_threshold = 0.5 #affects SVM validity check 

        print('Publishing...')
        self.planning_request_pub.publish(msg)

        self.planning_done = False
        self.publishToPlanner = False


    def planningResponseCallback(self, msg):
        self.planning_actions = msg.planned_actions
        self.planned_path = msg.planned_path
        self.planning_succeeded = msg.reached_goal

        self.planning_done = True


        self.planning_actions = np.array(self.planning_actions).reshape((-1,2))

        File = 'path_' + str(np.random.randint(1000)) + '.txt'
        np.savetxt(self.path + File, self.planning_actions, delimiter = ', ')

        print('[plan_call] Planned received and saved in ' + File)
        self.goal_counter += 1
        self.publishToPlanner = True
        

if __name__ == '__main__':
    
    try:
        planCall()
    except rospy.ROSInterruptException:
        pass