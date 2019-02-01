#!/usr/bin/python 

'''
Author: Avishai Sintov
'''

import rospy
import numpy as np 
from std_msgs.msg import Float64MultiArray, Float32MultiArray, String
from std_srvs.srv import Empty, EmptyResponse
from openhand.srv import MoveServos
from hand_control.srv import TargetAngles, IsDropped, observation, close
# from common_msgs_gl.srv import SendDoubleArray, SendBool
import geometry_msgs.msg
import math

class hand_control():

    finger_initial_offset = np.array([0., 0.])
    finger_opening_position = np.array([0.2, 0.33])
    finger_closing_position = np.array([0., 0.])
    finger_move_offset = np.array([0.01, 0.01])
    closed_load = np.array(20.)

    gripper_pos = np.array([0., 0.])
    gripper_load = np.array([0., 0.])
    base_pos = [0,0]
    base_theta = 0
    obj_pos = [0,0]
    obj_height = -1.0e3
    obj_grasped_height = 1.0e3
    R = []
    count = 1
    vel_ref = np.array([0.,0.])
    gripper_cur_pos = np.array([0.,0.])
    max_load = 250.0

    gripper_status = 'open'
    object_grasped = False

    move_servos_srv = 0.

    def __init__(self):
        rospy.init_node('hand_control', anonymous=True)
        
        if rospy.has_param('~finger_initial_offset'):
            self.finger_initial_offset = rospy.get_param('~finger_initial_offset')
            self.finger_opening_position = rospy.get_param('~finger_opening_position')
            self.finger_closing_position = rospy.get_param('~finger_closing_position')
            self.finger_move_offset = rospy.get_param('~finger_move_offset')
            self.closed_load = rospy.get_param('~finger_close_load')

        rospy.Subscriber('/gripper/pos', Float32MultiArray, self.callbackGripperPos)
        rospy.Subscriber('/gripper/load', Float32MultiArray, self.callbackGripperLoad)
        rospy.Subscriber('/cylinder_pose', geometry_msgs.msg.Pose, self.callbackMarkers)
        pub_gripper_status = rospy.Publisher('/gripper/gripper_status', String, queue_size=10)

        # vel_ref_srv = rospy.ServiceProxy('/gripper_t42/vel_ref', SendDoubleArray)
        # self.allow_motion_srv = rospy.ServiceProxy('/gripper_t42/allow_motion', SendBool)

        rospy.Service('/OpenGripper', Empty, self.OpenGripper)
        rospy.Service('/CloseGripper', close, self.CloseGripper)
        rospy.Service('/MoveGripper', TargetAngles, self.MoveGripper)
        rospy.Service('/IsObjDropped', IsDropped, self.CheckDropped)
        rospy.Service('/observation', observation, self.GetObservation)

        self.move_servos_srv = rospy.ServiceProxy('/MoveServos', MoveServos)

        #### Later I should remove the angles from hands.py and set initial angles here at the start ####

        self.rate = rospy.Rate(100)
        c = True
        while not rospy.is_shutdown():
            pub_gripper_status.publish(self.gripper_status)

            if c and not np.all(self.gripper_load==0): # Wait till openhand services ready and set gripper open pose
                self.moveGripper(self.finger_opening_position)
                c = False

            # if self.object_grasped:
            #     vel_ref_srv(self.vel_ref)

            # rospy.spin()
            self.rate.sleep()

    def callbackGripperPos(self, msg):
        self.gripper_pos = np.array(msg.data)

    def callbackGripperLoad(self, msg):
        self.gripper_load = np.array(msg.data)

    def callbackMarkers(self, msg):
        try:
            if np.abs(msg.position.x) < 0.2 and msg.position.y < 0.12 and msg.position.y > -0.5:
                self.obj_pos = np.array([msg.position.x, msg.position.y])
            self.obj_height = msg.position.z
        except:
            self.obj_pos = np.array([np.nan, np.nan])
            self.obj_height = np.nan

    def OpenGripper(self, msg):
        # self.vel_ref = np.array([0.,0.,])
        # self.allow_motion_srv(False)
        self.moveGripper(self.finger_opening_position)

        self.gripper_status = 'open'

        return EmptyResponse()

    def CloseGripper(self, msg):
        # self.vel_ref = np.array([0.,0.,])

        self.object_grasped = False
        for i in range(100):
            # print('Angles: ' + str(self.gripper_pos) + ', load: ' + str(self.gripper_load), self.closed_load)
            if abs(self.gripper_load[0]) > self.closed_load or abs(self.gripper_load[1]) > self.closed_load:
                rospy.loginfo('[hand] Object grasped.')
                self.gripper_status = 'closed'
                break

            desired = self.gripper_pos + np.array([ a*4.0 for a in self.finger_move_offset])/18. #self.finger_move_offset/2.0
            if desired[0] > 0.7 or desired[1] > 0.7:
                rospy.logerr('[hand] Desired angles out of bounds.')
                break
            print self.gripper_pos, desired
            self.moveGripper(desired)
            rospy.sleep(0.2)  

        self.rate.sleep()
        ## Verify grasp based on height - not useful if camera cannot see
        #print('[hand] Object height relative to gripper marker: %f'%self.obj_height)
        #if abs(self.obj_height) < 7.0e-2:
        #    self.object_grasped = True
        #    self.obj_grasped_height = self.obj_height

        ## Verify based on gripper motor angles
        print('[hand] Gripper actuator angles: ' + str(self.gripper_pos))
        # if self.gripper_pos[0] < 0.357 and self.gripper_pos[1] < 0.377:
        self.object_grasped = True
        #self.allow_motion_srv(True)
        self.obj_grasped_height = self.obj_height # This will have to be defined in hand base pose

        self.rate.sleep()
        self.gripper_cur_pos = self.gripper_pos

        return {'success': self.object_grasped}


    def MoveGripper(self, msg):
        # This function should accept a vector of normalized incraments to the current angles: msg.angles = [dq1, dq2], where dq1 and dq2 can be equal to 0 (no move), 1,-1 (increase or decrease angles by finger_move_offset)
        f = 100.0

        inc = np.array(msg.angles)
        inc_angles = np.multiply(self.finger_move_offset, inc)
        # suc = True

        self.gripper_cur_pos += inc_angles*1.0/f
        suc = self.moveGripper(self.gripper_cur_pos)

        return {'success': suc}
    
    def moveGripper(self, angles):
        if angles[0] > 0.9 or angles[1] > 0.9 or angles[0] < 0.05 or angles[1] < 0.05:
            rospy.logerr('[hand] Desired angles out of bounds.')
            return False

        if abs(self.gripper_load[0]) > self.max_load or abs(self.gripper_load[1]) > self.max_load:
            rospy.logerr('[hand] Pre-overload.')
            return False

        self.move_servos_srv.call(angles)

        return True

    def CheckDropped(self, msg):
        # Should spin (update topics) between moveGripper and this

        if self.gripper_pos[0] > 0.9 or self.gripper_pos[1] > 0.9 or self.gripper_pos[0] < 0.05 or self.gripper_pos[1] < 0.05:
            rospy.logerr('[hand] Desired angles out of bounds.')
            return {'dropped': True}

        # Check load
        if abs(self.gripper_load[0]) > self.max_load or abs(self.gripper_load[1]) > self.max_load:
            rospy.logerr('[hand] Pre-overload.')
            return {'dropped': True}

        # If object marker not visible, loop to verify and declare dropped.
        
        if self.obj_height == -1000:
            dr = True
            for _ in range(15):
                self.rate.sleep()
                if self.obj_height != -1000:
                    dr = False
                    break
            if dr:
                rospy.loginfo('[hand] Object not visible - assumed dropped.')
                return {'dropped': True}
        
        # If marker is visible but far from the height of the hand, loop to verify and declare dropped
        '''if abs(self.obj_height) > 0.05:
            dr = True
            for _ in range(10):
                rospy.sleep(0.05)
                self.rate.sleep()
                print("*", self.obj_height)
                if abs(self.obj_height) < 0.06:
                    dr = False
                    break
            if dr:
                rospy.loginfo('[hand] Object dropped.')
                print(self.obj_height)
                self.object_grasped = False
                return {'dropped': True}'''

        return {'dropped': False}

    def GetObservation(self, msg):
        obs = np.concatenate((self.obj_pos, self.gripper_load))

        return {'state': obs}



if __name__ == '__main__':
    
    try:
        hand_control()
    except rospy.ROSInterruptException:
        pass
