#!/usr/bin/env python

import rospy
from cl_control.srv import pathTrackReq
from std_srvs.srv import Empty, EmptyResponse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Ellipse, Polygon
import pickle
import time

# np.random.seed(10)

def medfilter(X, W):
    w = int(W/2)

    for k in range(X.shape[1]):
        x = np.copy(X[:,k])
        x_new = np.copy(x)
        for i in range(1, x.shape[0]-1):
            if i < w:
                x_new[i] = np.mean(x[:i+w])
            elif i > x.shape[0]-w:
                x_new[i] = np.mean(x[i-w:])
            else:
                x_new[i] = np.mean(x[i-w:i+w])
        X[:,k] = np.copy(x_new)
    return X

def tracking_error(S1, S2):
    Sum = 0.
    for s1, s2 in zip(S1, S2):
        Sum += np.linalg.norm(s1[:2]-s2[:2])**2

    l = 0.
    for i in range(1,S1.shape[0]):
        l += np.linalg.norm(S1[i,:2] - S1[i-1,:2])

    return np.sqrt(Sum / S1.shape[0]), l

track_srv = rospy.ServiceProxy('/control', pathTrackReq)

Obj = 'cyl35'
path = '/home/pracsys/catkin_ws/src/t42_control/cl_control/results/'

test_path = '/home/pracsys/catkin_ws/src/t42_control/hand_control/data/dataset/'
with open(test_path + 'testpaths_' + Obj + '_d_v' + str(1) + '.pkl', 'r') as f: 
    action_seq, test_paths, Obj, _ = pickle.load(f)

if 0:
    j = 0
    Pro = []
    Aro = []
    Suc = []
    I = []
    for S in test_paths:
        print("Rolling out path number %d..."%j)
        S = S[:,:4]

        P = []
        for k in range(10):
            print('Trial %d.'%k)
            res = track_srv(S.reshape((-1,)))
            Sreal = np.array(res.real_path).reshape(-1, 13)
            Areal = np.array(res.actions).reshape(-1, 2)
            success = res.success
            i_path = res.i_path
            P += [(j, Sreal, Areal, success, i_path, S)]
            print('Reached point %d out of %d.'%(i_path, S.shape[0]))
            
        Pro.append(P)

        with open(path + 'control_path_' + Obj + '_d_v' + str(1) + '.pkl', 'w') as f: 
            pickle.dump(Pro, f)
        j += 1
else:
    with open(path + 'control_path_' + Obj + '_d_v' + str(1) + '.pkl', 'r') as f: 
        Pro = pickle.load(f)


## Plot ##

plt.figure(1)

for P in Pro:
    # j = np.array([item[0] for item in P])
    Sreal = [item[1] for item in P]
    Sref = [item[5] for item in P]
    I = [item[4] for item in P]
    
    for sreal, sref in zip(Sreal, Sref):
        sreal = medfilter(sreal, 10)
        ds = sreal[0,:2] - sref[0,:2]
        sref[:,:2] += ds
        plt.plot(sref[:,0],sref[:,1],'--k')
        plt.plot(sreal[:,0],sreal[:,1],'-r')
    plt.show()


# for S, Sref in zip(Pro, test_paths):
#     ds = S[0,:2] - Sref[0,:2]
#     Sref[:,:2] += ds
    
#     S = medfilter(S, 10)

#     plt.plot(Sref[:,0],Sref[:,1],'--k', label='reference')
#     plt.plot(S[:,0],S[:,1],'-r', label='Rolled-out')

#     plt.legend()
#     plt.axis('equal')
#     # plt.savefig(path + File, dpi=300)
plt.show()


