#!/usr/bin/env python


import rospy
from std_srvs.srv import Empty, EmptyResponse
import numpy as np
import matplotlib.pyplot as plt
import pickle
import time
import glob
from hand_control.srv import RegraspObject, close, observation, planroll, TargetAngles
from rollout_t42.srv import rolloutReq
from std_msgs.msg import String, Float32MultiArray, Bool

from gpup_gp_node_exp.srv import gpup_transition, batch_transition, one_transition
gp_srv = rospy.ServiceProxy('/gp/transition', batch_transition)
naive_srv = rospy.ServiceProxy('/gp/transitionOneParticle', one_transition)

rospy.init_node('test', anonymous=True)


pklfile = '/home/pracsys/catkin_ws/src/t42_control/gpup_gp_node/data/dataset_processed/t42_cyl45_right_test_paths.obj'

with open(pklfile) as f:  
    Pro, Aro = pickle.load(f)

Straj = Pro[0]
A = Aro[0]

s_start = Straj[0,:]
state_dim = 4#S.shape[1]

print s_start
s = np.copy(s_start)# + np.random.normal(0, sigma_start)
Ypred = s.reshape(1,state_dim)

Sp = np.tile(s, (100,1))

print("Running (open loop) path...")
p_naive = 1
for i in range(0, A.shape[0]):
    print("[] Step " + str(i) + " of " + str(A.shape[0]))
    a = A[i,:]
    print s, a

    res = naive_srv(s.reshape(-1,1), a)
    s_next = np.array(res.next_state)
    s = np.copy(s_next)

    # res = gp_srv(Sp.reshape(-1,1), a)
    # S_next = np.array(res.next_states).reshape(-1, state_dim)
    # s_next = np.mean(S_next, 0)
    # Sp = np.copy(S_next)
    # s = np.copy(s_next)

    Ypred = np.append(Ypred, s_next.reshape(1, state_dim), axis=0)

# h = [40, 40, 100, 100]
# for i in range(state_dim):
#     try:
#         S[:,i] = medfilter(S[:,i], h[i])
#     except:
#         S[:,i] = medfilter(S[:,i], 40)

# plt.plot(S[:,0], S[:,1], '.-k', label='rollout')
plt.plot(Straj[:,0], Straj[:,1], '.-r', label='Plan')
plt.plot(Ypred[:,0], Ypred[:,1], '.-c', label='Naive')

plt.show()