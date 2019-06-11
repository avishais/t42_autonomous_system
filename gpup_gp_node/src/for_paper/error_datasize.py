#!/usr/bin/env python

import rospy
from gpup_gp_node_exp.srv import gpup_transition, batch_transition, one_transition, setk
from std_srvs.srv import Empty, EmptyResponse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Ellipse, Polygon
import pickle
from nn_predict.srv import StateAction2State
import time

# np.random.seed(10)

version = 0
Obj = 'cyl35'
if np.any(Obj == np.array(['sqr30','poly10','poly6','elp40','str40'])):
    state_dim = 5
else:
    state_dim = 4

naive_srv = rospy.ServiceProxy('/gp/transitionOneParticle', one_transition)
nn_srv = rospy.ServiceProxy('/nn/predict', StateAction2State)
setKD_srv = rospy.ServiceProxy('/gp/set_new_kdtree', setk)
rospy.init_node('error_analysis_t42', anonymous=True)

path = '/home/pracsys/catkin_ws/src/t42_control/gpup_gp_node/src/for_paper/results/'
test_path = '/home/pracsys/catkin_ws/src/t42_control/hand_control/data/dataset/'

def medfilter(x, W):
    w = int(W/2)
    x_new = np.copy(x)
    for i in range(0, x.shape[0]):
        if i < w:
            continue
            # x_new[i] = np.mean(x[:i+w])
        elif i > x.shape[0]-w:
            continue
            # x_new[i] = np.mean(x[i-w:])
        else:
            x_new[i] = np.mean(x[i-w:i+w])
    return x_new

w = [40, 40, 100, 100]

## GP
if 1:
    with open(test_path + 'testpaths_' + Obj + '_d_v' + str(version) + '.pkl', 'r') as f: 
        action_seq, test_paths, Obj, Suc = pickle.load(f)

    Ggp = []

    Ld = range(1000, 250000,2500)
    n = 100
    for l in Ld:
        print("Run %d, number of samples %d."%(l, len(Ggp)))
        setKD_srv(np.array([l]))
        e = 0.0
        for k in range(n):
            path_inx = np.random.randint(len(test_paths))
            R = test_paths[path_inx]
            A = action_seq[path_inx]
            if state_dim == 5:
                R = R[:,[0,1,11,12,2]]
            else:
                R = R[:,[0,1,11,12]]
            A = np.concatenate((A, np.tile(R[0,:], (A.shape[0], 1))), axis=1)
            st_inx = np.random.randint(R.shape[0]-2)
            s = R[st_inx]
            s_next = R[st_inx+1]
            a = A[st_inx]

            res = naive_srv(s.reshape(-1,1), a)
            s_next_pred = np.array(res.next_state)

            e += np.linalg.norm(s_next_pred[:2]-s_next[:2])**2

        Ggp.append(np.sqrt(e / n))
        
        with open(path + 'datasize_analysis_' + Obj + '_gp.pkl', 'w') as f: 
            pickle.dump([Ld, Ggp], f)

    Ggp = np.array(Ggp)

    with open(path + 'datasize_analysis_' + Obj + '_gp.pkl', 'w') as f: 
        pickle.dump([Ld, Ggp], f)

else:
    with open(path + 'datasize_analysis_' + Obj + '_gp.pkl', 'r') as f: 
        Ld, Ggp = np.array(pickle.load(f))

 
Ggp = medfilter(Ggp, 10)

plt.plot(Ld, Ggp, '-b', label = 'GP')

plt.xlabel('Number of datapoints', fontsize=16)
plt.ylabel('RMSE (mm)', fontsize=16)
plt.title('GP Prediction error')
# plt.legend()
# plt.xlim([0,32])
# plt.ylim([0,3])
plt.savefig(path + 'datasize_' + Obj + '.png', dpi=300) #str(np.random.randint(100000))
plt.show()

