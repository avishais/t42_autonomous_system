#!/usr/bin/env python

import rospy
from gpup_gp_node_exp.srv import gpup_transition, batch_transition, one_transition, setKD
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
Obj = 'poly10'
if np.any(Obj == np.array(['sqr30','poly10','poly6','elp40','str40','rec60','rec10','egg50','cre55'])):
    state_dim = 5
else:
    state_dim = 4

naive_srv = rospy.ServiceProxy('/gp/transitionOneParticle', one_transition)
nn_srv = rospy.ServiceProxy('/nn/predict', StateAction2State)
setKD_srv = rospy.ServiceProxy('/gp/set_new_kdtree', setKD)
rospy.init_node('error_analysis_t42', anonymous=True)

print "Waiting for service /gp/transitionOneParticle ..."
rospy.wait_for_service('/gp/transitionOneParticle')

path = '/home/juntao/catkin_ws/src/t42_control/gpup_gp_node/src/for_paper/results/'
test_path = '/home/juntao/catkin_ws/src/t42_control/hand_control/data/dataset/'

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

def predict_GP(s_start, A):
    print "Running GP with horizon " + str(A.shape[0])

    s = np.copy(s_start)# + np.random.normal(0, sigma_start)
    S = s.reshape(1,state_dim)

    p_naive = 1
    for i in range(0, A.shape[0]):
        # print("[GP] Step " + str(i) + " of " + str(A.shape[0]))
        a = A[i,:]

        res = naive_srv(s.reshape(-1,1), a)

        s_next = np.array(res.next_state)
        s = np.copy(s_next)

        S = np.append(S, s_next.reshape(1,state_dim), axis=0)

    return S

def tracking_error(S1, S2):
    Sum = 0.
    for s1, s2 in zip(S1, S2):
        Sum += np.linalg.norm(s1[:2]-s2[:2])**2

    l = 0.
    for i in range(1,S1.shape[0]):
        l += np.linalg.norm(S1[i,:2] - S1[i-1,:2])

    return np.sqrt(Sum / S1.shape[0]), l


w = [40, 40, 100, 100]

## GP
if 1:
    with open(test_path + 'testpaths_' + Obj + '_d_v' + str(version) + '.pkl', 'r') as f: 
        action_seq, test_paths, Obj, Suc = pickle.load(f)

    if 0:
        with open(path + 'datasize_analysis_' + Obj + '_gp.pkl', 'r') as f: 
            Ld, Ggp = pickle.load(f)
    else: 
        Ggp = []
        l_start = 1000

    Ld = range(l_start, 150000,2000)#2500)
    n = 100
    h = 10
    for l in Ld:
        g = setKD_srv(l)
        e = 0.0
        for k in range(n):
            print("Run data %d points, object %s, and trial %d."%(l, Obj, k))
            path_inx = np.random.randint(len(test_paths))
            R = test_paths[path_inx]
            A = action_seq[path_inx]
            if state_dim == 5:
                R = R[:,[0,1,11,12,2]]
            else:
                R = R[:,[0,1,11,12]]
            A = np.concatenate((A, np.tile(R[0,:], (A.shape[0], 1))), axis=1)
            st_inx = np.random.randint(R.shape[0]-h-1)

            # s = R[st_inx]
            # s_next = R[st_inx+1]
            # a = A[st_inx]
            # res = naive_srv(s.reshape(-1,1), a)
            # s_next_pred = np.array(res.next_state)
            # e += np.linalg.norm(s_next_pred[:2]-s_next[:2])**2

            for i in range(state_dim):
                if i == 4:
                    continue
                try:
                    R[:,i] = medfilter(R[:,i], w[i])
                except:
                    R[:,i] = medfilter(R[:,i], 40)
            R = R[st_inx:st_inx+h]
            A = A[st_inx:st_inx+h]
            R_gp = predict_GP(R[0,:], A)
            E, _ = tracking_error(R, R_gp)
            e += E**2

        Ggp.append(np.sqrt(e / n))
        
        with open(path + 'datasize_analysis_' + Obj + '_gp.pkl', 'w') as f: 
            pickle.dump([Ld, Ggp], f)

    Ggp = np.array(Ggp)

    with open(path + 'datasize_analysis_' + Obj + '_gp.pkl', 'w') as f: 
        pickle.dump([Ld, Ggp], f)

else:
    with open(path + 'datasize_analysis_' + Obj + '_gp.pkl', 'r') as f: 
        Ld, Ggp = np.array(pickle.load(f))

 
Ggp = medfilter(Ggp, 5)

plt.plot(Ld, Ggp, '-b', label = 'GP')

plt.xlabel('Number of datapoints', fontsize=16)
plt.ylabel('RMSE (mm)', fontsize=16)
plt.title('GP Prediction error')
# plt.legend()
# plt.xlim([0,32])
# plt.ylim([0,3])
plt.savefig(path + 'datasize_' + Obj + '.png', dpi=300) #str(np.random.randint(100000))
plt.show()

