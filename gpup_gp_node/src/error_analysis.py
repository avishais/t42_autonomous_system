#!/usr/bin/env python

import rospy
from gpup_gp_node_exp.srv import gpup_transition, batch_transition, one_transition
from std_srvs.srv import Empty, EmptyResponse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Ellipse, Polygon
import pickle
from nn_predict.srv import StateAction2State
import time
import var

# np.random.seed(10)

state_dim = 12
version = 0
Obj = 'cyl45'

naive_srv = rospy.ServiceProxy('/gp/transitionOneParticle', one_transition)
nn_srv = rospy.ServiceProxy('/nn/predict', StateAction2State)
rospy.init_node('error_analysis_t42', anonymous=True)

path = '/home/pracsys/catkin_ws/src/t42_control/gpup_gp_node/src/results/'
test_path = '/home/pracsys/catkin_ws/src/t42_control/hand_control/data/dataset/'

def medfilter(x, W):
    w = int(W/2)
    x_new = np.copy(x)
    for i in range(0, x.shape[0]):
        if i < w:
            x_new[i] = np.mean(x[:i+w])
        elif i > x.shape[0]-w:
            x_new[i] = np.mean(x[i-w:])
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

def predict_NN(s_start, A):
        print "Running NN with horizon " + str(A.shape[0])

        s = np.copy(s_start)# + np.random.normal(0, sigma_start)
        # s = np.tile(s, (Np,1)) + np.random.normal(0, sigma_start, (Np, state_dim))
        S = s.reshape(1,state_dim)

        p_naive = 1
        for i in range(0, A.shape[0]):
            # print("[NN] Step " + str(i) + " of " + str(A.shape[0]))
            a = A[i,:]

            res = nn_srv(s.reshape(-1,1), a)

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

def plato(G, n = 100):
    lmax = G[:,1].max()
    l = np.linspace(0, lmax, n)

    H = []
    H.append(0.0)
    for i in range(1,len(l)):
        inx = np.where(np.logical_and(G[:,1] >= l[i-1],  G[:,1] <= l[i]))[0]
        H.append( np.mean(G[inx,2]) )

    return l, np.array(H)

H = range(0, 301, 10)
H[0] = 1
w = [40, 40, 100, 100]

## GP
if 1:
    with open(test_path + 'testpaths_' + Obj + '_d_v' + str(version) + '.pkl', 'r') as f: 
        action_seq, test_paths, Obj, Suc = pickle.load(f)

    if 1:
        with open(path + 'prediction_analysis_' + Obj + '_gp1.pkl', 'r') as f: 
            Ggp = pickle.load(f)
    else: 
        Ggp = []

    j = 1
    while j < 10000:
        print("Run %d, number of samples %d."%(j, len(Ggp)))
        try:
            h = np.random.randint(1,150)
            path_inx = np.random.randint(len(test_paths))
            R = test_paths[path_inx]
            A = action_seq[path_inx]
            R = R[:,[0,1,11,12,3,4,5,6,7,8,9,10]]

            # Randomly pick a section with length h
            st_inx = np.random.randint(R.shape[0]-h-1)
            R = R[st_inx:st_inx+h]
            A = A[st_inx:st_inx+h]

            for i in range(state_dim):
                try:
                    R[:,i] = medfilter(R[:,i], w[i])
                except:
                    R[:,i] = medfilter(R[:,i], 40)

            s_start = R[0,:]
            R_gp = predict_GP(s_start, A)

            e, l = tracking_error(R, R_gp)
        except:
            continue

        Ggp.append(np.array([h, l, e]))
        j += 1

        if j == 10000 or j % 50 == 0:
            with open(path + 'prediction_analysis_' + Obj + '_gp1.pkl', 'w') as f: 
                pickle.dump(Ggp, f)

    # Ggp = np.zeros((len(H), 2))    

    # j = 0
    # for h in H: # Horizon
    #     t = int(-19./10.*h) + 200 if h < 110 else 10
    #     Sum_gp = 0.0
    #     Sum_nn = 0.0
    #     for _ in range(t): # Number of tests to average
    #         path_inx = np.random.randint(len(test_paths))
    #         R = test_paths[path_inx]
    #         A = action_seq[path_inx]
    #         R = R[:,[0,1,11,12]]

    #         # Randomly pick a section with length h
    #         st_inx = np.random.randint(R.shape[0]-h-1)
    #         R = R[st_inx:st_inx+h]
    #         A = A[st_inx:st_inx+h]

    #         for i in range(state_dim):
    #             R[:,i] = medfilter(R[:,i], w[i])

    #         s_start = R[0,:]
    #         R_gp = predict_GP(s_start, A)

    #         Sum_gp += tracking_error(R, R_gp)

    #     Ggp[j,0] = h
    #     Ggp[j,1] = Sum_gp / t
    #     j += 1

    with open(path + 'prediction_analysis_' + Obj + '_gp1.pkl', 'w') as f: 
        pickle.dump(Ggp, f)

    ######################################## Plot ###########################################################
else:
    with open(path + 'prediction_analysis_' + Obj + '_gp1.pkl', 'r') as f: 
        Ggp = pickle.load(f) 

if 0:
    with open(test_path + 'testpaths_' + Obj + '_d_v' + str(version) + '.pkl', 'r') as f: 
        action_seq, test_paths, Obj, Suc = pickle.load(f)

    if 1:
        with open(path + 'prediction_analysis_' + Obj + '_nn.pkl', 'r') as f: 
            Gnn = pickle.load(f)
    else: 
        Gnn = []

    j = 1
    while j < 5000:
        print("Run %d, number of samples %d."%(j, len(Gnn)))
        try:
            h = np.random.randint(150,250)
            path_inx = np.random.randint(len(test_paths))
            R = test_paths[path_inx]
            A = action_seq[path_inx]
            R = R[:,[0,1,11,12]]

            # Randomly pick a section with length h
            st_inx = np.random.randint(R.shape[0]-h-1)
            R = R[st_inx:st_inx+h]
            A = A[st_inx:st_inx+h]

            for i in range(state_dim):
                R[:,i] = medfilter(R[:,i], w[i])

            s_start = R[0,:]
            R_nn = predict_NN(s_start, A)

            e, l = tracking_error(R, R_nn)
        except:
            continue

        Gnn.append(np.array([h, l, e]))
        j += 1

        if j == 5000 or j % 200 == 0:
            with open(path + 'prediction_analysis_' + Obj + '_nn.pkl', 'w') as f: 
                pickle.dump(Gnn, f)

    # j = 0
    # for h in H: # Horizon
    #     t = 500
    #     Sum_gp = 0.0
    #     Sum_nn = 0.0
    #     for k in range(t): # Number of tests to average
    #         print("Test number %d."%k)
    #         path_inx = np.random.randint(len(test_paths))
    #         R = test_paths[path_inx]
    #         A = action_seq[path_inx]
    #         R = R[:,[0,1,11,12]]

    #         # Randomly pick a section with length h
    #         st_inx = np.random.randint(R.shape[0]-h-1)
    #         R = R[st_inx:st_inx+h]
    #         A = A[st_inx:st_inx+h]

    #         for i in range(state_dim):
    #             R[:,i] = medfilter(R[:,i], w[i])

    #         s_start = R[0,:]
    #         R_nn = predict_NN(s_start, A)

    #         Sum_nn += tracking_error(R, R_nn)

    #     Gnn[j,0] = h
    #     Gnn[j,1] = Sum_nn / t
    #     j += 1

    ######################################## Plot ###########################################################
else:
    with open(path + 'prediction_analysis_' + Obj + '_nn.pkl', 'r') as f: 
        Gnn = pickle.load(f) 
 
Ggp = np.array(Ggp)
lgp, Egp = plato(Ggp, 20)

Gnn = np.array(Gnn)
lnn, Enn = plato(Gnn, 20)

plt.plot(Ggp[:,1], Ggp[:,2], '.m', label = 'GP raw')
plt.plot(lgp, Egp, '.-b', label = 'GP')
plt.plot(Gnn[:,1], Gnn[:,2], '.r', label = 'NN raw')
plt.plot(lnn, Enn, '.-k', label = 'NN')

plt.xlabel('Horizon (number of steps)')
plt.ylabel('RMSE [mm]')
plt.legend()
plt.show()

