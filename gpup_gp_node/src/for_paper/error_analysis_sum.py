#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Ellipse, Polygon
import pickle
import time
import glob


version = 0
Obj = 'cyl35'
if np.any(Obj == np.array(['sqr30','poly10','poly6','elp40','str40','rec60','rec10','str40','egg50'])):
    state_dim = 5
else:
    state_dim = 4


path = '/home/pracsys/catkin_ws/src/t42_control/gpup_gp_node/src/for_paper/results/'

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
    S = []
    H.append(0.0)
    S.append(0.0)
    for i in range(1,len(l)):
        inx = np.where(np.logical_and(G[:,1] >= l[i-1],  G[:,1] <= l[i]))[0]
        H.append( np.mean(G[inx,2]) )
        S.append( np.std(G[inx, 2]) )

    H = np.array(H)
    S = np.array(S)

    inx = np.where(np.isnan(H))
    H = np.delete(H, inx, 0)
    S = np.delete(S, inx, 0)
    l = np.delete(l, inx, 0)

    return l, np.array(H), np.array(S)


# Error-horizon plot
files_pkl = glob.glob(path + 'prediction_analysis_' + "*_gp.pkl")

plt.figure(figsize=(12, 3.5))
for F in files_pkl:

    with open(F, 'r') as f: 
        Ggp = np.array(pickle.load(f))

    ix = F.find('analysis_') + 9
    obj = F[ix:ix+5]
 
    lgp, Egp, Sgp = plato(Ggp, 50)
    Egp = medfilter(Egp, 10)

    plt.plot(lgp, Egp, '-', label = obj)

plt.xlabel('Horizon (mm)', fontsize=16)
plt.ylabel('RMSE (mm)', fontsize=16)
# plt.title('GP Prediction error')
plt.legend()
plt.xlim([0,100])
plt.ylim([0,12])
plt.gcf().subplots_adjust(bottom=0.15)
plt.savefig(path + 'pred_all_modeling.png', dpi=300) #str(np.random.randint(100000))

# Error-datasize plot
files_pkl = glob.glob(path + 'datasize_analysis_' + "*_gp.pkl")

plt.figure(figsize=(10,4.5))
for F in files_pkl:

    with open(F, 'r') as f: 
        Ld, Ggp = np.array(pickle.load(f))
    Ld = Ld[:len(Ggp)]

    ix = F.find('analysis_') + 9
    obj = F[ix:ix+5]
 
    # Ggp = medfilter(Ggp, 10)

    plt.plot(Ld, Ggp, '-', label = obj)

plt.xlabel('Datasize', fontsize=16)
plt.ylabel('RMSE (mm)', fontsize=16)
# plt.title('GP Prediction error')
plt.legend()
# plt.xlim([0,32])
# plt.ylim([0,3])
# plt.savefig(path + 'datasize_all.png', dpi=300) #str(np.random.randint(100000))
plt.show()

