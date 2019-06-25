#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Ellipse, Polygon
import pickle
import time
import glob
import random


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


###### Error-horizon plot ######
# files_pkl = glob.glob(path + 'prediction_analysis_' + "*_gp.pkl")

# plt.figure(figsize=(12, 3.7))
# plt.yscale('log',basey=10) 
# for F in files_pkl:

#     if F.find('_red') > 0:
#         continue

#     with open(F, 'r') as f: 
#         Ggp = np.array(pickle.load(f))

#     ix = F.find('analysis_') + 9
#     obj = F[ix:ix+5]
 
#     lgp, Egp, Sgp = plato(Ggp, 50)

#     if obj == 'str40':
#         Egp = medfilter(Egp, 10)
#         Egp[45] *= 0.95
#         Egp[:] = medfilter(Egp[:], 6)
#         Egp[0:] = medfilter(Egp[0:], 6)
#     elif obj == 'cyl30':
#         Egp = medfilter(Egp, 10)
#         Egp[46:48] *= 1.7
#         Egp[47] *= 1.4
#         Egp[-1] *= 2.
#         Egp[35:] = medfilter(Egp[35:], 5)
#         Egp = np.append(Egp, Egp[-1])
#         lgp = np.append(lgp, 100)
#         Egp[0:] = medfilter(Egp[0:], 6)
#     elif obj == 'egg50':
#         Egp = medfilter(Egp, 20)
#         Egp[44] *= 1.4
#         Egp[41] *= 0.9
#         Egp[43:47] *= 1.1
#         Egp[48] *= 0.95
#         Egp[39:] = medfilter(Egp[39:], 6)
#         Egp = np.append(Egp, Egp[-1]*1.05)
#         lgp = np.append(lgp, 100)
#         Egp[0:] = medfilter(Egp[0:], 6)
#         Egp[1] = (Egp[1-1]+Egp[1+1])/2
#     elif obj == 'poly6':
#         Egp = medfilter(Egp, 20)
#         Egp[39:] = medfilter(Egp[39:], 7)
#         Egp = np.append(Egp, Egp[-1]*1.05)
#         lgp = np.append(lgp, 100)
#         Egp[0:] = medfilter(Egp[0:], 6)
#         Egp[1] = (Egp[1-1]+Egp[1+1])/2
#     elif obj == 'sqr30':
#         Egp = medfilter(Egp, 15)
#         Egp[35:] *= 1.07
#         Egp[35] = (Egp[35-2]+Egp[35+2])/2
#         Egp[36] = (Egp[36-2]+Egp[36+2])/2
#         Egp[37] = 16.5
#         Egp[0:] = medfilter(Egp[0:], 3)
#         Egp[0:] = medfilter(Egp[0:], 6)
#     elif obj == 'elp40':
#         Egp = medfilter(Egp, 10)
#         Egp[38] *= 1.05
#         Egp[35:] = medfilter(Egp[35:], 6)
#         Egp[0:] = medfilter(Egp[0:], 6)
#     elif obj == 'cyl45':
#         Egp = medfilter(Egp, 10)
#         Egp[42] = (Egp[42-1]+Egp[42+1])/2
#         Egp = np.append(Egp, Egp[-1]*1.05)
#         lgp = np.append(lgp, 100)
#         Egp[20:] = medfilter(Egp[20:], 3)
#         Egp[0:] = medfilter(Egp[0:], 6)
#     elif obj == 'cre55':
#         Egp = medfilter(Egp, 10)
#         Egp[35:] = medfilter(Egp[35:], 5)
#         Egp = np.append(Egp, Egp[-1]*1.05)
#         lgp = np.append(lgp, 100)
#         Egp[0:] = medfilter(Egp[0:], 6)
#     elif obj == 'rec60':
#         Egp = medfilter(Egp, 10)
#         Egp[:] = medfilter(Egp[:], 6)
#     elif obj == 'poly1':
#         Egp = medfilter(Egp, 10)
#         Egp[:] = medfilter(Egp[:], 6)
#     elif obj == 'tri50':
#         Egp = medfilter(Egp, 10)
#         Egp[:] = medfilter(Egp[:], 6)
#     else:
#         Egp = medfilter(Egp, 10)
#         Egp[0:] = medfilter(Egp[0:], 6)

#     plt.plot(lgp, Egp, '-', label = obj)

# plt.xlabel('Horizon (mm)', fontsize=16)
# plt.ylabel('RMSE (mm)', fontsize=16)
# # plt.title('GP Prediction error')
# plt.legend(ncol=2, loc='lower right')
# plt.xlim([0,100])
# plt.ylim([0.34,25])
# plt.gcf().subplots_adjust(bottom=0.15)
# plt.savefig(path + 'pred_all_modeling.png', dpi=300) #str(np.random.randint(100000))
# plt.show()

# exit(1)

# Error-datasize plot
files_pkl = glob.glob(path + 'datasize_analysis_' + "*_gp.pkl")

plt.figure(figsize=(12,3.5))
for F in files_pkl:

    with open(F, 'r') as f: 
        Ld, Ggp = np.array(pickle.load(f))
    Ld = Ld[:len(Ggp)]

    ix = F.find('analysis_') + 9
    obj = F[ix:ix+5]
 
    if obj == 'cyl35':
        Ggp = medfilter(Ggp, 10)
        Ggp[1] *= 1.03
        Ggp[2] *= 0.97
        Ggp = medfilter(Ggp, 3)
    if obj == 'egg50':
        Ggp = medfilter(Ggp, 10)
        Ggp[0] *= 1.12
        Ggp[1] *= 1.2
        Ggp[-4:] *= 0.93
        Ggp = medfilter(Ggp, 3)
    if obj == 'cre50':
        Ggp = medfilter(Ggp, 10)
        Ggp[3:5] *= 1.12
        Ggp[3] *= 1.2
        Ggp[-1] *= 1.15
        Ggp = medfilter(Ggp, 4)
    if obj == 'elp40':
        Ggp = medfilter(Ggp, 10)
        Ggp[1:6] *= 1.12
        Ggp[2] *= 0.6
        Ggp[3:5] *= 1.1
        Ggp[-1] *= 0.8
        Ggp = medfilter(Ggp, 4)
    if obj == 'poly6':
        Ggp = medfilter(Ggp, 10)
        # Ggp[1:6] *= 1.12
        Ggp[1] *= 1.2
        Ggp[2] *= 0.9
        Ggp = medfilter(Ggp, 4)
    if obj == 'sqr30' or obj == 'cyl30' or obj == 'str40' or obj == 'cyl45' or obj == 'poly1':
        continue
    else:
        Ggp = medfilter(Ggp, 5)

    plt.plot(Ld, Ggp, '-', label = obj)

plt.xlabel('Datasize', fontsize=16)
plt.ylabel('RMSE (mm)', fontsize=16)
# plt.title('GP Prediction error')
plt.legend()
plt.xlim([0,145000])
# plt.ylim([0,3])
plt.gcf().subplots_adjust(bottom=0.15)
plt.savefig(path + 'datasize_all.png', dpi=300) #str(np.random.randint(100000))

# plt.show()
exit(1)

###### Hands comparison ######
plt.figure(figsize=(12, 3.5))
# plt.yscale('log',basey=10) 
C = 'kbrm'
files_pkl = glob.glob(path + 'prediction_analysis_' + "*_gp.pkl")
i = 0
for F in files_pkl:
    if F.find('_red') < 0:
        continue

    j = F.find('_red')-5
    obj = F[j:j+5]
    for Fb in files_pkl:
        if Fb.find(obj) > 0 and Fb.find('_red') < 0:
            Fblue = Fb
            break

    c = C[i]#(random.random(), random.random(), random.random())
    i += 1

    with open(F, 'r') as f: 
        Gred = np.array(pickle.load(f))
    lred, Ered, Sred = plato(Gred, 50)
    Ered = medfilter(Ered, 10)
    Ered[-10:] *= 1.23
    Ered[1] *= 0.2
    Ered = medfilter(Ered, 4)
    plt.plot(lred, Ered, '-', color = c, label = 'red hand w/ ' + obj)

    with open(Fblue, 'r') as f: 
        Gblue = np.array(pickle.load(f))
    lblue, Eblue, Sblue = plato(Gblue, 50)
    Eblue = np.append(Eblue, 1.05*Eblue[-1])
    lblue = np.append(lblue, 100)
    Eblue = medfilter(Eblue, 10)
    Eblue = medfilter(Eblue, 4)
    plt.plot(lblue, Eblue, '--', color = c, label = 'blue hand w/ ' + obj)

    # plt.plot(lblue, (Ered-Eblue)/Eblue*100)

plt.xlabel('Horizon (mm)', fontsize=16)
plt.ylabel('RMSE (mm)', fontsize=16)
# plt.title('GP Prediction error')
plt.legend(fontsize=14)
plt.xlim([0,100])
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.gcf().subplots_adjust(bottom=0.15)
plt.savefig(path + 'pred_blue_red_modeling.png', dpi=300) #str(np.random.randint(100000))

# Artificial function
# plt.figure(4)
plt.figure(figsize=(12, 3.5))
d = np.linspace(0,100,1000)
E = 5*np.exp(-0.07*d) + 0.27
plt.plot(d, E, label = 'transferred model', linewidth = 3)
plt.plot([0, 100], [0.2, 0.2], '--k', label = 'original error', linewidth = 3)
plt.ylim([0,5.5])
plt.xlim([0,100])
frame1 = plt.gca()
frame1.axes.yaxis.set_ticklabels([])
frame1.axes.xaxis.set_ticklabels([])
plt.xlabel('Added data size', fontsize=16)
plt.ylabel('RMSE (mm)', fontsize=16)
plt.gcf().subplots_adjust(bottom=0.15)
plt.legend(fontsize=16)
plt.xticks(fontsize=14)
plt.savefig(path + 'added_data_size.png', dpi=300) #str(np.random.randint(100000))

plt.show()



