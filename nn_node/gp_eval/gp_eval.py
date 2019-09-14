#!/usr/bin/env python

import rospy
import numpy as np
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF
import matplotlib.pyplot as plt
from matplotlib import cm
import pickle
from gpup_gp_node_exp.srv import one_transition
from sklearn.neighbors import KDTree

# o_srv = rospy.ServiceProxy('/nn/transitionOneParticle', one_transition)
# rospy.init_node('gp_eval', anonymous=True)

path = '/home/pracsys/catkin_ws/src/t42_control/nn_node/gp_eval/'

def tracking_error(S1, S2):
    Sum = 0.
    for s1, s2 in zip(S1, S2):
        Sum += np.linalg.norm(s1[:2]-s2[:2])**2

    return np.sqrt(Sum / S1.shape[0])

with open(path + 't42_cyl35_data_discrete_v0_d4_m1_episodes.obj', 'rb') as f: 
    D = pickle.load(f)
del D[:182] # Delete data that was used for training


l_prior = 40
if 0:
    if 1:
        O = []
        M = []
        E = []
        Apr = []
    else:
        with open(path + 'error_points_P' + str(l_prior) + 'a.pkl', 'rb') as f: 
            O, M, Apr, E = pickle.load(f)
            O = list(O)
            E = list(E)
            Apr = list(Apr)
            M = list(M)
    N = 1000000*2
    for k in range(len(O), N):
        ix = np.random.randint(len(D))
        l = np.random.randint(10,30)
        jx = np.random.randint(D[ix].shape[0]-l)

        h = 1
        while h < l and np.all(D[ix][jx, 4:6] == D[ix][jx + h, 4:6]):
            h += 1
        if h < 10:
            continue
        l = np.minimum(h, l)

        S = D[ix][jx:jx+l,:4]
        A = D[ix][jx:jx+l,4:10]
        S_next = D[ix][jx:jx+l,10:]

        H = D[ix][np.maximum(0, jx-l_prior):jx, 4:10]
        Hl = np.copy(H)
        if H.shape[0] < l_prior:
            H = np.concatenate((np.zeros((l_prior-H.shape[0], 6)), H), axis=0) # Validate size!!!
        Sl = D[ix][np.maximum(0, jx-l_prior):jx, :4]
        
        Sp = []
        state = S[0]
        Sp.append(state)
        i = 0
        for a in A:
            state = o_srv(state.reshape(-1,1), a.reshape(-1,1)).next_state
            state = np.array(state)
            Sp.append(state)
        Sp = np.array(Sp)
        ep = tracking_error(S, Sp)

        if Sl.shape[0] > 0:
            SL = []
            state = Sl[0]
            SL.append(state)
            i = 0
            for a in Hl:
                state = o_srv(state.reshape(-1,1), a.reshape(-1,1)).next_state
                state = np.array(state)
                SL.append(state)
            SL = np.array(SL)
            el = tracking_error(Sl, SL)
        else:
            el = 0.0

        e = ep + el
        o = np.concatenate((S[0], A[0]), axis = 0)
        O.append(o)
        M.append(l)
        Apr.append(H)
        E.append(e)

        print k, A[0], l, e

        if k > 1 and not k % 2000:
            O1 = np.array(O)
            M1 = np.array(M)
            Apr1 = np.array(Apr)
            E1 = np.array(E)

            with open(path + 'error_points_P' + str(l_prior) + '.pkl', 'wb') as f: 
                pickle.dump([O1, M1, Apr1, E1], f)
else:
    # with open(path + 'error_points_P' + str(l_prior) + 'b.pkl', 'r') as f: 
    #     O1, L1, Apr1, E1 = pickle.load(f)
    # with open(path + 'error_points_P' + str(l_prior) + 'a.pkl', 'r') as f: 
    #     O2, L2, Apr2, E2 = pickle.load(f)

    # O = np.concatenate((O1, O2), axis=0)
    # L = np.concatenate((L1, L2), axis=0)
    # Apr = np.concatenate((Apr1, Apr2), axis=0)
    # E = np.concatenate((E1, E2), axis=0)

    # with open(path + 'error_points_P' + str(l_prior) + '.pkl', 'w') as f: 
    #     pickle.dump([O, L, Apr, E], f)
    with open(path + 'error_points_P' + str(l_prior) + '.pkl', 'r') as f: 
        O, L, Apr, E = pickle.load(f)




# On = []
# En = []
# for o, e in zip(O, E):
#     # if e < 0.47:
#     if np.all(o[4:6] == np.array([1,-1])):
#         On.append(o)
#         En.append(e)
# O = np.array(On)
# E = np.array(En)

# gridsize = 20
# plt.hexbin(O[:,0], O[:,1], C=E, gridsize=gridsize, cmap=cm.jet, bins=None)
# plt.colorbar()
# plt.show()
# exit(1)

print O.shape

if 1:
    with open(path + 'data_P' + str(l_prior) + '.pkl', 'rb') as f: 
        X, E = pickle.load(f)

    print X.shape
else:
    X = []
    i = 0
    for o, apr, l in zip(O, Apr, L):
        x = np.concatenate((o[:6], np.array([l]), apr.reshape((-1))), axis = 0)
        X.append(x)
        if i > 500000:
            break
        else:
            i += 1
    X = np.array(X)
    with open(path + 'data_P' + str(l_prior) + '.pkl', 'wb') as f: 
        pickle.dump([X, E], f)

M = 1000
Otrain = X[:-M]
Otest = X[-M:]

import warnings
warnings.filterwarnings("ignore")

Etrain = E[:-M]
Etest = E[-M:]
d = Otrain.shape[1]

if 0:
    kdt = KDTree(Otrain, leaf_size=100, metric='euclidean')
    with open(path + 'kdt_P' + str(l_prior) + '.pkl', 'wb') as f: 
        pickle.dump(kdt, f)
else:
    with open(path + 'kdt_P' + str(l_prior) + '.pkl', 'rb') as f: 
        kdt = pickle.load(f)
K = 10
kernel = RBF(length_scale=1.0, length_scale_bounds=(1e-1, 10.0))
i = 0
T = []
Err = []
import time
for e, o in zip(Etest, Otest):
    # print i
    # print o
    st = time.time()
    idx = kdt.query(o[:d].reshape(1,-1), k = K, return_distance=False)
    O_nn = Otrain[idx,:].reshape(K, d)
    E_nn = Etrain[idx].reshape(K, 1)

    gpr = GaussianProcessRegressor(kernel=kernel).fit(O_nn, E_nn)
    e_mean = gpr.predict(o.reshape(1, -1), return_std=False)[0][0]
    T.append(time.time() - st)
    Err.append(np.abs(e-e_mean))
    # print e, e_mean, np.abs(e-e_mean), o[-1]
    if i >=0:
        print e, e_mean
    i += 1

print "Time: " + str(np.mean(T))
print "Error: " + str(np.mean(Err))