#!/usr/bin/env python

import rospy
from gpup_gp_node_exp.srv import gpup_transition, batch_transition, one_transition
from std_srvs.srv import Empty, EmptyResponse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Ellipse, Polygon
import pickle
from rollout_t42.srv import rolloutReq
import time
from data_load import data_load
import var

# np.random.seed(10)

state_dim = var.state_dim_
stepSize = var.stepSize_
version = var.data_version_
Obj = 'cyl45'

gp_srv = rospy.ServiceProxy('/gp/transition', batch_transition)
naive_srv = rospy.ServiceProxy('/gp/transitionOneParticle', one_transition)
rospy.init_node('GP_t42', anonymous=True)

path = '/home/juntao/catkin_ws/src/t42_control/gpup_gp_node/src/results/'
test_path = '/home/juntao/catkin_ws/src/t42_control/hand_control/data/dataset/'

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

if 0:
    with open(test_path + 'testpaths_' + Obj + '_d_v' + str(version) + '.pkl', 'r') as f: 
        action_seq, test_paths, Obj, Suc = pickle.load(f)
    
    # action_seq = action_seq[:1]
    # test_paths = test_paths[:1]
    # Suc = Suc[:1]

    # action_seq[0] = action_seq[0][:200,:]
    # test_paths[0] = test_paths[0][:200,:]#[:,:4]

    GP_batch = []
    GP_naive = []
    filtered_test_paths = []
    for A, R in zip(action_seq, test_paths):
        if np.any(Obj == np.array(['sqr30','poly10','poly6','elp40'])): # Include orientation angle
            R = R[:,[0,1,11,12,2]]
        else:
            # R = R[:,[0,1,11,12,3,4,5,6,7,8,9,10]]
            R = R[:,[0,1,11,12]]
        state_dim = R.shape[1]
        # R = R[20:,:]
        # A = A[20:,:]

        A = np.concatenate((A, np.tile(R[0,:], (A.shape[0],1))), axis=1)

        print('Smoothing data...')
        h = [40, 40, 100, 100]
        for i in range(state_dim):
            try:
                R[:,i] = medfilter(R[:,i], h[i])
            except:
                R[:,i] = medfilter(R[:,i], 40)


        A = np.concatenate((A, np.tile(R[0,:], (A.shape[0],1))), axis=1)

        filtered_test_paths.append(R)

        s_start = R[0,:]
        sigma_start = np.ones((1,state_dim))*1e-3

        Np = 100 # Number of particles

        ######################################## GP propagation ##################################################

        print "Running GP."
        
        t_gp = 0

        s = np.copy(s_start)
        S = np.tile(s, (Np,1)) + np.random.normal(0, sigma_start, (Np, state_dim))
        Ypred_mean_gp = s.reshape(1,state_dim)
        Ypred_std_gp = sigma_start.reshape(1,state_dim)

        Pgp = []; 
        p_gp = 1
        print("Running (open loop) path...")
        for i in range(0, A.shape[0]):
            # print("[GP] Step " + str(i) + " of " + str(A.shape[0]))
            Pgp.append(S)
            a = A[i,:]

            # st = time.time()
            # res = gp_srv(S.reshape(-1,1), a)
            # t_gp += (time.time() - st) 

            # S_next = np.array(res.next_states).reshape(-1,state_dim)
            # if res.node_probability < p_gp:
            #     p_gp = res.node_probability
            # s_mean_next = np.mean(S_next, 0)
            # s_std_next = np.std(S_next, 0)
            # S = S_next

            # if S_next.shape[0] == 0:
            #     break

            s_mean_next = np.zeros((state_dim,))
            s_std_next = np.zeros((state_dim,))

            Ypred_mean_gp = np.append(Ypred_mean_gp, s_mean_next.reshape(1,state_dim), axis=0)
            Ypred_std_gp = np.append(Ypred_std_gp, s_std_next.reshape(1,state_dim), axis=0)

        t_gp /= A.shape[0]

        GP_batch.append([(Ypred_mean_gp), (Ypred_std_gp)])
        

        ######################################## naive propagation ###############################################

        print "Running Naive."
        Np = 1 # Number of particles
        t_naive = 0

        s = np.copy(s_start)# + np.random.normal(0, sigma_start)
        # s = np.tile(s, (Np,1)) + np.random.normal(0, sigma_start, (Np, state_dim))
        Ypred_naive = s.reshape(1,state_dim)

        print("Running (open loop) path...")
        p_naive = 1
        for i in range(0, A.shape[0]):
            print("[Naive] Step " + str(i) + " of " + str(A.shape[0]))
            a = A[i,:]
            print s, a


            st = time.time()
            res = naive_srv(s.reshape(-1,1), a)
            t_naive += (time.time() - st) 

            if res.node_probability < p_naive:
                p_naive = res.node_probability
            s_next = np.array(res.next_state)
            s = np.copy(s_next)

            Ypred_naive = np.append(Ypred_naive, s_next.reshape(1,state_dim), axis=0)

        GP_naive.append(Ypred_naive)

        t_naive /= A.shape[0]

    ######################################## Save ###########################################################

    with open(path + 'testpaths_' + Obj + '_d_v' + str(version) + '.pkl', 'w') as f: 
        pickle.dump([action_seq, filtered_test_paths, GP_batch, GP_naive], f)

    ######################################## Plot ###########################################################
else:
    with open(path + 'testpaths_' + Obj + '_d_v' + str(version) + '.pkl', 'r') as f: 
        action_seq, filtered_test_paths, GP_batch, GP_naive = pickle.load(f)  

ix = [0, 1]
# plt.figure(1)
# ax1 = plt.subplot(2,1,1)
# ax1.plot(t[:-1], Smean[:,ix[0]], '-b', label='rollout mean')
# ax1.fill_between(t[:-1], Smean[:,ix[0]]+Sstd[:,ix[0]], Smean[:,ix[0]]-Sstd[:,ix[0]], facecolor='blue', alpha=0.5, label='rollout std.')
# ax1.plot(t, Ypred_mean_gp[:,ix[0]], '-r', label='BPP mean')
# ax1.fill_between(t, Ypred_mean_gp[:,ix[0]]+Ypred_std_gp[:,ix[0]], Ypred_mean_gp[:,ix[0]]-Ypred_std_gp[:,ix[0]], facecolor='red', alpha=0.5, label='BGP std.')
# ax1.plot(t, Ypred_mean_gpup[:,0], '--c', label='GPUP mean')
# ax1.fill_between(t, Ypred_mean_gpup[:,0]+Ypred_std_gpup[:,0], Ypred_mean_gpup[:,0]-Ypred_std_gpup[:,0], facecolor='cyan', alpha=0.5, label='GPUP std.')
# ax1.plot(t, Ypred_naive[:,0], '-k', label='Naive')
# ax1.plot(t, Ypred_bmean[:,0], '-m', label='Batch mean')
# ax1.legend()
# plt.title('Path ' + tr)
# ax2 = plt.subplot(2,1,2)
# ax2.plot(t[:-1], Smean[:,ix[1]], '-b')
# ax2.fill_between(t[:-1], Smean[:,ix[1]]+Sstd[:,ix[1]], Smean[:,ix[1]]-Sstd[:,ix[1]], facecolor='blue', alpha=0.5)
# ax2.plot(t, Ypred_mean_gp[:,ix[1]], '-r')
# ax2.fill_between(t, Ypred_mean_gp[:,ix[1]]+Ypred_std_gp[:,ix[1]], Ypred_mean_gp[:,ix[1]]-Ypred_std_gp[:,ix[1]], facecolor='red', alpha=0.5)
# ax2.plot(t, Ypred_mean_gpup[:,1], '--c')
# ax2.fill_between(t, Ypred_mean_gpup[:,1]+Ypred_std_gpup[:,1], Ypred_mean_gpup[:,1]-Ypred_std_gpup[:,1], facecolor='cyan', alpha=0.5)
# ax2.plot(t, Ypred_naive[:,1], '-k')
# ax2.plot(t, Ypred_bmean[:,1], '-m')

ix = [0, 1]
tr = 0
for S, Sbatch, Snaive in zip(filtered_test_paths, GP_batch, GP_naive):

    plt.figure(0)
    plt.plot(S[:,ix[0]], S[:,ix[1]], '.-k', label='rollout')
    # plt.plot(Sbatch[0][:,ix[0]], Sbatch[0][:,ix[1]], '.-r', label='BPP')
    plt.plot(Snaive[:,ix[0]], Snaive[:,ix[1]], '.-c', label='Naive')
    plt.axis('equal')
    plt.legend()
    # plt.show()

    plt.savefig('/home/juntao/catkin_ws/src/t42_control/gpup_gp_node/src/results/path_' + str(tr) + '.png', dpi=300) #str(np.random.randint(100000))
    tr += 1
plt.show()

