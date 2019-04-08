#!/usr/bin/env python

import rospy
from gpup_gp_node.srv import gpup_transition, batch_transition, one_transition
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
tr = '1'
stepSize = var.stepSize_
version = var.data_version_

gp_srv = rospy.ServiceProxy('/gp/transition', batch_transition)
naive_srv = rospy.ServiceProxy('/gp/transitionOneParticle', one_transition)
rollout_srv = rospy.ServiceProxy('/rollout/rollout', rolloutReq)

rospy.init_node('verification_t42', anonymous=True)


# path = '/home/pracsys/catkin_ws/src/t42_control/rollout_t42/rolls/'
path = '/home/pracsys/catkin_ws/src/t42_control/gpup_gp_node/src/results/'


# DL = data_load(simORreal = 't42_35', K = 10)
# Smean = R = DL.Qtest[:,:4]
# A = DL.Qtest[:,4:6]

# Smean = np.loadtxt('/home/pracsys/catkin_ws/src/t42_control/hand_control/plans/robust_particles_pc_svmHeuristic_goal1_run0_traj.txt', delimiter=',')[:,:4]
# A = np.loadtxt('/home/pracsys/catkin_ws/src/t42_control/hand_control/plans/robust_particles_pc_svmHeuristic_goal1_run0_plan.txt', delimiter=',')[:,:2]
# R = np.loadtxt('/home/pracsys/catkin_ws/src/t42_control/hand_control/plans/robust_particles_pc_svmHeuristic_goal1_run0_roll.txt', delimiter=',')[:,:4]

if tr == '1':
    # A = np.tile(np.array([1.,-1.]), (600*1./stepSize,1))
    A = np.concatenate(  (np.array([[1.,  -1.] for _ in range(int(800*1./stepSize))]), 
            np.array([[ -1., 1.] for _ in range(int(200*1./stepSize))])), axis=0 )
if tr == '2':
    A = np.concatenate( (np.array([[-1.,  1.] for _ in range(int(300*1./stepSize))]), 
            np.array([[ 1., -1.] for _ in range(int(300*1./stepSize))]),
            np.array([[-1., 1.] for _ in range(int(300*1./stepSize))]) ), axis=0 )
if tr == '3':
    A = np.concatenate( (np.array([[1.,  -1.] for _ in range(int(200*1./stepSize))]), 
            np.array([[1.,  1.] for _ in range(int(130*1./stepSize))]), 
            np.array([[ -1., 1.] for _ in range(int(250*1./stepSize))])), axis=0 )
if tr == '4':
    A = np.concatenate( (np.array([[-1.,  1.] for _ in range(int(200*1./stepSize))]), 
            np.array([[1.,  1.] for _ in range(int(130*1./stepSize))]), 
            np.array([[ 1., -1.] for _ in range(int(250*1./stepSize))]),
            np.array([[ -1., -1.] for _ in range(int(250*1./stepSize))])), axis=0 )
if tr == '5':
    # A = np.tile(np.array([-1.,1.]), (800*1./stepSize,1))
    A = np.concatenate(  (np.array([[-1.,  1.] for _ in range(int(800*1./stepSize))]), 
            np.array([[ 1., -1.] for _ in range(int(200*1./stepSize))])), axis=0 )


if 0:
    Af = A.reshape((-1,))
    Pro = []
    for j in range(10):
        print("Rollout number " + str(j) + ".")
        
        roll = rollout_srv(Af)
        R = np.array(roll.states).reshape(-1,state_dim)
        suc = roll.success

        Pro.append(R)
        
        with open(path + 'rollout_' + tr + '_v' + str(version) + '_d' + str(state_dim) + '_m' + str(stepSize) + '.pkl', 'w') as f: 
            pickle.dump([Pro, suc], f)
with open(path + 'rollout_' + tr + '_v' + str(version) + '_d' + str(state_dim) + '_m' + str(stepSize) + '.pkl', 'r') as f:  
    Pro, _ = pickle.load(f) 



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

print('Smoothing data...')
for R in Pro:
    for i in range(4):
        R[:,i] = medfilter(R[:,i], 100)
# for i in range(2,4):
#     R[:,i] = medfilter(R[:,i], 40)

Smean = []
Sstd = []
for i in range(A.shape[0]-1):
    F = []
    for j in range(len(Pro)): 
        if Pro[j].shape[0] < 100:
            continue
        if Pro[j].shape[0] > i:
            F.append(Pro[j][i])
    Smean.append( np.mean(np.array(F), axis=0) )
    Sstd.append( np.std(np.array(F), axis=0) )
Smean = np.array(Smean)
Sstd = np.array(Sstd)

# s_start = Smean[0,:]
# sigma_start = Sstd[0,:]

for R in Pro:
    if R.shape[0] < 100:
        continue
    plt.plot(R[:,0], R[:,1], '.-k')
    plt.plot(R[0,0], R[0,1], 'or')
    # plt.plot(Smean[:,0],Smean[:,1], '-b')
    # # plt.show()

    s_start = R[1,:]
    sigma_start = np.ones((1,4))*1e-5


    if 1:   
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
            print("[GP] Step " + str(i) + " of " + str(A.shape[0]))
            Pgp.append(S)
            a = A[i,:]

            st = time.time()
            res = gp_srv(S.reshape(-1,1), a)
            t_gp += (time.time() - st) 

            S_next = np.array(res.next_states).reshape(-1,state_dim)
            if res.node_probability < p_gp:
                p_gp = res.node_probability
            s_mean_next = np.mean(S_next, 0)
            s_std_next = np.std(S_next, 0)
            S = S_next

            if S_next.shape[0] == 0:
                break

            # s_mean_next = np.array([0,0,0,0])
            # s_std_next = np.array([0,0,0,0])

            Ypred_mean_gp = np.append(Ypred_mean_gp, s_mean_next.reshape(1,state_dim), axis=0)
            Ypred_std_gp = np.append(Ypred_std_gp, s_std_next.reshape(1,state_dim), axis=0)

        t_gp /= A.shape[0]
        

        ######################################## naive propagation ###############################################

        print "Running Naive."
        Np = 1 # Number of particles
        t_naive = 0

        s = np.copy(s_start) + np.random.normal(0, sigma_start)
        # s = np.tile(s, (Np,1)) + np.random.normal(0, sigma_start, (Np, state_dim))
        Ypred_naive = s.reshape(1,state_dim)

        print("Running (open loop) path...")
        p_naive = 1
        for i in range(0, A.shape[0]):
            print("[Naive] Step " + str(i) + " of " + str(A.shape[0]))
            a = A[i,:]

            st = time.time()
            res = naive_srv(s.reshape(-1,1), a)
            t_naive += (time.time() - st) 

            if res.node_probability < p_naive:
                p_naive = res.node_probability
            s_next = np.array(res.next_state)
            s = s_next

            Ypred_naive = np.append(Ypred_naive, s_next.reshape(1,state_dim), axis=0)

        t_naive /= A.shape[0]



        plt.plot(Ypred_mean_gp[:,0], Ypred_mean_gp[:,1], '.-r', label='BPP')
        plt.plot(Ypred_naive[:,0], Ypred_naive[:,1], '.-c', label='Naive')
plt.axis('equal')
# plt.legend()


plt.savefig('/home/pracsys/catkin_ws/src/t42_control/gpup_gp_node/src/results/path_all_' + tr + '.png', dpi=300) #str(np.random.randint(100000))
plt.show()

