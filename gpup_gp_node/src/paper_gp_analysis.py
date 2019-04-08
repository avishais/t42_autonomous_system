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
tr = '5'
stepSize = var.stepSize_
version = var.data_version_

gp_srv = rospy.ServiceProxy('/gp/transition', batch_transition)
naive_srv = rospy.ServiceProxy('/gp/transitionOneParticle', one_transition)
rollout_srv = rospy.ServiceProxy('/rollout/rollout', rolloutReq)

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
    A = np.tile(np.array([-1.,1.]), (800*1./stepSize,1))
    # A = np.concatenate(  (np.array([[-1.,  1.] for _ in range(int(800*1./stepSize))]), 
    #         np.array([[ 1., -1.] for _ in range(int(200*1./stepSize))])), axis=0 )


if 1:
    Af = A.reshape((-1,))
    Pro = []
    for j in range(2):
        print("Rollout number " + str(j) + ".")
        
        roll = rollout_srv(Af)
        R = np.array(roll.states).reshape(-1,state_dim)
        suc = roll.success

        if R.shape[0] < 10:
            continue

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
# for R in Pro:
#     for i in range(4):
#         R[:,i] = medfilter(R[:,i], 100)
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

R = Pro[0]
s_start = R[0,:]
sigma_start = np.ones((1,4))*1e-3

for R in Pro:
    print R.shape
    if R.shape[0] < 2:
        continue
    plt.plot(R[:,0], R[:,1], '.-k')
    plt.plot(R[0,0], R[0,1], 'or')
# plt.plot(Smean[:,0],Smean[:,1], '-b')
plt.show()
exit(1)


rospy.init_node('verification_t42', anonymous=True)


if 0:   
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

    ######################################## Mean propagation ##################################################

    print "Running Batch Mean."
    Np = 100 # Number of particles

    t_mean = 0

    s = np.copy(s_start)
    S = np.tile(s, (Np,1))
    Ypred_bmean = s.reshape(1,state_dim)

    print("Running (open loop) path...")
    p_mean = 1
    for i in range(0, 2+0*A.shape[0]):
        print("[Mean] Step " + str(i) + " of " + str(A.shape[0]))
        a = A[i,:]

        st = time.time()
        res = gp_srv(S.reshape(-1,1), a)
        t_mean += (time.time() - st) 

        if res.node_probability < p_mean:
            p_mean = res.node_probability
        S_next = np.array(res.next_states).reshape(-1,state_dim)
        s_mean_next = np.mean(S_next, 0)
        S = np.tile(s_mean_next, (Np,1))

        Ypred_bmean = np.append(Ypred_bmean, s_mean_next.reshape(1,state_dim), axis=0)

    t_mean /= A.shape[0]

    ######################################## Save ###########################################################

    stats = np.array([[t_gp, t_naive, t_mean], [p_gp, p_naive, p_mean]])

    with open(path + 'ver_t42_pred_' + tr + '_v' + str(version) + '_d' + str(state_dim) + '_m' + str(stepSize) + '.pkl', 'w') as f:
        pickle.dump([Ypred_mean_gp, Ypred_std_gp, Pgp, Ypred_naive, Ypred_bmean, stats, A], f)

######################################## Plot ###########################################################


with open(path + 'ver_t42_pred_' + tr + '_v' + str(version) + '_d' + str(state_dim) + '_m' + str(stepSize) + '.pkl') as f:  
    Ypred_mean_gp, Ypred_std_gp, Pgp, Ypred_naive, Ypred_bmean, stats, A = pickle.load(f)  

# Compare paths
# d_gp = d_gpup = d_naive = d_mean = d = 0.
# for i in range(A.shape[0]):
#     if i < Smean.shape[0]-1:
#         d += np.linalg.norm(Smean[i,:]-Smean[i+1,:])
#     d_gp += np.linalg.norm(Ypred_mean_gp[i,:] - Smean[i,:])
#     # d_naive += np.linalg.norm(Ypred_bmean[i,:] - Smean[i,:])
#     # d_mean += np.linalg.norm(Ypred_naive[i,:] - Smean[i,:])
# d_gp = np.sqrt(d_gp/A.shape[0])
# # d_naive = np.sqrt(d_naive/A.shape[0])
# # d_mean = np.sqrt(d_mean/A.shape[0])

# print "-----------------------------------"
# print "Path length: " + str(d)
# print "-----------------------------------"
# print "GP rmse: " + str(d_gp) + "mm"
# # print "Naive rmse: " + str(d_naive) + "mm"
# # print "mean rmse: " + str(d_mean) + "mm"
# print "-----------------------------------"
# print "GP runtime: " + str(stats[0][0]) + "sec."
# # print "GP Naive: " + str(stats[0][2]) + "sec."
# print "GP mean: " + str(stats[0][1]) + "sec."
# print "-----------------------------------"
# print "GP probability: " + str(stats[1][0])
# print "GP naive probability: " + str(stats[1][1])
# print "GP mean probability: " + str(stats[1][2])
# print "-----------------------------------"

# Animate
if 0:
    fig = plt.figure(0)
    ax = fig.add_subplot(111)#, aspect='equal')
    # for j in range(len(Pro)): 
    #     Sro = Pro[j]
    #     ax.plot(Sro[:,0], Sro[:,1], '.-b')

    prtc_mean_line, = ax.plot([], [], '-g')
    prtc, = ax.plot([], [], '.k', markersize=1)

    sm, = ax.plot([], [], 'ok', markerfacecolor='r', markersize=8)

    prtc_mean, = ax.plot([], [], '*g')
    patch_prtc = Ellipse(xy=(Ypred_mean_gp[0,0], Ypred_mean_gp[0,1]), width=Ypred_std_gp[0,0]*2, height=Ypred_std_gp[0,1]*2, angle=0., animated=True, edgecolor='y', linewidth=2., fill=False)
    ax.add_patch(patch_prtc)

    naive, = ax.plot([], [], '-k')

    # plt.xlim(np.min(Ypred_mean_gp, 0)[0]*0-5, np.max(Ypred_mean_gp, 0)[0]*1.0)
    # plt.ylim(np.min(Ypred_mean_gp, 0)[1]*0.99, np.max(Ypred_mean_gp, 0)[1]*1.01)

    def init():
        prtc.set_data([], [])
        prtc_mean.set_data([], [])
        prtc_mean_line.set_data([], [])
        # sm.set_data([], [])
        naive.set_data([], [])

        return prtc, prtc_mean, prtc_mean_line, patch_prtc, naive,

    def animate(i):

        S = Pgp[i]
        prtc.set_data(S[:,0], S[:,1])
        # print i, len(idx_g), len(idx_f)

        # sm.set_data(Smean[i][0], Smean[i][1])

        prtc_mean.set_data(Ypred_mean_gp[i,0], Ypred_mean_gp[i,1])
        prtc_mean_line.set_data(Ypred_mean_gp[:i+1,0], Ypred_mean_gp[:i+1,1])

        patch_prtc.center = (Ypred_mean_gp[i,0], Ypred_mean_gp[i,1])
        patch_prtc.width = Ypred_std_gp[i,0]*2
        patch_prtc.height = Ypred_std_gp[i,1]*2

        naive.set_data(Ypred_naive[:i+1,0], Ypred_naive[:i+1,1])

        return prtc, prtc_mean, prtc_mean_line, patch_prtc, naive,

    ani = animation.FuncAnimation(fig, animate, frames=len(Pgp), init_func=init, interval=80, repeat_delay=1000, blit=True)
    # ani.save(path + 'belief_gazebo_' + str(tr) + '_v5.mp4', metadata={'artist':'Avishai Sintov','year':'2019'}, bitrate=-1, codec="libx264")

    # plt.figure(1)
    # for k in range(4):
    #     ax1 = plt.subplot(2,2,k+1)
    #     for i in range(len(Pgp)):
    #         S = Pgp[i]
    #         ax1.plot(S[:,k], '-b')
    #     for i in range(len(Pro)):
    #         S = Pro[i]
    #         ax1.plot(S[:,k], '-k')

    plt.show()


t = range(A.shape[0]+1)

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

# plt.figure(2)
# ax1 = plt.subplot(1,2,1)
try:
    plt.plot(R[:,ix[0]], R[:,ix[1]], '.-k', label='rollout')
except:
    pass
plt.plot(Ypred_mean_gp[:,ix[0]], Ypred_mean_gp[:,ix[1]], '.-r', label='BPP')
plt.plot(Ypred_naive[:,0], Ypred_naive[:,1], '.-c', label='Naive')
plt.axis('equal')
plt.legend()


# ax2 = plt.subplot(1,2,2)
# try:
#     plt.plot(R[:,ix[0]+2], R[:,ix[1]+2], '.-k', label='rollout')
# except:
#     pass
# plt.plot(Ypred_mean_gp[:,ix[0]+2], Ypred_mean_gp[:,ix[1]+2], '-r', label='BPP mean')
# plt.plot(Ypred_naive[:,2], Ypred_naive[:,3], '.-c', label='Naive')
# plt.legend(loc='best')

plt.savefig('/home/pracsys/catkin_ws/src/t42_control/gpup_gp_node/src/results/path_' + tr + '.png', dpi=300) #str(np.random.randint(100000))
plt.show()

