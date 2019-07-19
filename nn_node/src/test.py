#!/usr/bin/env python

""" 
Author: Avishai Sintov
"""

import rospy
import numpy as np
import matplotlib.pyplot as plt
import time

import pickle
import random

from gpup_gp_node_exp.srv import batch_transition, one_transition


b_srv = rospy.ServiceProxy('/nn/transition', batch_transition)
o_srv = rospy.ServiceProxy('/nn/transitionOneParticle', one_transition)

rospy.init_node('test', anonymous=True)

if 1:
    task = 'real_A' 
    held_out = .1
    test_traj = 2
    nn_type = '1'
    method = ''
    save_path = 'save_model/robotic_hand_real/pytorch/'

    trajectory_path_map = {
        'real_A': '/home/pracsys/catkin_ws/src/t42_control/hand_control/data/dataset/testpaths_cyl35_d_v0.pkl', 
        'real_B': 'data/robotic_hand_real/B/testpaths_cyl35_red_d_v0.pkl',
        'transferA2B': 'data/robotic_hand_real/B/testpaths_cyl35_red_d_v0.pkl',
        'transferB2A': 'data/robotic_hand_real/A/testpaths_cyl35_d_v0.pkl',
        }
    trajectory_path = trajectory_path_map[task]

    with open(trajectory_path, 'rb') as pickle_file:
        trajectory = pickle.load(pickle_file)#, encoding='latin1')

    def make_traj(trajectory, test_traj):
        acts = trajectory[0][test_traj][:-1]
        real_positions = trajectory[1][test_traj][:,[0,1,11,12]]
        return np.append(real_positions, acts, axis=1)

    # NN = predict_nn()
    state_dim = 4
    action_dim = 6

    traj = make_traj(trajectory, test_traj)
    true_states = traj[:,:state_dim]
    state = traj[0][:state_dim]
    start_state = np.copy(state)

    states = []
    BATCH = True
    # BATCH = False
    if BATCH:
        Np = 1000

        sigma_start = np.ones((1,state_dim))*1e-9
        S = np.tile(state, (Np,1)) + np.random.normal(0, sigma_start, (Np, state_dim))

        t = c = 0
        for i, point in enumerate(traj):
            print i
            states.append(np.mean(S, 0))
            action = point[state_dim:state_dim+action_dim]
            action = np.concatenate((action, start_state), axis=0)

            st = time.time()
            res = b_srv(S.reshape(-1,1), action)
            t += time.time() - st
            S_next = np.array(res.next_states).reshape(-1,state_dim)
            c += 1

            S = np.copy(S_next)

        print "Batch %d prediction time: "%Np, t / c
        states.append(np.mean(S, 0))
        states = np.array(states)
    else:
        t = c = 0
        for i, point in enumerate(traj):
            print i
            states.append(state)
            action = point[state_dim:state_dim+action_dim]
            action = np.concatenate((action, start_state), axis=0)

            st = time.time()
            state = o_srv(state.reshape(-1,1), action).next_state
            t += time.time() - st
            c += 1

            state = np.array(state)

        print "one prediction time: ", t / c
        states = np.stack(states, 0)

    if BATCH:
        plt.figure(1)
        plt.scatter(traj[0, 0], traj[0, 1], marker="*", label='start')
        plt.plot(traj[:, 0], traj[:, 1], color='blue', label='Ground Truth', marker='.')
        plt.plot(states[:, 0], states[:, 1], color='red', label='NN Prediction')
        plt.axis('scaled')
        plt.title('Bayesian NN Prediction -- pos Space')
        plt.legend()
        plt.show()
    else:
        plt.figure(1)
        plt.scatter(traj[0, 0], traj[0, 1], marker="*", label='start')
        plt.plot(traj[:, 0], traj[:, 1], color='blue', label='Ground Truth', marker='.')
        plt.plot(states[:, 0], states[:, 1], color='red', label='NN Prediction')
        plt.axis('scaled')
        plt.title('Bayesian NN Prediction -- pos Space')
        plt.legend()
        plt.show()