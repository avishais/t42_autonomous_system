#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Ellipse, Polygon
import pickle

Obj = 'elp40'
version = 0
test_path = '/home/pracsys/catkin_ws/src/t42_control/hand_control/data/dataset/'

if np.any(Obj == np.array(['sqr30','poly10','poly6','elp40','tri50','egg50','str40','rec60','rec10'])): # Include orientation angle
    dim_ = '5'
else:
    dim_ = '4'

data = '/home/pracsys/catkin_ws/src/t42_control/gpup_gp_node/data/dataset_processed/t42_' + Obj + '_data_discrete_v0_d' + dim_ + '_m1.obj'

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


with open(data, 'rb') as f: 
    D, _, _, _, _ = pickle.load(f)

plt.plot(D[:,0], D[:,1], '.y')


with open(test_path + 'testpaths_' + Obj + '_d_v' + str(version) + '.pkl', 'r') as f: 
        action_seq, test_paths, Obj, Suc = pickle.load(f)

for S in test_paths:
    for i in range(2):
        S[:,i] = medfilter(S[:,i], 15)
    plt.plot(S[:,0], S[:,1])
plt.show()