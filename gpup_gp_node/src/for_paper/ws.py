#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Ellipse, Polygon
import pickle
import random

Obj = 'cyl35'
version = 0
w = 23.4

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

fig, ax = plt.subplots(figsize=(12,6))

with open(data, 'rb') as f: 
    D, _, _, _, _ = pickle.load(f)

D[:,0] -= w
# plt.plot(D[:,0], D[:,1], '.m')

# from scipy.spatial import ConvexHull, convex_hull_plot_2d
# hull = ConvexHull(D[:,:2])
# H1 = []
# for simplex in hull.vertices:
#     H1.append(D[simplex, :2])
H1 = np.array([[109.43661928,  45.45193231],
    [109.50734019,  45.48163721],
    [109.55533981,  45.51312672],
    [109.66525674,  45.61027214],
    [109.76466537,  45.71999609],
    [109.7899437 ,  45.7532702 ],
    [109.818762  ,  45.8920449 ],
    [103.75124216,  63.26119602],
    [101.20272636,  68.47952306],
    [ 91.55581395,  85.36463135],
    [ 82.23831654,  95.83953023],
    [ 56.97445869, 114.53938466],
    [ 55.3663969 , 115.56509882],
    [ 40.51083326, 119.81970072],
    [ 37.26506233, 119.99034882],
    [  9.94575024, 119.83285844],
    [ -9.41121578, 113.63378167],
    [-26.0669589 , 102.4928391 ],
    [-39.59908932,  88.99509944],
    [-39.83750641,  88.75419796],
    [-39.90573734,  88.68383244],
    [-39.97552544,  88.6100959 ],
    [-40.42713046,  88.1207196 ],
    [-41.84839576,  86.53383218],
    [-43.51329803,  84.62289628],
    [-51.62924528,  75.13655722],
    [-56.84250593,  64.16174769],
    [-62.06816435,  39.81572762],
    [-61.94485589,  39.46453519],
    [-61.91357613,  39.40508559],
    [-61.40811145,  38.84640774],
    [-61.20847613,  38.67814187],
    [-60.8488217 ,  38.51114213],
    [-60.75637937,  38.47118448],
    [-60.48326939,  38.43883537],
    [-60.46220064,  38.43718059],
    [-60.29364914,  38.4239424 ],
    [-60.07242203,  38.40656728],
    [-59.83011425,  38.38753738]])
H2 = np.array([[-44,55],[-26,68],[-12,76],[23,86],[47,78],[65,69],[81,62],[97,50]])
H = np.concatenate((np.array(H1), H2), axis=0)
H[:,0] -= w
pgon = plt.Polygon(H, color=(169./255, 143./255, 0), alpha=1, zorder=0)
ax.add_patch(pgon)

# test_path = '/home/pracsys/catkin_ws/src/t42_control/rollout_t42/manual_rolls/'
# Pro = []
# Aro = []
# for i in range(11):
#     try:
#         with open(test_path + 'manual_path_' + Obj + '_' + str(i) + '.pkl', 'r') as f: 
#             S, A = pickle.load(f)
#     except:
#         continue

#     if i == 0:
#         S = S[:-30,:]
#         A = A[:-30,:]
#     if i == 7:
#         S = S[:-34,:]
#         A = A[:-34,:]

#     Pro.append(S)
#     Aro.append(A)

#     for i in range(2):
#         S[:,i] = medfilter(S[:,i], 15)
#     plt.plot(S[:,0]-w, S[:,1])
# Suc = 0
# with open(test_path + 'testpaths_' + Obj + '_d_v' + str(1) + '.pkl', 'w') as f: 
#     pickle.dump([Aro, Pro, Obj, Suc], f)

# plt.axis('equal')
# plt.xlim([-86,86])
# plt.ylim([37,122])
# plt.xlabel('x', fontsize=16)
# plt.ylabel('y', fontsize=16)
# plt.savefig(test_path + 'ws_' + Obj + '.png', dpi=300) #str(np.random.randint(100000))
# # plt.show()

test_path = '/home/pracsys/catkin_ws/src/t42_control/hand_control/data/dataset/'
with open(test_path + 'testpaths_' + Obj + '_d_v' + str(1) + '.pkl', 'r') as f: 
    action_seq, test_paths, Obj, _ = pickle.load(f)

for S in test_paths:
    print S[:,:2]
    for i in range(2):
        S[:,i] = medfilter(S[:,i], 15)
    plt.plot(S[:,0]-w, S[:,1], linewidth = 2.5)#, color=(random.random(),random.random(),random.random()))
for S in test_paths:
    plt.plot(S[0,0]-w, S[0,1], 'ob')   
plt.axis('equal')
plt.xlim([-86,86])
plt.ylim([37,122])
plt.xlabel('x (mm)', fontsize=16)
plt.ylabel('y (mm)', fontsize=16)
plt.savefig(test_path + 'ws_' + Obj + '.png', dpi=300) #str(np.random.randint(100000))
# plt.show()
