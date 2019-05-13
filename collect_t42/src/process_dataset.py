#!/usr/bin/env python

import numpy as np
import time
import random
import pickle
import os.path
import matplotlib.pyplot as plt
from transition_experience import *
import glob

import sys
sys.path.insert(0, '/home/pracsys/catkin_ws/src/t42_control/gpup_gp_node/src/')
from data_load import data_load
from svm_class import svm_failure

path = '/home/pracsys/catkin_ws/src/t42_control/hand_control/data/dataset/'
dest_path = '/home/pracsys/catkin_ws/src/t42_control/gpup_gp_node/data/dataset_processed/'

def get_objs(discrete=True):
    files = glob.glob(path + "*.obj")

    F = []
    for file in files:
        if file.find('_c_' if discrete else '_d_') > 0:
            continue
        obj = file[file.find('raw_')+4:file.find('_d_' if discrete else '_c_')]
        F.append(obj)

    return F

def check_opt(self, obj):
    files = glob.glob(dest_path + "*.obj")

    for file in files:
        if file.find('t42_' + obj + '_opt') > 0:
            return True

    return False


def main():
    discrete = True
    objs = get_objs(discrete)

    # Process raw data
    if 1:
        for obj in objs:
            # if np.any(obj == np.array(['sqr30','poly10','cyl35','poly6'])):
            #     continue
            print("\n\n[process_dataset] Processing object '%s'...\n\n"%obj)
            texp = transition_experience(Load = True, discrete=discrete, postfix='', Object = obj)

            texp.process_transition_data(stepSize = 1, plot = False)
            texp.process_svm(stepSize = 1)

    # Generate kd-trees and pre-compute hyper-parameters
    if 0:
        for obj in objs:
            print("\n\n[process_dataset] Computing hyper-parameters for object '%s'...\n\n"%obj)
            DL = data_load(simORreal = 't42_' + obj, discreteORcont = ('discrete' if discrete else 'cont'), K = 100)    

            SVM = svm_failure(simORreal = 't42_' + obj, discrete = discrete)   

    return 1


if __name__=='__main__':
    main()