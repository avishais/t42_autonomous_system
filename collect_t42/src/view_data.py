#!/usr/bin/env python

import numpy as np
import time
import random
import pickle
import os.path
import matplotlib.pyplot as plt
from transition_experience import *


def main():
<<<<<<< HEAD
    texp = transition_experience(Load = True, discrete=False, postfix='', Object = 'str40', with_fingers = False)
=======

    texp = transition_experience(Load = True, discrete=True, postfix='', Object = 'sem60', with_fingers = False)
>>>>>>> 7685905aea88becb45a7e61f84ae011f2497089d

    # texp.process_transition_data(stepSize = 1, plot = True)
    # texp.process_svm(stepSize = 1)

    # texp.plot_data()

    return 1


if __name__=='__main__':
    main()