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
    texp = transition_experience(Load = True, discrete=False, postfix='', Object = 'poly10', with_fingers = False)

    # texp.process_transition_data(stepSize = 1, plot = True)
    # texp.process_svm(stepSize = 1)
=======
    texp = transition_experience(Load = True, discrete=True, postfix='', Object = 'cyl30', with_fingers = False)

    # texp.process_transition_data(stepSize = 1, plot = True)
    texp.process_svm(stepSize = 1)
>>>>>>> 714febbca4679450a4db95f1b74779234ae9d6d9

    # texp.plot_data()

    return 1


if __name__=='__main__':
    main()