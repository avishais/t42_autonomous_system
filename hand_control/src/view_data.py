#!/usr/bin/env python

import numpy as np
import time
import random
import pickle
import os.path
import matplotlib.pyplot as plt
from transition_experience import *




def main():
    texp = transition_experience(Load = True, discrete=True, postfix='bu')

<<<<<<< HEAD
    # texp.save_to_file()
=======
    # texp.process_transition_data(stepSize = 1, plot = True)
    texp.process_svm(stepSize = 1)
>>>>>>> d8bdba09fb630f5cb93340f38caebb1b23810fd9

    # texp.divide_and_save(n = 750)

    texp.plot_data()

    return 1


if __name__=='__main__':
    main()