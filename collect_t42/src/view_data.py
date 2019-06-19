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
    texp = transition_experience(Load = True, discrete=True, postfix='', Object = 'egg50', with_fingers = False)

    # texp.process_transition_data(stepSize = 1, plot = True)
=======
<<<<<<< HEAD
    texp = transition_experience(Load = True, discrete=True, postfix='', Object = 'cre55', with_fingers = False)
=======
    texp = transition_experience(Load = True, discrete=True, postfix='')#, Object = 'poly10', with_fingers = False)
>>>>>>> ca3faf2af00b3cf67f94f8f2e0714105e715b228

    texp.process_transition_data(stepSize = 1, plot = True)
>>>>>>> 18b0a07bea923f4bf684fa5fc4450cc097a068b9
    texp.process_svm(stepSize = 1)

    texp.plot_data()

    return 1


if __name__=='__main__':
    main()