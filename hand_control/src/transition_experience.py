
import numpy as np
import pickle
import os.path
import matplotlib.pyplot as plt
from scipy.io import savemat
import scipy.signal

class transition_experience():
    path = '/home/pracsys/catkin_ws/src/t42_control/hand_control/data/'

    def __init__(self, Load=True, discrete = True, postfix=''):

        if discrete:
            self.mode = 'd' # Discrete actions
        else:
            self.mode = 'c' # Continuous actions
        
        self.postfix = postfix
        self.file_name = self.path + 'raw_35_' + self.mode + '_v4' + self.postfix + '.obj'

        if Load:
            self.load()
        else:
            self.clear()
        
    def add(self, state, action, next_state, done):
        self.memory += [(state, action, next_state, done)]
        
    def clear(self):
        self.memory = []

    def load(self):
        if os.path.isfile(self.file_name):
            print('Loading data from ' + self.file_name)
            with open(self.file_name, 'rb') as filehandler:
            # filehandler = open(self.file_name, 'r')
                self.memory = pickle.load(filehandler)
            print('Loaded transition data of size %d.'%self.getSize())
        else:
            self.clear()

    def getComponents(self):

        states = np.array([item[0] for item in self.memory])
        actions = np.array([item[1] for item in self.memory])
        next_states = np.array([item[2] for item in self.memory])

        return states, actions, next_states

    def save(self):
        print('Saving data...')
        file_pi = open(self.file_name, 'wb')
        pickle.dump(self.memory, file_pi)
        print('Saved transition data of size %d.'%self.getSize())
        file_pi.close()

    def getSize(self):
        return len(self.memory)

    def plot_data(self):

        states = np.array([item[0] for item in self.memory])
        # states[:,:2] *= 1000.
        done = np.array([item[3] for item in self.memory])
        failed_states = states[done]

        plt.figure(1)
        ax1 = plt.subplot(121)
        # ax1.plot(states[:,0],states[:,1],'-k')
        ax1.plot(states[:,0],states[:,1],'.y')
        ax1.plot(failed_states[:,0],failed_states[:,1],'.r')
        ax1.set(title='Object position')
        ax1.axis('equal')
        # plt.ylim((0.06, 0.12))
        # plt.xlim((-0.03, 0.11))
        
        ax2 = plt.subplot(122)
        ax2.plot(states[:,2],states[:,3],'.k')
        ax2.plot(failed_states[:,2],failed_states[:,3],'.r')
        ax2.set(title='Actuator loads')
        
        plt.show()

    def save_to_file(self):

        filen = self.path + 'transition_data_' + self.mode + '.db'

        n = self.getSize()

        states = np.array([item[0] for item in self.memory])
        actions = np.array([item[1] for item in self.memory])
        next_states = np.array([item[2] for item in self.memory])
        done = np.array([item[3] for item in self.memory])

        inx = np.where(done)

        M = np.concatenate((states, actions, next_states), axis=1)
        M = np.delete(M, inx, 0)

        np.savetxt(filen, M, delimiter=' ')

    
    def process_transition_data(self, stepSize = 1, plot = False):

        def smooth(D, done):
            print('[transition_experience] Smoothing data...')

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

            ks = 0
            kf = 1
            while kf < D.shape[0]:
                if kf >= D.shape[0]:
                    break 

                # Idenfify end of episode
                while not done[kf]:
                    kf += 1
                # print ks, kf
                
                # Aplly filter to episode
                for i in range(4):
                    D[ks:kf+1,i] = medfilter(D[ks:kf+1,i], 20)
                                
                # Update next state columns
                D[ks:kf, 6:10] = D[ks+1:kf+1, 0:4]

                ks = kf+1
                kf += 1  

            return D
           
        def clean(D, done):
            print('[transition_experience] Cleaning data...')

            i = 0
            inx = []
            while i < D.shape[0]:
                if i > 0 and np.linalg.norm( D[i, 0:2] - D[i-1, 0:2] ) > 1 and not done[i] and not done[i-1]:
                    i += 1
                    continue
                if D[i,0] < -50. or D[i,0] > 120:
                    i += 1
                    continue
                if np.linalg.norm( D[i, 0:2] - D[i, 6:8] ) <= 1 and not np.all(D[i,4:6] == 0.0):
                    inx.append(i)
                i += 1

            return D[inx,:], done[inx]

        def multiStep(D, done, stepSize): 
            Dnew = []
            ia = range(4,6)
            for i in range(D.shape[0]-stepSize):
                a = D[i, ia] 
                if not np.all(a == D[i:i+stepSize+1, ia]) or np.any(done[i:i+stepSize+1]):
                    continue

                Dnew.append( np.concatenate((D[i,:ia[0]], a, D[i+stepSize, ia[-1]+1:]), axis=0) )

            return np.array(Dnew)

        print('[transition_experience] Saving transition data...')
        is_start = 1
        is_end = 277

        states = np.array([item[0] for item in self.memory])
        states[:,:2] *= 1000.
        actions = np.array([item[1] for item in self.memory])
        next_states = np.array([item[2] for item in self.memory])
        next_states[:,:2] *= 1000.
        done = np.array([item[3] for item in self.memory]) 

        # For data from recorder
        if self.postfix == 'bu':
            next_states = np.roll(states, -1, axis=0) 

        self.state_dim = states.shape[1]

        for i in range(done.shape[0]):
            if done[i]:
                done[i-2:i] = True

        D = np.concatenate((states, actions, next_states), axis = 1)
        D, done = clean(D, done)

        # Start dist.
        St = [D[0,:4]]
        for i in range(D.shape[0]-1):
            if done[i] and not done[i+1]:
                St.append(D[i+1,:4])
        St = np.array(St)
        s_start = np.mean(St, 0)
        s_std = np.std(St, 0)
        print "start mean: ", s_start
        print "start std.: ", s_std

        # plt.plot(D[:,2], D[:,3], '.k')
        # plt.plot(St[:,2], St[:,3], '.r')
        # plt.plot(s_start[2], s_start[3], 'og')
        # plt.show()
        # exit(1)

        D = smooth(D, done)

        if stepSize > 1:
            D = multiStep(D, done, stepSize)

        inx = np.where(done)
        D = np.delete(D, inx, 0) # Remove drop transitions
        done = np.delete(done, inx, 0)
              
        self.D = D

        # Bounds
        print "Max: ", np.max(D, 0)
        print "Min: ", np.min(D, 0)

        is_start = 60000
        is_end = is_start+100
        # plt.plot(D[is_start,0], D[is_start,1],'or')
        # plt.plot(D[is_start:is_end,0], D[is_start:is_end,1],'.-b')
        # plt.show()
        # exit(1)

        savemat(self.path + 't42_35_data_discrete_v4_d4_m' + str(stepSize) + '.mat', {'D': D, 'is_start': is_start, 'is_end': is_end})
        savemat('/home/pracsys/catkin_ws/src/beliefspaceplanning/gpup_gp_node/data/' + 't42_35_data_discrete_v4_d4_m' + str(stepSize) + '.mat', {'D': D, 'is_start': is_start, 'is_end': is_end})
        print "Saved mat file with " + str(D.shape[0]) + " transition points."

        if plot:
            plt.figure(0)
            plt.plot(D[:,0], D[:,1],'ok')
            # for _ in range(1000):
            #     j = np.random.randint(D.shape[0])
            #     plt.plot([D[j,0], D[j,6]], [D[j,1], D[j,7]],'o-r')
            plt.title(str(stepSize))
            plt.show()
    
    def process_svm(self, stepSize = 1):

        def clean_done(states, done):
            # Cancel drop mark if episode is very short
            i = 0
            while i < states.shape[0]-1:
                j = i + 1
                while j < states.shape[0] and not done[j]:
                    j +=1
                if done[j] and j-i < 10:
                    done[j] = False 
                i = j

            # Cancel drop if load is not critical
            for i in range(states.shape[0]):
                if done[i] and np.all(np.abs(states[i, 2:]) < 260) and np.all(np.abs(states[i, 2:]) > 40):
                    if np.random.uniform() > 0.5:
                        done[i] = False

            return done

        def multiStep(D, done, stepSize): 
            Dnew = []
            done_new = []
            ia = range(4,6)
            for i in range(D.shape[0]-stepSize):
                a = D[i, ia] 
                if not np.all(a == D[i:i+stepSize+1, ia]):
                    continue
                
                if np.any(done[i:i+stepSize+1]):
                    done_new.append(True)
                else:
                    done_new.append(False)

                Dnew.append( np.concatenate((D[i,:ia[0]], a), axis=0) )

            return np.array(Dnew), np.array(done_new)

        from sklearn import svm
        from sklearn.preprocessing import StandardScaler

        states = np.array([item[0] for item in self.memory])
        states[:,:2] *= 1000.
        actions = np.array([item[1] for item in self.memory])
        done = np.array([item[3] for item in self.memory])

        done = clean_done(states, done)

        for i in range(done.shape[0]):
            if done[i]:
                done[i-2:i] = True

        SA = np.concatenate((states, actions), axis=1)
        SA, done = multiStep(SA, done, stepSize)
        print('Transition data with steps size %d has now %d points'%(stepSize, SA.shape[0]))

        inx_fail = np.where(done)[0]
        print "Number of failed states " + str(inx_fail.shape[0])
        T = np.where(np.logical_not(done))[0]
        inx_suc = T[np.random.choice(T.shape[0], inx_fail.shape[0], replace=False)]
        SA = np.concatenate((SA[inx_fail], SA[inx_suc]), axis=0)
        done = np.concatenate((done[inx_fail], done[inx_suc]), axis=0)

        with open(self.path + 't42_35_svm_data_' + self.mode + '_v4_d4_m' + str(stepSize) + '.obj', 'wb') as f: 
            pickle.dump([SA, done], f)
        with open('/home/pracsys/catkin_ws/src/beliefspaceplanning/gpup_gp_node/data/' + 't42_35_svm_data_' + self.mode + '_v4_d4_m' + str(stepSize) + '.obj', 'wb') as f: 
            pickle.dump([SA, done], f)
        # savemat(self.path + 't42_35_svm_data_' + self.mode + '_v0_d4_m' + str(stepSize) + '.mat', {'SA': SA, 'done': done})
        print('Saved svm data.')

        ##  Test data
        # Normalize
        scaler = StandardScaler()
        SA = scaler.fit_transform(SA)
        x_mean = scaler.mean_
        x_std = scaler.scale_

        # Test data
        ni = int(np.floor(0.1*inx_fail.shape[0]))
        T = np.where(done)[0]
        inx_fail = T[np.random.choice(T.shape[0], ni, replace=False)]
        T = np.where(np.logical_not(done))[0]
        inx_suc = T[np.random.choice(T.shape[0], ni, replace=False)]
        SA_test = np.concatenate((SA[inx_fail], SA[inx_suc]), axis=0)
        done_test = np.concatenate((done[inx_fail], done[inx_suc]), axis=0)

        SA = np.delete(SA, inx_fail, axis=0)
        SA = np.delete(SA, inx_suc, axis=0)
        done = np.delete(done, inx_fail, axis=0)
        done = np.delete(done, inx_suc, axis=0)

        print 'Fitting SVM...'
        clf = svm.SVC( probability=True, class_weight='balanced', C=1.0 )
        clf.fit( list(SA), 1*done )
        print 'SVM fit with %d classes: '%len(clf.classes_) + str(clf.classes_)

        s = 0
        s_suc = 0; c_suc = 0
        s_fail = 0; c_fail = 0
        for i in range(SA_test.shape[0]):
            p = clf.predict_proba(SA_test[i].reshape(1,-1))[0]
            fail = p[1]>0.5
            # print p, done_test[i], fail
            s += 1 if fail == done_test[i] else 0
            if done_test[i]:
                c_fail += 1
                s_fail += 1 if fail else 0
            else:
                c_suc += 1
                s_suc += 1 if not fail else 0
        print 'Success rate: ' + str(float(s)/SA_test.shape[0]*100)
        print 'Drop prediction accuracy: ' + str(float(s_fail)/c_fail*100)
        print 'Success prediction accuracy: ' + str(float(s_suc)/c_suc*100)

