import numpy as np
import math
import random
import time
import matplotlib.pyplot as plt
import target_class as t_c

#Classe per creare un Fading filter per stimare misure fino al terzo ordine di veriabili misurate con dimensione 3
# e con una temporizzazione definibile da costruttore
class Fading_Filter:
    def __init__(self, target, Nstd=3e-3, Dimensions=2, Order=2, Beta=0.5, dt=0.05):
        self.order = Order
        self.dt = dt
        self.target = target
        self.dim = Dimensions
        self.std = Nstd
        if Order <= 1 or Order > 3:
            print('it is not possible create a filter with this order')
            return
        self.pos_est = np.zeros(3, dtype='float')
        self.vel_est = np.zeros(3, dtype='float')
        #self.list_pos = self.pos_est.reshape((1,Dimensions))
        self.real_pos = 0
        self.real_vel = 0
        self.list_pos = 0
        self.list_vel = 0
        self.time_line = 0
        if Order > 1:
           # self.list_vel = self.vel_est
            self.G = 1 - math.pow(Beta, 2)
            self.H = math.pow((1 - Beta), 2)

        if Order > 2:
            self.acc_est = np.zeros(3, dtype='float')
           # self.list_acc = self.acc_est
            self.G = 1 - math.pow((1 - Beta), 3 )
            self.H = 1.5 * math.pow((1 - Beta), 2 )*(1 + Beta)
            self.K = 0.5 * math.pow((1 - Beta), 3 )
            self.real_acc = 0
            self.list_acc = 0

    #funzione che permette di inizializzare le stime iniziali passando un numpy con dimensione (Order,3) da
    def initial(self, initial_estimation):
        if initial_estimation.shape[0] != self.order or initial_estimation.shape[1] != self.pos_est.shape[0]:
            print('Incorrect Dimension of Initial Estimate Variablem Tensor')
            return False
        #elif init_real.shape[0] != self.order or init_real.shape[1] != self.pos_est.shape[0]:
        #    print('Incorrect Dimension of Initial Estimate Variablem Tensor')
        #    return False

        else:
            (p, v, a, time) = self.target.get_target()
            #print(p,v,a)
            self.pos_est = initial_estimation[0, :]
            self.list_pos = self.pos_est.reshape((1, 3))
            self.real_pos = p.reshape((1, 3))
            self.vel_est = initial_estimation[1, :]
            self.list_vel = self.vel_est.reshape((1, 3))
            self.real_vel = v.reshape((1, 3))

            if self.order == 3:
                self.acc_est = initial_estimation[2, :]
                self.list_acc = self.acc_est.reshape((1, 3))
                self.real_acc = a.reshape((1, 3))
            return True

    #aggiorna le stime con la nuova misura
    def compute(self, measure):
        if measure.shape[0] != self.pos_est.shape[0]:
            print('Incorrect Dimension of Measure Tensor')
            return False

        if type(self.list_pos).__module__ != np.__name__:
            print('Estimation not Initialized')
            return False

        if self.order == 2:
            int_pos_est = self.pos_est + self.dt * self.vel_est

            pos_est_suc = int_pos_est + self.G * (measure - int_pos_est)
            vel_est_suc = self.vel_est + (self.H/self.dt) * (measure - int_pos_est)

        if self.order == 3:
            int_pos_est = self.pos_est + self.dt * self.vel_est + 0.5 * self.acc_est * math.pow(self.dt,2)

            pos_est_suc = int_pos_est + self.G * ( measure - int_pos_est )
            vel_est_suc = self.vel_est + self.dt * self.acc_est + (self.H / self.dt) * (measure - int_pos_est )
            acc_est_suc = self.acc_est + ((2 * self.K)/math.pow(self.dt,2)) * (measure - int_pos_est)

            self.acc_est = acc_est_suc
            self.list_acc = np.concatenate((self.list_acc, self.acc_est.reshape(1, 3)), axis=0)

        self.pos_est = pos_est_suc
        self.vel_est = vel_est_suc
        self.list_pos = np.concatenate((self.list_pos, self.pos_est.reshape(1, 3)), axis=0)
        self.list_vel = np.concatenate((self.list_vel, self.vel_est.reshape(1, 3)), axis=0)

        return True

    #aggiorna le variabili reali per utilizzarle nel plot finale in modo da confrontarle con le stime
    def add_real_quantities(self, p, v, a):
        if type(self.real_pos).__module__ != np.__name__ :
            print('Not Initialized Real Quantities')
            return False
        self.real_pos = np.concatenate((self.real_pos,p.reshape(1,3)),axis=0)
        self.real_vel = np.concatenate((self.real_vel, v.reshape(1,3)), axis=0)
        if self.order == 3:
            self.real_acc = np.concatenate((self.real_acc, a.reshape(1, 3)), axis=0)
        return True

    # ritorna le stime ed aggiorna la sua temporizzazione
    def get_estimation(self):
        (p, v, a, time) = self.target.get_target()
        measure = p + np.array([random.gauss(0, self.std), random.gauss(0, self.std), random.gauss(0, self.std)])

        if not self.compute(measure):
            pass
        if not self.add_real_quantities(p, v, a):
            pass
        if time != 0.0:
            self.time_line = np.append(self.time_line, np.array([time]), axis=0)
        if self.order ==3:
            return (self.pos_est,self.vel_est,self.acc_est)
        else:
            return (self.pos_est,self.vel_est)

    def start(self):
        self.time_line = np.array([time.time()])
        t_c.run = True
        self.target.start()

    def stop(self):
        t_c.run = False
        self.target.stop()
        self.time_line -= self.time_line[0]

    def plot_Filter(self):
        legend=np.array([['px','py','pz'],['vx','vy','vz'],['ax','ay','az']])
        sp, ax = plt.subplots(3,2)
        sp.suptitle('Estimation vs Real Position')
        for i in range(3):
            ax[i,0].plot(self.time_line,self.real_pos[:,i],'-g',label=f'real {legend[0,i]}')
            ax[i,0].legend()
            ax[i,0].plot(self.time_line,self.list_pos[:,i],'--b',label=f'estimate {legend[0,i]}')
            ax[i,0].legend()
            error= self.list_pos[:,i]-self.real_pos[:,i]
            ax[i,1].plot(self.time_line,error,'-r',label=f'{legend[0,i]} error')
            ax[i, 1].legend()
        plt.show()
        sp, ax = plt.subplots(3,2)
        sp.suptitle('Estimation vs Real Velocity')
        for i in range(3):
            ax[i,0].plot(self.time_line, self.real_vel[:,i], '-r',label=f'real {legend[1,i]}')
            ax[i,0].legend()
            ax[i,0].plot(self.time_line, self.list_vel[:,i], '--b',label=f'estimate {legend[1,i]}')
            ax[i,0].legend()
            error = self.list_vel[:, i] - self.real_vel[:, i]
            ax[i, 1].plot(self.time_line, error, '-r', label=f'{legend[1, i]} error')
            ax[i, 1].legend()
        plt.show()
        if self.order == 3:
            sp, ax = plt.subplots(3,2)
            sp.suptitle('Estimation vs Real Acceleration')
            for i in range(3):
                ax[i,0].plot(self.time_line, self.real_acc[:,i], '-r',label=f'real {legend[2,i]}')
                ax[i,0].legend()
                ax[i,0].plot(self.time_line, self.list_acc[:,i], '--b',label=f'estimate {legend[2,i]}')
                ax[i,0].legend()
                error = self.list_acc[:, i] - self.real_acc[:, i]
                ax[i, 1].plot(self.time_line, error, '-r', label=f'{legend[2, i]} error')
                ax[i, 1].legend()

            plt.show()

'''delta = 0.05
in_p= np.array([1.0,1.0,0.5])
in_v = np.array([0.0,0.1,0.0])
in_a = 0.0
tg= t_c.target(initial_position=in_p,initial_velocity=in_v,initial_acceleration_module=in_a,dt=0.005)

ff = Fading_Filter(tg,Nstd=0,Beta=0.7,Order=2,dt= delta)
ff.initial(initial_estimation=np.array([in_p,in_v]))
ff.start()
for i in range(100):
    ff.get_estimation()
    time.sleep(delta)
ff.stop()

ff.plot_Filter()'''