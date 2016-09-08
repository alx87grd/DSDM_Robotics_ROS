#!/usr/bin/env python

import numpy as np
import matplotlib
import matplotlib.pyplot as plt


#data = np.array([[ self.q , self.dq , self.q_d , self.f , self.k , t, self.w1 , self.w2 , self.i1 , self.i2 , self.brake ]])


## LOADING

name      = 'succes5'
name      = '/home/alex/ROS_WS/src/dsdm_robotics/dsdm_sensing/data/' + name + '.npy'

DATA = np.load( name )

i = 1 + 500 * 1.47
j = 0 + 500 * 6.0

## DEOCODING
q  = DATA[i:j,0]
dq = DATA[i:j,1]
qd = DATA[i:j,2]
f  = DATA[i:j,3]
k  = DATA[i:j,4]
t  = DATA[i:j,5] - DATA[1,5] 
w1 = DATA[i:j,6] 
w2 = DATA[i:j,7]  
i1 = DATA[i:j,8] 
i2 = DATA[i:j,9]
b  = DATA[i:j,10] / 255.
id1= DATA[i:j,11] 
id2= DATA[i:j,12]


## Figures

def plot_main():

    fontsize = 5
    
    matplotlib.rc('xtick', labelsize=fontsize )
    matplotlib.rc('ytick', labelsize=fontsize )
    
    
    simfig , plot = plt.subplots(3, sharex=True,figsize=(4, 3),dpi=400, frameon=True)
    
    simfig.canvas.set_window_title('Closed loop trajectory')
    
    plot[0].plot( t ,  q , 'b')
    plot[0].plot( t ,  qd , 'r')
    plot[0].grid(True)
    
    plot[1].plot( t ,  dq , 'b')
    plot[1].grid(True)
    
    plot[2].plot( t ,  f , 'b')
    plot[2].plot( t ,  k * 0.1 , 'r')
    #plot[1].plot( t ,  b , 'c')
    plot[2].grid(True)
    
    #plot[2].plot( t ,  w1 , 'b')
    #plot[2].plot( t ,  w2 , 'r')
    
    plot[0].set_ylabel('Angle [rad]' , fontsize=fontsize )
    
    plot[-1].set_xlabel('Time [sec]', fontsize=fontsize )
    
    simfig.tight_layout()
    
    simfig.show()
    

def plot_motors():

    fontsize = 5
    
    matplotlib.rc('xtick', labelsize=fontsize )
    matplotlib.rc('ytick', labelsize=fontsize )
    
    
    simfig , plot = plt.subplots(1, sharex=True,figsize=(4, 3),dpi=400, frameon=True)
    
    simfig.canvas.set_window_title('Motors velocity')
    
    plot.plot( t ,  w1 , 'b')
    plot.plot( t ,  w2 , 'r')
    plot.grid(True)
    
    plot.set_ylabel('Velocity [rad]/sec' , fontsize=fontsize )
    
    plot.set_xlabel('Time [sec]', fontsize=fontsize )
    
    simfig.tight_layout()
    
    simfig.show()
    
def plot_currents():

    fontsize = 5
    
    matplotlib.rc('xtick', labelsize=fontsize )
    matplotlib.rc('ytick', labelsize=fontsize )
    
    
    simfig , plot = plt.subplots(1, sharex=True,figsize=(4, 3),dpi=400, frameon=True)
    
    simfig.canvas.set_window_title('Motors Currents')
    
    plot.plot( t ,  i1 , 'b')
    plot.plot( t ,  i2 , 'r')
    plot.plot( t ,  id1 , 'g.')
    plot.plot( t ,  id2 , 'c.')
    plot.grid(True)
    
    plot.set_ylabel('Currents [mA]' , fontsize=fontsize )
    
    plot.set_xlabel('Time [sec]', fontsize=fontsize )
    
    simfig.tight_layout()
    
    simfig.show()
    
    
    
plot_main()
plot_motors()
plot_currents()