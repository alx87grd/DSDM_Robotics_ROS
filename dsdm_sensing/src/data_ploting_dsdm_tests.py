#!/usr/bin/env python

import numpy as np
import matplotlib
import matplotlib.pyplot as plt


#data = np.array([[ self.q , self.dq , self.q_d , self.f , self.k , t, self.w1 , self.w2 , self.i1 , self.i2 , self.brake ]])

#########################################
class plotter(object):
    
    def __init__(self):
        
        pass
    

    def loaddata( self, name , i , j ):
    
        fullname      = '/home/alex/ROS_WS/src/dsdm_robotics/dsdm_sensing/data/' + name + '.npy'
        
        DATA = np.load( fullname )
        
        ## DEOCODING
        self.q  = DATA[i:j,0] - DATA[i,0]
        self.dq = DATA[i:j,1]
        self.qd = DATA[i:j,2]
        self.f  = DATA[i:j,3]
        self.k  = DATA[i:j,4]
        self.t  = DATA[i:j,5] - DATA[i,5] 
        self.w1 = DATA[i:j,6] 
        self.w2 = DATA[i:j,7]  
        self.i1 = DATA[i:j,8] 
        self.i2 = DATA[i:j,9]
        self.b  = DATA[i:j,10] 
        self.id1= DATA[i:j,11] 
        self.id2= DATA[i:j,12]
        self.wr1= DATA[i:j,13] 
        self.wr2= DATA[i:j,14]
        
        
    ## Figures
    
    def plot_position_tracking( self):
    
        fontsize = 5
        
        matplotlib.rc('xtick', labelsize=fontsize )
        matplotlib.rc('ytick', labelsize=fontsize )
        
        
        simfig , plot = plt.subplots(4, sharex=True,figsize=(5, 3),dpi=400, frameon=True)
        
        #simfig.canvas.set_window_title('Closed loop trajectory')
        
        plot[0].plot( self.t ,  self.qd , 'r--', label = 'Ref. trajectory')
        plot[0].plot( self.t ,  self.q , 'b' ,  label = 'Actual position')
        plot[0].set_yticks([0, 3.14])
        plot[0].set_ylim(-2,5)
        legend = plot[0].legend(loc='lower right', fancybox=True, shadow=False, prop={'size':fontsize})
        legend.get_frame().set_alpha(0.4)
        plot[0].grid(True)
        
        plot[1].plot( self.t ,  self.dq , 'b')
        #plot[1].set_yticks([-6,0,6])
        #plot[1].set_ylim(-8,8)
        plot[1].grid(True)
        
        plot[2].plot( self.t ,  self.f , 'b')
        plot[2].set_yticks([-0.2,0,0.2])
        plot[2].set_ylim(-0.3,0.3)
        plot[2].grid(True)
        
        plot[3].plot( self.t ,  self.k * 451 + 23 , 'r'   , label = 'target')
        plot[3].plot( self.t ,  self.b * 451 + 23 , 'b--' , label = 'engaged')
        plot[3].set_yticks([23,474])
        plot[3].set_ylim(-100,600)
        plot[3].grid(True)
        legend = plot[3].legend(loc='lower right', fancybox=True, shadow=False, prop={'size':fontsize})
        legend.get_frame().set_alpha(0.4)
        
        plot[0].set_ylabel('Angle [rad]' , fontsize=fontsize )
        plot[1].set_ylabel('Speed [rad/sec]' , fontsize=fontsize )
        plot[2].set_ylabel('Torque [Nm]' , fontsize=fontsize )
        plot[3].set_ylabel('Gear Ratio' , fontsize=fontsize )
        
        plot[-1].set_xlabel('Time [sec]', fontsize=fontsize )
        #plot[-1].set_xlim(0,4)
        #plot[-1].set_xticks([0,1,2,3,4])
    
    
        simfig.tight_layout()
        
        simfig.show()
        
        
    def plot_all_OL(self):
    
        fontsize = 5
        
        matplotlib.rc('xtick', labelsize=fontsize )
        matplotlib.rc('ytick', labelsize=fontsize )
        
        
        simfig , plot = plt.subplots(4, sharex=True,figsize=(3, 3),dpi=400, frameon=True)
        
        simfig.canvas.set_window_title('All')
        
        #plot[0].plot( t ,  qd , 'r--', label = 'Ref. trajectory')
        plot[0].plot( self.t ,  self.q , 'b' ,  label = 'Actual position')
        #plot[0].set_yticks([-3.14,0])
        #plot[0].set_ylim(-5,1)
        plot[0].grid(True)
        #legend = plot[0].legend(loc='lower right', fancybox=True, shadow=False, prop={'size':fontsize})
        #legend.get_frame().set_alpha(0.4)
        
        plot[1].plot( self.t ,  self.dq , 'b')
        #plot[1].set_yticks([-6,0,6])
        #plot[1].set_ylim(-8,8)
        plot[1].grid(True)
        
        plot[2].plot( self.t ,  self.f , 'b')
        plot[2].set_yticks([-0.2,0,0.2])
        plot[2].set_ylim(-0.3,0.3)
        plot[2].grid(True)
        
        plot[3].plot( self.t ,  self.k * 451 + 23 , 'r'   , label = 'target')
        plot[3].plot( self.t ,  self.b * 451 + 23 , 'b--' , label = 'engaged')
        plot[3].set_yticks([23,474])
        plot[3].set_ylim(-100,600)
        plot[3].grid(True)
        legend = plot[3].legend(loc='lower right', fancybox=True, shadow=False, prop={'size':fontsize})
        legend.get_frame().set_alpha(0.4)
        
        plot[0].set_ylabel('Angle [rad]' , fontsize=fontsize )
        plot[1].set_ylabel('Speed [rad/sec]' , fontsize=fontsize )
        plot[2].set_ylabel('Torque [Nm]' , fontsize=fontsize )
        plot[3].set_ylabel('Gear Ratio' , fontsize=fontsize )
        
        plot[-1].set_xlabel('Time [sec]', fontsize=fontsize )
        #plot[-1].set_xlim(0,4)
        #plot[-1].set_xticks([0,1,2,3,4])
    
    
        simfig.tight_layout()
        
        simfig.show()
        
    
    def plot_motors(self):
    
        fontsize = 5
        
        matplotlib.rc('xtick', labelsize=fontsize )
        matplotlib.rc('ytick', labelsize=fontsize )
        
        
        simfig , plot = plt.subplots(1, sharex=True,figsize=(4, 3),dpi=400, frameon=True)
        
        simfig.canvas.set_window_title('Motors velocity')
        
        plot.plot( self.t ,  -self.w1 , 'b' , label = 'M1')
        plot.plot( self.t ,  -self.w2 , 'r' , label = 'M2')
        plot.grid(True)
        legend = plot.legend(loc='lower right', fancybox=True, shadow=False, prop={'size':fontsize})
        legend.get_frame().set_alpha(0.4)
        
        plot.set_ylabel('Velocity [rad/sec' , fontsize=fontsize )
        
        plot.set_xlabel('Time [sec]', fontsize=fontsize )
        
        simfig.tight_layout()
        
        simfig.show()
        
        
    def plot_shift(self):
    
        fontsize = 5
        
        matplotlib.rc('xtick', labelsize=fontsize )
        matplotlib.rc('ytick', labelsize=fontsize )
        
        
        simfig , plot = plt.subplots(3, sharex=True,figsize=(3, 3),dpi=400, frameon=True)
        
        simfig.canvas.set_window_title('Shift')
        
        plot[0].plot( self.t ,  self.dq , 'b' ,  label = 'Actual position')
        #plot[0].set_yticks([-3.14,0])
        #plot[0].set_ylim(-5,1)
        plot[0].grid(True)
        
        plot[1].plot( self.t ,  -self.w1 , 'b' , label = 'M1')
        plot[1].plot( self.t ,  -self.w2 , 'r' , label = 'M2')
        plot[1].grid(True)
        legend = plot[1].legend(loc='lower right', fancybox=True, shadow=False, prop={'size':fontsize})
        legend.get_frame().set_alpha(0.4)
        
        plot[2].plot( self.t ,  self.k * 451 + 23 , 'r'   , label = 'target')
        plot[2].plot( self.t ,  self.b * 451 + 23 , 'b--' , label = 'engaged')
        plot[2].set_yticks([23,474])
        plot[2].set_ylim(-100,600)
        plot[2].grid(True)
        legend = plot[2].legend(loc='lower right', fancybox=True, shadow=False, prop={'size':fontsize})
        legend.get_frame().set_alpha(0.4)
        
        plot[0].set_ylabel('Velocity [rad/sec]' , fontsize=fontsize )
        plot[1].set_ylabel('Velocity [rad/sec]' , fontsize=fontsize )
        plot[2].set_ylabel('Gear Ratio' , fontsize=fontsize )
        
        plot[-1].set_xlabel('Time [sec]', fontsize=fontsize )
        
        simfig.tight_layout()
        
        simfig.show()
        
        
    def plot_shift_zoom(self):
    
        fontsize = 5
        
        matplotlib.rc('xtick', labelsize=fontsize )
        matplotlib.rc('ytick', labelsize=fontsize )
        
        
        simfig , plot = plt.subplots(4, sharex=True,figsize=(3, 3),dpi=400, frameon=True)
        
        simfig.canvas.set_window_title('Shift')
        
        plot[0].plot( self.t ,  self.dq , 'b' ,  label = 'Actual position')
        #plot[0].set_yticks([-3.14,0])
        #plot[0].set_ylim(-5,1)
        plot[0].grid(True)
        
        plot[1].plot( self.t ,  -self.wr1 , 'b' , label = 'M1')
        #plot[1].plot( self.t ,  -self.w2 , 'r' , label = 'M2')
        plot[1].grid(True)
        
        #plot[1].plot( self.t ,  -self.w1 , 'b' , label = 'M1')
        plot[2].plot( self.t ,  -self.w2 , 'r' , label = 'M2')
        plot[2].grid(True)
        
        plot[3].plot( self.t ,  self.k * 451 + 23 , 'r'   , label = 'target')
        plot[3].plot( self.t ,  self.b * 451 + 23 , 'b--' , label = 'engaged')
        plot[3].set_yticks([23,474])
        plot[3].set_ylim(-100,600)
        plot[3].grid(True)
        legend = plot[2].legend(loc='lower right', fancybox=True, shadow=False, prop={'size':fontsize})
        legend.get_frame().set_alpha(0.4)
        
        plot[0].set_ylabel('Output [rad/sec]' , fontsize=fontsize )
        plot[1].set_ylabel('M1 [ticks/dt]' , fontsize=fontsize )
        plot[2].set_ylabel('M2 [rad/sec]' , fontsize=fontsize )
        plot[3].set_ylabel('Gear Ratio' , fontsize=fontsize )
        
        plot[-1].set_xlabel('Time [sec]', fontsize=fontsize )
        
        simfig.tight_layout()
        
        simfig.show()
        
        
        
    def plot_speed_tracking(self):
    
        fontsize = 5
        
        matplotlib.rc('xtick', labelsize=fontsize )
        matplotlib.rc('ytick', labelsize=fontsize )
        
        
        simfig , plot = plt.subplots(3, sharex=True,figsize=(3, 3),dpi=400, frameon=True)
        
        simfig.canvas.set_window_title('Speed tracking')
        
        plot[0].plot( self.t ,  self.dq , 'b' ,  label = 'Actual')
        plot[0].plot( self.t ,  self.qd , 'r--', label = 'Target')
        #plot[0].set_yticks([-3.14,0])
        #plot[0].set_ylim(-5,1)
        plot[0].grid(True)
        
        plot[1].plot( self.t ,  -self.w1 , 'b' , label = 'M1')
        plot[1].plot( self.t ,  -self.w2 , 'r' , label = 'M2')
        plot[1].grid(True)
        legend = plot[1].legend(loc='lower right', fancybox=True, shadow=False, prop={'size':fontsize})
        legend.get_frame().set_alpha(0.4)
        
        plot[2].plot( self.t ,  self.k * 451 + 23 , 'r'   , label = 'target')
        plot[2].plot( self.t ,  self.b * 451 + 23 , 'b--' , label = 'engaged')
        plot[2].set_yticks([23,474])
        plot[2].set_ylim(-100,600)
        plot[2].grid(True)
        legend = plot[2].legend(loc='lower right', fancybox=True, shadow=False, prop={'size':fontsize})
        legend.get_frame().set_alpha(0.4)
        
        plot[0].set_ylabel('Velocity [rad/sec]' , fontsize=fontsize )
        plot[1].set_ylabel('Velocity [rad/sec]' , fontsize=fontsize )
        plot[2].set_ylabel('Gear Ratio' , fontsize=fontsize )
        
        plot[-1].set_xlabel('Time [sec]', fontsize=fontsize )
        
        simfig.tight_layout()
        
        simfig.show()
        
        
    def plot_nullspace(self):
    
        fontsize = 5
        
        matplotlib.rc('xtick', labelsize=fontsize )
        matplotlib.rc('ytick', labelsize=fontsize )
        
        
        simfig , plot = plt.subplots(3, sharex=True,figsize=(3, 3),dpi=400, frameon=True)
        
        simfig.canvas.set_window_title('Nullspace')
        
        plot[0].plot( self.t ,  self.dq , 'b' ,  label = 'Actual')
        plot[0].plot( self.t ,  self.qd , 'r--', label = 'Target')
        #plot[0].set_yticks([-3.14,0])
        #plot[0].set_ylim(-5,1)
        plot[0].grid(True)
        
        plot[1].plot( self.t ,  -self.w1 , 'b' , label = 'M1')
        #plot[1].plot( self.t ,  -self.w2 , 'r' , label = 'M2')
        plot[1].grid(True)
        
        #plot[2].plot( self.t ,  -self.w1 , 'b' , label = 'M1')
        plot[2].plot( self.t ,  -self.w2 , 'r' , label = 'M2')
        plot[2].grid(True)
        
        plot[0].set_ylabel('Output [rad/sec]' , fontsize=fontsize )
        plot[1].set_ylabel('M1 [rad/sec]' , fontsize=fontsize )
        plot[2].set_ylabel('M2 [rad/sec]' , fontsize=fontsize )
        
        plot[-1].set_xlabel('Time [sec]', fontsize=fontsize )
        
        simfig.tight_layout()
        
        simfig.show()
        


# Main

pltr = plotter()
    
##############################################
###      Manual Experiment ###################
##############################################
    
#name      = 'dsdm_a2_manual_no1'
#i = 1 + 500 * 30
##j = i + 500 * 
#j = 1 + 500 * 42
#
#pltr.loaddata( name , i , j)
#pltr.plot_all_OL()
#pltr.plot_motors()

# nice global data
name      = 'dsdm_a2_manual_no2'
i = 1 + 500 * 1
#j = i + 500 * 
j = 1 + 500 * 37

pltr.loaddata( name , i , j)
pltr.plot_all_OL()
pltr.plot_motors()

#ZOOM on shifts
name      = 'dsdm_a2_manual_no2'
i = 1 + 500 * 23
#j = i + 500 * 
j = 1 + 500 * 32

pltr.loaddata( name , i , j)
pltr.plot_shift()

##############################################
###   Speed Tracking Experiment ##############
##############################################

name      = 'dsdm_a2_speed_no1'
i = 1 + 500 * 5
#j = i + 500 * 
j = 1 + 500 * 21

pltr.loaddata( name , i , j)
pltr.plot_speed_tracking()

##############################################
###  Position Tracking Experiment ############
##############################################

name      = 'dsdm_a2_position_no3'

# HF

i = 1 + 500 * 40
#j = i + 500 * 
j = 1 + 500 * 80

pltr.loaddata( name , i , j)
pltr.plot_position_tracking()

## HS
#
#i = 1 + 500 * 67
##j = i + 500 * 
#j = 1 + 500 * 71
#
#pltr.loaddata( name , i , j)
#pltr.plot_position_tracking()

##############################################
###  Nullspace  ############
##############################################

name      = 'dsdm_a2_nullspace_no1'

i = 1 + 500 * 1
#j = i + 500 * 
j = 1 + 500 * 60

pltr.loaddata( name , i , j)
pltr.plot_nullspace()


##############################################
###  Nullspace  ############
##############################################

# Wihtout preparation

name      = 'dsdm_a2_cstspd_shifts_no1'

i = 1 + 500 * 5
#j = i + 500 * 
j = 1 + 500 * 13

pltr.loaddata( name , i , j)
pltr.plot_shift()


# With preparation


name      = 'dsdm_a2_cstspd_shifts_no2'

i = 1 + 500 * 1
#j = i + 500 * 
j = 1 + 500 * 24

pltr.loaddata( name , i , j)
pltr.plot_shift()


# With preparation zoom

i = 1 + 500 * 4.6
#j = i + 500 *4
j = 1 + 500 * 4.9

pltr.loaddata( name , i , j)
pltr.plot_shift_zoom()