# -*- coding: utf-8 -*-
"""
Created on Sun Mar  6 15:27:04 2016

@author: alex
"""

from AlexRobotics.planning import RandomTree           as RPRT
from AlexRobotics.dynamic  import CustomManipulator    as CM
import numpy as np
import matplotlib.pyplot as plt


ReComputeTraj = False
name_traj     = '../data/pendulum_traj.npy'


####################################
R  =  CM.TestPendulum()

R.g = 9.8 * 3.0 # gravity on

R.x_ub[0] = 2*np.pi
R.x_ub[1] = 0
R.x_ub[2] = 0
R.x_ub[4] = 0
R.x_ub[5] = 0
R.x_lb[0] = - 2*np.pi
R.x_lb[1] = - 0
R.x_lb[2] = - 0
R.x_lb[4] = - 0
R.x_lb[5] = - 0

R.ubar = np.array([0,0,0,1])

x_start = np.array([-2.8,0,0,0,0,0])
x_goal  = np.array([ 0,0,0,0,0,0])

RRT = RPRT.RRT( R , x_start )

T    = 0.01
u_R1 = 1
#u_R2 = 1

RRT.U = np.array([ [ T,0,0,u_R1] , [ 0,0,0,u_R1] , [ -T,0,0,u_R1] ])

R.phase_plane()


RRT.dt                    = 0.1
RRT.goal_radius           = 0.3
RRT.alpha                 = 0.8
RRT.max_nodes             = 10000
RRT.max_solution_time     = 5

# Dynamic plot
RRT.dyna_plot             = False
RRT.dyna_node_no_update   = 1000

RRT.y1axis = 0
RRT.y2axis = 3



if ReComputeTraj:
    
    RRT.find_path_to_goal( x_goal )
    RRT.save_solution( name_traj  )
    RRT.plot_2D_Tree()
    
else:
    
    RRT.load_solution( name_traj  )

plt.ion()