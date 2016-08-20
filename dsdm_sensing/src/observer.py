#!/usr/bin/env python
"""
Created on Sat Aug 20 14:47:23 2016

@author: alex
"""

import rospy
import numpy as np
from dsdm_msgs.msg import dsdm_actuator_sensor_feedback
from std_msgs.msg  import Float64MultiArray

from AlexRobotics.dynamic  import CustomManipulator    as CM

import matplotlib
import matplotlib.pyplot as plt

#import mtTkinter as Tkinter

#########################################
class DSDM_OBS(object):
    """
    Observer Node for DSDM :
    
    listen to DSDM actuator outputs
    publish state estimates
    
    """
    
    #################################
    def __init__(self):
        
        self.verbose = False
        self.plot    = True
        
        # Message outgoing
        self.pub_x          = rospy.Publisher("x_hat", Float64MultiArray , queue_size=1      )
        
        # Messages Inputs        
        self.sub_a0         = rospy.Subscriber("a0/y", dsdm_actuator_sensor_feedback , self.feedback_callback_a0 , queue_size=1 )
        self.sub_a1         = rospy.Subscriber("a1/y", dsdm_actuator_sensor_feedback , self.feedback_callback_a1 , queue_size=1 )
        self.sub_a2         = rospy.Subscriber("a2/y", dsdm_actuator_sensor_feedback , self.feedback_callback_a2 , queue_size=1 )
        
        # Timer
        self.timer          = rospy.Timer( rospy.Duration.from_sec(0.2),    self.timer_callback  )
        
        # Load robot model
        self.R       = CM.BoeingArm()

        #########################
        
        # Params
        
        self.a_zero = np.array( [0.05,0,0] ) 
        
        #self.load_params( None )
        
        
        # Variable Init
        self.a      = np.zeros( self.R.dof )       
        self.da     = np.zeros( self.R.dof )
        self.q      = np.zeros( self.R.dof )       
        self.dq     = np.zeros( self.R.dof )
        self.x_hat  = self.R.q2x( self.q , self.dq )
        
        if self.plot:
            self.R.show_3D( self.q )
            
        
        #self.R.update_show_3D( [1,0,0] )
        
        #print 'LHFDSLHFLKDJHDSLHDLKHDFLKHDLKHDLDHFLD'
            
    ###################################
    def main_callback(self):
        """ """
        
        # Extimate state vector
        self.estimate_state()
            
        # Publish state estimate
        self.pub_state_estimate()
    
    ###################################
    def estimate_state(self ):
        """ """
        
        # Just pure kinematic for boeing Arm
        self.q     = self.R.a2q(   self.a  + self.a_zero      )
        self.dq    = self.R.da2dq( self.da , self.q )
        self.x_hat = self.R.q2x( self.q , self.dq   )  
        
        
    ###################################
    def timer_callback(self , event ):
        """ """
        
        # Update graphic
        if self.plot:
            self.R.update_show_3D( self.q )
            
            print "GRAPH UPDATED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    
    #######################################   
    def pub_state_estimate( self ):
        """ pub control inputs """

        msg = Float64MultiArray()
        
        msg.data = self.x_hat
        
        self.pub_x.publish( msg )
        
        
    ###################################
    def feedback_callback_a0(self, msg ):
        """ """
        
        self.a[0]  = msg.a
        self.da[0] = msg.da
        
        # Evante based timing
        self.main_callback()
        
    ###################################
    def feedback_callback_a1(self, msg ):
        """ """
        
        self.a[1]  = msg.a
        self.da[1] = msg.da
        
    ###################################
    def feedback_callback_a2(self, msg ):
        """ """
        
        self.a[2]  = msg.a
        self.da[2] = msg.da
        
        
        
#########################################
if __name__ == '__main__':
    
    plt.ion()
    rospy.init_node('obs',anonymous=False)
    node = DSDM_OBS()
    #rospy.spin()
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        
        node.timer_callback( None )
        rate.sleep()
