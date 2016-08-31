#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg    import Float64MultiArray
from sensor_msgs.msg import Joy
from dsdm_msgs.msg   import dsdm_actuator_control_inputs

#########################################
class dsdm_pid(object):
    """
    navigation
    """
    def __init__(self):
        
        self.verbose = True
        
        self.pub_u              = rospy.Publisher("a0/u", dsdm_actuator_control_inputs , queue_size=1  )
        self.sub_joy            = rospy.Subscriber("joy", Joy , self.joy_callback , queue_size=1       )
        self.sub_state_feedback = rospy.Subscriber("x_hat", Float64MultiArray , self.update_state ,  queue_size=1      )
        
        # Init DSDM msg
        self.f = 0
        self.k = 1 # High force mode
        
        # Init sensor feedback
        self.x  = np.zeros(2)
        self.a  = self.x[0]
        self.da = self.x[1]
        
        # Init joy feedback
        self.joy_msg         = Joy()
        self.joy_msg.axes    = [ 0 , 0 , 0 , 0 , 0 ]
        self.joy_msg.buttons = [ 0 , 0 , 0 , 0 , 0 ]
        
        # PID
        self.enable = False
        self.mode   = 'PID_speed'
        self.gain   = np.array([ 1.0 , 0 , 0 ])
        
    
    #######################################   
    def callback( self, msg ):
        """ """
        
        if self.enable:
            
            # OPENLOOP
            if self.mode == 'PWM' :
                # Pick set_point with joysticks gain
                self.f  =    self.joy_msg.axes[1] * 0.03 
                    
                # Pick mode with trigger
                if ( self.joy_msg.axes[5] < 0):
                    self.k = 0
                else:
                    self.k = 1
                        
            
            # PID Position
            elif self.mode == 'PID_position' :
                
                e  = self.set_point - self.a
                de = 0              - self.da
                
                kp = self.gain[0]
                ki = self.gain[1]
                kd = self.gain[2]
                
                cmd = kp * e + kd * de
                
                self.f = cmd * 0.03 
                
                #  Pick mode with trigger
                if ( self.joy_msg.axes[5] < 0):
                    self.k = 0
                else:
                    self.k = 1
                    
            # PID Position
            elif self.mode == 'PID_speed' :
                
                e  = self.set_point - self.da
                de = 0
                
                kp = self.gain[0]
                ki = self.gain[1]
                kd = self.gain[2]
                
                cmd = kp * e + kd * de
                
                self.f = cmd * 0.03 
                
                #  Pick mode with trigger
                if ( self.joy_msg.axes[5] < 0):
                    self.k = 0
                else:
                    self.k = 1
        
        else:
            self.f = 0
            self.k = 1
        
        
        self.pub_u_msg()
        
        
    #######################################   
    def joy_callback( self, msg ):
        """ Log joy msg """
        
        self.joy_msg = msg
        
        self.set_point = msg.axes[1]
        
        # Pick ctrl_mode with button state
        if ( msg.buttons[0] == 1 ):
            self.enable = True
        else:
            self.enable = False
            
            
    #######################################   
    def pub_u_msg( self ):
        """ pub actuator cmd """
        
        msg = dsdm_actuator_control_inputs()
        
        msg.f = self.f
        msg.k = self.k
        
        self.pub_u.publish( msg )
        
    
    #######################################   
    def update_state( self, msg ):
        """ state feedback """
        
        
        self.x = msg.data
        
        self.a  = self.x[0]
        self.da = self.x[3]
        
        self.callback( None )
        

            


#########################################
if __name__ == '__main__':
    
    rospy.init_node('dsdm_pid',anonymous=False)
    node = dsdm_pid()
    rospy.spin()
