#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg    import Float64MultiArray
from sensor_msgs.msg import Joy
from dsdm_msgs.msg   import dsdm_actuator_control_inputs, ctl_error

#########################################
class dsdm_pid(object):
    """
    navigation
    """
    def __init__(self):
        
        self.verbose = False
        
        # Publishers
        self.pub_u              = rospy.Publisher("a2/u", dsdm_actuator_control_inputs , queue_size=1  )
        self.pub_e              = rospy.Publisher("pid_error", ctl_error               , queue_size=1  )
        
        # Suscribers
        self.sub_joy            = rospy.Subscriber("joy", Joy , self.joy_callback , queue_size=1       )
        self.sub_state_feedback = rospy.Subscriber("x_hat", Float64MultiArray , self.update_state ,  queue_size=1      )
        
        # Init DSDM msg
        self.f  = 0
        self.k  = 1 # High force mode
        
        # Init sensor feedback
        self.x  = np.zeros(2)
        self.a  = self.x[0]
        self.da = self.x[1]
        
        # Init joy feedback
        self.joy_msg         = Joy()
        self.joy_msg.axes    = [ 0 , 0 , 0 , 0 , 0 ]
        self.joy_msg.buttons = [ 0 , 0 , 0 , 0 , 0 ]
        
        # Init ctl_error msg
        self.e               = 0
        self.de              = 0
        self.set_point       = 0
        self.actual          = 0
        self.n               = 0
        self.n_d             = 0  # Nullspace setpoint
        
        # PID
        self.enable    = False
        self.mode      = 'PWM'
        #self.mode      = 'PID_position'
        #self.mode      = 'PID_speed'
        self.last_mode = self.mode
        self.e_sum     = 0
        self.gain      = np.array([ 0.2 , 0.05 , 0.2 ])
        self.dt        = 0.02  # assuming 500 HZ
        
    
    #######################################   
    def callback( self, msg ):
        """ """
        
        if self.enable:
            
            # OPENLOOP
            if self.mode == 'PWM' :
                # Pick set_point with joysticks gain
                self.f  =    self.joy_msg.axes[1] * 0.2
                    
                # Pick mode with trigger
                if ( self.joy_msg.axes[5] < 0):
                    self.k = 0
                else:
                    self.k = 1
                    
                self.e_sum = 0 # Reset integral error
                        
            
            # PID Position
            elif self.mode == 'PID_position' :
                
                e  = self.set_point - self.a
                de = 0              - self.da
                
                self.e_sum = self.e_sum + e * self.dt
                
                kp = self.gain[0]
                ki = self.gain[1]
                kd = self.gain[2]
                
                cmd = kp * e + kd * de + ki * self.e_sum
                
                self.f = cmd * 0.2
                
                # For debug
                self.actual = self.a
                self.e      = e
                self.de     = de
                
                #  Pick mode with trigger
                if ( self.joy_msg.axes[5] < 0):
                    self.k = 0
                else:
                    self.k = 1
                    
            # PID Position
            elif self.mode == 'PID_speed' :
                
                e  = self.set_point - self.da
                de = 0
                
                self.e_sum = self.e_sum + e * self.dt
                
                kp = self.gain[0]
                ki = self.gain[1]
                kd = self.gain[2]
                
                cmd = kp * e + kd * de + ki * self.e_sum
                
                self.f = cmd * 0.2
                
                # For debug
                self.actual = self.da
                self.e      = e
                self.de     = de
                
                #  Pick mode with trigger
                if ( self.joy_msg.axes[5] < 0):
                    self.k = 0
                else:
                    self.k = 1
                    
            # Nullspace Target
            self.n = self.n_d
            
        
        else:
            self.f   = 0
            self.k   = 1
            self.n_d = 0
        
        
        self.pub_u_msg()
        
        
    #######################################   
    def joy_callback( self, msg ):
        """ Log joy msg """
        
        self.joy_msg = msg
        
        self.set_point           = msg.axes[1] * np.pi
        
        self.n_d                 = msg.axes[4] 
        
        # Pick ctrl_mode with button state
        if ( msg.buttons[0] == 1 ):
            self.enable = True
        else:
            self.enable = False
            
        # Pick ctrl_mode with button state
        #    Default
        self.mode == 'PWM'
            
        if ( msg.buttons[1] == 1 ):
            self.mode   = 'PID_position'
        elif ( msg.buttons[2] == 1 ):
            self.mode   = 'PID_speed'
        else:
            self.mode == 'PWM'
            
        # Reset Integral Error is mode changed
        if not( self.last_mode == self.mode ):
            
            self.e_sum = 0
            
        self.last_mode = self.mode
            
            
            
    #######################################   
    def pub_u_msg( self ):
        """ pub actuator cmd """
        
        msg = dsdm_actuator_control_inputs()
        
        msg.f = self.f
        msg.k = self.k
        msg.n = self.n
        
        self.pub_u.publish( msg )
        
        # Publish error data
        msg2              = ctl_error()
        msg2.header.stamp = rospy.Time.now()
        msg2.set_point    = self.set_point
        msg2.actual       = self.actual
        msg2.e            = self.e
        msg2.de           = self.de
        
        self.pub_e.publish( msg2 )
        
        if self.verbose:
            print('Error:', self.e )
        
    
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
