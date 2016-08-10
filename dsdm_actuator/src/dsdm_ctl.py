#!/usr/bin/env python
import rospy
import numpy as np
from dsdm_msgs.msg import dsdm_actuator_control_inputs
from dsdm_msgs.msg import dsdm_actuator_sensor_feedback
from flexsea_execute.msg import inputs
from flexsea_execute.msg import outputs
from sensor_msgs.msg import Joy

#########################################
class DSDM_CTL(object):
    """
    Controller for DSDM :
    
    received DSDM control inputs
    send msg to two FlexSEA Driver Node
    """
    
    def __init__(self):
        
        self.verbose = False
        
        self.pub_cmd_m1     = rospy.Publisher("M1/u", inputs , queue_size=1      )
        self.pub_cmd_m2     = rospy.Publisher("M2/u", inputs , queue_size=1      )
        self.pub_y          = rospy.Publisher("y", dsdm_actuator_sensor_feedback , queue_size=1      )
        
        self.sub_u          = rospy.Subscriber("u", dsdm_actuator_control_inputs , self.setpoint_callback , queue_size=1 )
        self.sub_y_m1       = rospy.Subscriber("M1/y", outputs , self.feedback_callback_m1 , queue_size=1 )
        self.sub_y_m2       = rospy.Subscriber("M2/y", outputs , self.feedback_callback_m2 , queue_size=1 )
        
        # Set point
        self.f = 0
        self.k = 1
        
        # Feedback from sensors
        self.y = np.zeros(3) # M1 = [1], M2 = [1]
        self.w = np.zeros(3)  # Velocity
        
        # Memory for speed filter
        self.y_past = np.zeros(3)   # t-1 value
        self.w_past = np.zeros(3)  # t-1 value
        self.w_x    = np.zeros(3)  # state of the filter
        
        self.dt = 0.02
        RC      = 1.0
        
        self.alpha = self.dt / ( RC + self.dt )
        print self.alpha
        
        
        #Motor signs: TODO read from params
        self.m_sign = [-1,1]
        self.g      = [1,1,1]   # Gear ratios [output,M1,M2]
        
        
        # Init flexsea input msg
        
        # Shared data
        self.ctrl_gains      = [10,0,0,0,0,0]
        self.trap_mode       = False
        trap_spd             = 50000
        trap_acc             = 50000
        self.trap_values     = [ 0 , 0 , 0 , trap_spd , trap_acc , 0 ]
        
        # Idividual data
        self.setpoints      = [0,0]
        self.ctrl_modes     = [0,0]
        self.brake_state_M1 = 0
        
    ###########################################
    def controller( self ):
        """ """
        
        # SIMPLE OPEN LOOP  wihtout synchronization
        
        if ( self.k == 0 ):
            # High speed mode
            self.setpoints        = [ self.f , 0 ] # Direct M1 PWM
            self.ctrl_modes       = [      1 , 0 ] # PWM mode for M!
            self.brake_state_M1   = 255 # Brake open
            
        elif ( self.k ==1 ):
            # High force mode
            self.setpoints        = [ 0 , self.f ] # Direct M1 PWM
            self.ctrl_modes       = [ 0 , 1      ] # PWM mode for M!
            self.brake_state_M1   = 0 # Brake close
            
        
        ##########################################
        
        # Adjust for sign
        self.setpoints[0] = self.setpoints[0] * self.m_sign[0]
        self.setpoints[1] = self.setpoints[1] * self.m_sign[1]
        
        # Publish Motor cmd
        self.pub_cmd()
    
    
    ############################################
    def setpoint_callback( self , msg ):
        
        """ Record new setpoint """
        self.f = msg.f  # effort
        self.k = msg.k  # mode
        
    ############################################
    def feedback_callback_m1( self , msg ):
        """ """
        
        self.update_feedback( msg , 1 )
    
    ############################################
    def feedback_callback_m2( self , msg ):
        """ """
        
        self.update_feedback( msg , 2 )
        
        # For asynchrone ctl
        self.controller()
        self.pub_feedback( msg )
        
        
    ############################################
    def update_feedback( self , msg , motor_ID ):
        """ """
        self.y_past[ motor_ID ] = self.y[ motor_ID ]
        self.y[ motor_ID ]      = msg.encoder
        
        # TODO : FILTER diff for speed
        dy = self.y[ motor_ID ] - self.y_past[ motor_ID ]
        
        w  = ( dy + 0.0)  / self.dt      
        
        w_filtered = self.alpha * self.w_x[ motor_ID ] + ( 1 - self.alpha ) * w
        
        self.w[ motor_ID ] = int( w_filtered )
          
        # Memory
        self.w_x[    motor_ID ] = w_filtered
        self.y_past[ motor_ID ] = self.y[ motor_ID ]
        
        
    ###########################################
    def pub_cmd( self ):
        """ """
        
        msg_m1 = inputs()
        msg_m2 = inputs()
        
        msg_m1.ctrl_mode       = self.ctrl_modes[0]
        msg_m1.ctrl_gains      = self.ctrl_gains 
        msg_m1.ctrl_setpoint   = int( self.setpoints[0] )
        msg_m1.trap_mode       = self.trap_mode
        msg_m1.trap_values     = self.trap_values
        msg_m1.brake_pwm       = 0
        
        msg_m2.ctrl_mode       = self.ctrl_modes[1]
        msg_m2.ctrl_gains      = self.ctrl_gains 
        msg_m2.ctrl_setpoint   = int( self.setpoints[1] )
        msg_m2.trap_mode       = self.trap_mode
        msg_m2.trap_values     = self.trap_values
        msg_m2.brake_pwm       = self.brake_state_M1
        
        self.pub_cmd_m1.publish( msg_m1 )
        self.pub_cmd_m2.publish( msg_m2 )
        
        
    ###########################################
    def pub_feedback( self , msg ):
        """ """
        
        msg_y = dsdm_actuator_sensor_feedback()
        
        # Copy Header
        
        msg_y.header.stamp     = msg.header.stamp 
        msg_y.header.frame_id  = msg.header.frame_id
        
        # Feedback info
        
        msg_y.theta = self.y
        msg_y.w     = self.w
        
        msg_y.test   = self.w[2]
        
        self.pub_y.publish( msg_y )
        
        
#########################################
if __name__ == '__main__':
    
    rospy.init_node('ctl',anonymous=False)
    node = DSDM_CTL()
    rospy.spin()
