#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg    import Bool
from dsdm_msgs.msg   import joint_position

#########################################
class nav(object):
    """
    navigation
    """
    def __init__(self):
        
        self.verbose = True
        self.dof     = 3
        
        self.pub_enable   = rospy.Publisher("enable", Bool  , queue_size=1                       )
        self.pub_u        = rospy.Publisher("ddq_setpoint", joint_position  , queue_size=1       )
        self.sub_joy      = rospy.Subscriber("joy", Joy , self.joy_callback , queue_size=1       )
        
        
        self.ddq_d        = np.zeros( self.dof )
        
    #######################################   
    def joy_callback( self, msg ):
        """ """
        
        # Pick ctrl_mode with button state
        if ( msg.buttons[0] == 1 ):
            enable = True
        else:
            enable = False
        
        ######################
        if enable:
            # Pick set_point with joysticks gain
            self.ddq_d[0]  =    msg.axes[1] #int( msg.axes[1] * 1  )
                
                
        else:
            self.ddq_d[0]  = 0 
        ###########################
        
        self.pub_e( enable )
        self.pub_u_msg()
        
        if self.verbose:
            
            print( 'ddq_d: ', self.ddq_d )
            
            
    #######################################   
    def pub_u_msg( self ):
        """ pub actuator cmd """
        
        msg     = joint_position()
        
        msg.ddq = self.ddq_d
        
        self.pub_u.publish( msg )
        
        
    #######################################   
    def pub_e( self , enable ):
        """ pub actuator cmd """
        
        msg     = Bool()
        
        msg.data = enable
        
        self.pub_enable.publish( msg )
        

            


#########################################
if __name__ == '__main__':
    
    rospy.init_node('nav',anonymous=False)
    node = nav()
    rospy.spin()
