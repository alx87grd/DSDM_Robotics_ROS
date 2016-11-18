#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg    import Float64MultiArray, Bool, Int32
from dsdm_msgs.msg   import dsdm_actuator_control_inputs

from AlexRobotics.dynamic  import Prototypes             as Proto
from AlexRobotics.control  import RolloutComputedTorque  as RollCTC

#########################################
class Robot_controller(object):
    """
    navigation
    """
    def __init__(self):
        
        self.verbose = False
        
        self.sub_joy      = rospy.Subscriber("joy"   , Joy               , self.joy_callback   , queue_size=1 )
        self.sub_state    = rospy.Subscriber("x_hat" , Float64MultiArray , self.state_callback , queue_size=1 )
        
        self.pub_a0u      = rospy.Publisher("a0/u", dsdm_actuator_control_inputs , queue_size=1  )
        self.pub_a1u      = rospy.Publisher("a1/u", dsdm_actuator_control_inputs , queue_size=1  )
        self.pub_a2u      = rospy.Publisher("a2/u", dsdm_actuator_control_inputs , queue_size=1  )
        
        
        # Load ROS params
        self.load_params( None )
        
        
        # Load robot model for the right configuration
        if self.robot_config == 'wrist-only':
            self.R = Proto.SingleRevoluteDSDM()
            
        elif self.robot_config == 'dual-plane' :
            self.R = Proto.TwoPlanarSerialDSDM()
            
        else:
            self.R = None
            
        # Operating Modes
        self.enable = False
        self.mode   = 0
        self.modes  = [ 'open_loop ' , 'acceleration' , 'speed' , 'position' , 'auto' ]
        
        # Init DSDM msgs
        self.f = np.array([0.,0.,0.])
        self.k = np.array([1 ,1 ,1 ]) # High force mode
        self.n = np.array([0.,0.,0.])
        
        # Sub message init
        self.joy = Joy()                 # memory for last joy msg
        self.x   = np.zeros( self.R.n )  # Memory for robots state feedback
        
        
        # Timers
        self.t_joy   =  rospy.get_rostime()  # last joy change
        
    ###########################################
    def load_params(self, event):
        """ Load param on ROS server """
        
        self.robot_type     = rospy.get_param("robot_type"  ,  'pendulum'    )
        self.robot_config   = rospy.get_param("robot_config",  'wrist-only'  )
        
        
    #######################################   
    def joy_callback( self, msg ):
        """ Read joystick states """
        
        t_last = self.t_joy
        t_now  = rospy.get_rostime()
        
        ######################
        # Enabled button state
        if ( msg.axes[2] < 0 ):
            self.enable = True
            
        else:
            self.enable = False
            
        #####################
        # Update control mode
            
        if ( ( t_now -  t_last ).to_sec() > 0.5 ) :  # Avoid double detection
            if msg.buttons[13] :
                self.mode = self.mode + 1
                print('--------- Control Mode set to : ' + self.modes[ self.mode ]  + ' ----------------------')
                
            elif msg.buttons[14] :
                self.mode = self.mode - 1
                print('--------- Control Mode set to : ' + self.modes[ self.mode ]  + ' ----------------------' )
        
        #########################
        # Save last buttons states
        self.joy   = msg
        self.t_joy = t_now
        
        
    #######################################   
    def state_callback( self, msg ):
        """ state feedback """
        
        if self.robot_config == 'wrist-only':
            # Last DoF only
            self.x = np.array([ msg.data[2]  , msg.data[5]  ])
            
        elif self.robot_config == 'dual-plane' :
            # Last two DoF only
            self.x = np.array([ msg.data[1] , msg.data[2]  , msg.data[4] , msg.data[5]  ])
            
        else:
            # All DoF
            self.x = msg.data

        # Asynchrone control
        #self.control_callback( None )
        
            
    
    #######################################   
    def control_callback( self, msg ):
        """ Main control loop """
        
        #######################################
        # Open Loop
        if ( self.mode == 0 ):
            # Pick set_point with joysticks gain
        
            """ Ball screw DoF """
            #self.f  =    msg.axes[1] * 0.2
            self.f[0] = self.joy.buttons[4] * 0.2 + self.joy.buttons[5] * -0.2 
                
            # Pick mode with trigger
            if ( self.joy.axes[5] < 0):
                self.k[0] = 0
            else:
                self.k[0] = 1
                
            """ Shoulder """
            
            self.f[1] = self.joy.axes[1] * 0.2
            self.k[1] = not( self.joy.buttons[0] )
            self.n[1] = self.joy.axes[0] * 0.5
            
            """ Elbow """
            
            self.f[2] = self.joy.axes[4] * 0.2
            self.k[2] = not( self.joy.buttons[1] )
            self.n[2] = self.joy.axes[3] * 0.5
            
        #######################################
        # Closed Loop Acc
        elif ( self.mode == 1 ):
            
            print self.x
        
        
        #######################################
        # Other
        
        else:
            self.f = np.array([0.,0.,0.])
            self.k = np.array([1,1,1])
            self.n = np.array([0.,0.,0.])
        
        
        self.pub_u_msg()
        
        if self.verbose:
            
            print( 'Ctrl mode: ', self.k , ' Set point: ', self.f )
            
            
    #######################################   
    def pub_u_msg( self ):
        """ pub actuator cmd """
        
        msg0 = dsdm_actuator_control_inputs()
        msg1 = dsdm_actuator_control_inputs()
        msg2 = dsdm_actuator_control_inputs()
        
        # Load data only if enabled
        if self.enable:
            
            msg0.f = self.f[0]
            msg0.k = self.k[0]
            
            msg1.f = self.f[1]
            msg1.k = self.k[1]
            msg1.n = self.n[1]
            
            msg2.f = self.f[2]
            msg2.k = self.k[2]
            msg2.n = self.n[2]
            
        else:
            
            msg0.f = 0
            msg0.k = 1 # Brake engaged
            
            msg1.f = 0
            msg1.k = 1 # Brake engaged
            msg1.n = 0
            
            msg2.f = 0
            msg2.k = 1 # Brake engaged
            msg2.n = 0
            
        
        self.pub_a0u.publish( msg0 )
        self.pub_a1u.publish( msg1 )
        self.pub_a2u.publish( msg2 )
        


#########################################
if __name__ == '__main__':
    
    rospy.init_node('Master',anonymous=False)
    node = Robot_controller()
    rospy.spin()
