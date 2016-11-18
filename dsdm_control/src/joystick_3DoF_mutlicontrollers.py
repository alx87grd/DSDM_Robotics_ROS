#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg    import Float64MultiArray, Bool, Int32
from dsdm_msgs.msg   import dsdm_actuator_control_inputs

from AlexRobotics.dynamic  import Prototypes             as Proto
from AlexRobotics.control  import RminComputedTorque     as RminCTC
from AlexRobotics.control  import RolloutComputedTorque  as RollCTC

#########################################
class Robot_controller(object):
    """
    General class for controlling dsdm robot control
    """
    def __init__(self):
        
        self.verbose = False
        
        # Suscribers
        self.sub_joy      = rospy.Subscriber("joy"   , Joy               , self.joy_callback   , queue_size=1 )
        self.sub_state    = rospy.Subscriber("x_hat" , Float64MultiArray , self.state_callback , queue_size=1 )
        
        # Publishers
        self.pub_a0u      = rospy.Publisher("a0/u", dsdm_actuator_control_inputs , queue_size=1  )
        self.pub_a1u      = rospy.Publisher("a1/u", dsdm_actuator_control_inputs , queue_size=1  )
        self.pub_a2u      = rospy.Publisher("a2/u", dsdm_actuator_control_inputs , queue_size=1  )
        
        # Load ROS params
        self.load_params( None )
        
        # Operating Modes
        self.enable = False
        self.mode   = 0
        self.modes  = [ 'open_loop ' , 'acceleration' , 'speed' , 'position' , 'custom1' , 'custom2' , 'custom3' ]
        
        # Init DSDM msgs
        self.f = np.array([0.,0.,0.])
        self.k = np.array([1 ,1 ,1 ]) # High force mode
        self.n = np.array([0.,0.,0.])
        
        # Sub message init
        self.joy = Joy()                 # memory for last joy msg
        self.x   = np.zeros( self.R.n )  # Memory for robots state feedback
        
        # Timers
        self.t_joy   =  rospy.get_rostime()  # last joy change
        self.t_zero  =  rospy.get_rostime()
        
        # Init setpoints
        
        self.fd = np.array([0.,0.,0.])
        self.kd = np.array([1 ,1 ,1 ]) # High force mode
        self.nd = np.array([0.,0.,0.])
        
        self.ddq_d = np.zeros( self.R.dof )
        self.dq_d  = np.zeros( self.R.dof )
        self.q_d   = np.zeros( self.R.dof )
        
        
    ###########################################
    def load_params(self, event):
        """ Load param on ROS server """
        
        self.robot_type     = rospy.get_param("robot_type"  ,  'pendulum'    )
        self.robot_config   = rospy.get_param("robot_config",  'wrist-only'  )
        self.robot_ctl      = rospy.get_param("controller",  'RfixCTC'       )
        self.fixed_mode     = rospy.get_param("fixed_mode",  1               )
        
        
        ###############################################
        # Load robot model for the right configuration
        if self.robot_config == 'wrist-only':
            self.R = Proto.SingleRevoluteDSDM()
            
        elif self.robot_config == 'dual-plane' :
            self.R = Proto.TwoPlanarSerialDSDM()
            
        else:
            self.R = None
            
        ###############################################
        # Load controller
        if self.robot_ctl == 'RfixCTC' :
            self.Ctl = RminCTC.RfixComputedTorqueController( self.R , self.fixed_mode )
            
        elif self.robot_ctl == 'RminCTC' :
            self.Ctl = RminCTC.RminComputedTorqueController( self.R )
            
        elif self.robot_ctl == 'RfixSLD' :
            self.Ctl = RminCTC.RfixSlidingModeController( self.R , self.fixed_mode )
            
        elif self.robot_ctl == 'RminSLD' :
            self.Ctl = RminCTC.RminSlidingModeController( self.R )
            
        elif self.robot_ctl == 'RollCTC' :
            self.Ctl = RollCTC.RolloutComputedTorqueController( self.R )
            
        elif self.robot_ctl == 'RollSLD' :
            self.Ctl = RollCTC.RolloutSlidingModeController( self.R )
            
        else:
            self.Ctl = None
            
        
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
            
        if ( ( t_now -  t_last ).to_sec() > 0.2 ) :  # Avoid double detection
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
        
                
        ############################################
        # update setpoints for the possible modes
                
        """ Open Loop Setpoints """
        
        """ Ball screw DoF """
        #self.f  =    msg.axes[1] * 0.2
        self.fd[0] = self.joy.buttons[4] * 0.2 + self.joy.buttons[5] * -0.2 
            
        # Pick mode with trigger
        if ( self.joy.axes[5] < 0):
            self.kd[0] = 0
        else:
            self.kd[0] = 1
            
        """ elbow """            
        self.fd[1] = self.joy.axes[1] * 0.2
        self.kd[1] = not( self.joy.buttons[0] )
        self.nd[1] = self.joy.axes[0] * 0.5
        
        """ wrist """            
        self.fd[2] = self.joy.axes[4] * 0.2
        self.kd[2] = not( self.joy.buttons[1] )
        self.nd[2] = self.joy.axes[3] * 0.5
        
        ###################
        
        if self.robot_config == 'wrist-only':
            
            self.ddq_d[0] = self.joy.axes[4] * np.pi * 10
            self.dq_d[0]  = self.joy.axes[4] * np.pi * 1
            self.q_d[0]   = self.joy.axes[4] * np.pi * 1
            
        elif self.robot_config == 'dual-plane' :
            
            self.ddq_d[0] = self.joy.axes[1] * np.pi * 10
            self.dq_d[0]  = self.joy.axes[1] * np.pi * 1
            self.q_d[0]   = self.joy.axes[1] * np.pi * 1
            
            self.ddq_d[1] = self.joy.axes[4] * np.pi * 10
            self.dq_d[1]  = self.joy.axes[4] * np.pi * 1
            self.q_d[1]   = self.joy.axes[4] * np.pi * 1
                
        
        
        
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
        self.control_callback( None )
        
            
    
    #######################################   
    def control_callback( self, msg ):
        """ Main control loop """
        
        ######################################
        
        # State feedback
        x             = self.x
        [ q , dq ]    = self.R.x2q( self.x  )
        
        # Get time
        t_ros = rospy.get_rostime() - self.t_zero
        t     = t_ros.to_sec()
        
        #######################################
        # Open Loop
        if ( self.mode == 0 ):
            # Pick commands with joysticks setpoints
            
            self.f = self.fd
            self.k = self.kd
            self.n = self.nd
            
        #######################################
        # Closed Loop Acc
        elif ( self.mode == 1 ):
            
            self.Ctl.ddq_manual_setpoint = self.ddq_d
            
            u = self.Ctl.manual_acc_ctl(  x , t )
            
            self.u2fkn( u ) # convert to actuator cmds
            
            
        #######################################
        # Closed Loop Speed
        elif ( self.mode == 2 ):
            
            self.Ctl.goal = self.R.q2x( q , self.dq_d )
            
            u = self.Ctl.fixed_goal_ctl( x  , t )
            
            self.u2fkn( u ) # convert to actuator cmds
            
            
        #######################################
        # Closed Loop Position
        elif ( self.mode == 3 ):
            
            self.Ctl.goal = self.R.q2x( self.q_d , np.zeros( self.R.dof ) )
            
            u = self.Ctl.fixed_goal_ctl( x  , t )
            
            self.u2fkn( u ) # convert to actuator cmds
            
            
        #######################################
        # Custom 1
        elif ( self.mode == 4 ):
            
            print x
            
        #######################################
        # Custom 2
        elif ( self.mode == 5 ):
            
            pass
            
        #######################################
        # Custom 3
        elif ( self.mode == 6 ):
            
            pass
        
        
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
    def u2fkn( self , u ):
        """ Convert input array into individual actuator commands """
        
        self.f = np.array([0.,0.,0.])
        self.k = np.array([1 ,1 ,1 ]) # High force mode
        self.n = np.array([0.,0.,0.])
        
        if self.robot_config == 'wrist-only':
            
            """ wrist """            
            self.f[2] = u[0]
            self.n[2] = 0
            
            """ Dsicrete mode """
            if u[1] == self.R.R[0] :
                self.k[2] = 0
            else:
                self.k[2] = 1
                
            
        elif self.robot_config == 'dual-plane' :
            
            """ elbow """            
            self.f[1] = u[0]
            self.n[1] = 0
            
            """ wrist """            
            self.f[2] = u[1]
            self.n[2] = 0
            
            """ Dsicrete mode """
            if u[2] == 0 :
                self.k[1] = 0
                self.k[2] = 0
                
            elif u[2] == 1 :
                self.k[1] = 0
                self.k[2] = 1
                
            elif u[2] == 2 :
                self.k[1] = 1
                self.k[2] = 0
                
            else:
                self.k[1] = 1
                self.k[2] = 1
            
        
        
        
        
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
