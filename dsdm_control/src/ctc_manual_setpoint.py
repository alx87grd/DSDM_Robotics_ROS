#!/usr/bin/env python
import rospy
import numpy as np

# Ros stuff
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg    import Float64MultiArray, Bool
from dsdm_msgs.msg   import joint_position, ctl_error
from dsdm_msgs.msg   import dsdm_actuator_sensor_feedback, dsdm_actuator_control_inputs


from AlexRobotics.dynamic  import CustomManipulator    as CM
from AlexRobotics.control  import RminComputedTorque   as RminCTC


#########################################
class CTC_controller(object):
    """
    navigation
    """
    def __init__(self):
        
        self.verbose = True
        
        # Sub 
        self.sub_ddq            = rospy.Subscriber("ddq_setpoint", joint_position , self.update_setpoint ,  queue_size=1 )
        self.sub_enable         = rospy.Subscriber("enable"      , Bool           , self.update_enable   ,  queue_size=1 )
        self.sub_state_feedback = rospy.Subscriber("x_hat"       , Float64MultiArray , self.update_state ,  queue_size=1 )
        
        # Pub
        #self.pub_control        = rospy.Publisher("u", dsdm_robot_control_inputs , queue_size=1   )     
        self.pub_control        = rospy.Publisher("a0/u", dsdm_actuator_control_inputs , queue_size=1   )  # Temp for testing, only one actuator
        #self.pub_control        = rospy.Publisher("CTC_U", dsdm_actuator_control_inputs , queue_size=1   ) 
        self.pub_e              = rospy.Publisher("ctc_error", ctl_error               , queue_size=1  )
        
        # Timer
        #self.timer              = rospy.Timer( rospy.Duration.from_sec(0.01),    self.callback  )
        
        # Load ROS params
        self.load_params( None )
        
        #self.control_type = 'acc'
        #self.control_type = 'spd'
        self.control_type = 'pos'
        
        # Assign controller
        if self.robot_type == 'BoeingArm':
            self.R       = CM.BoeingArm()
            self.CTC     = RminCTC.RminComputedTorqueController( self.R )
            
        elif self.robot_type == 'pendulum':
            self.R       = CM.TestPendulum()
            self.CTC     = RminCTC.RminComputedTorqueController( self.R )
        
        else:
            print 'Error loading robot type'
        
        # Load params
        self.CTC.w0            = 2.0
        self.CTC.zeta          = 0.7
        self.CTC.n_gears       = 1
        
        # INIT
        self.t_zero   =  rospy.get_rostime()
        self.x        =  np.array([ 0 , 0 , 0 , 0 , 0 , 0])
        self.ddq_d    =  np.array([ 0 , 0 , 0 ])
        self.dq_d     =  np.array([ 0 , 0 , 0 ])
        self.q_d      =  np.array([ 0 , 0 , 0 ])
        self.enable   =  False
        self.setpoint = 0
        self.actual   = 0
        self.e        = 0
        self.de       = 0
        
        
    ###########################################
    def load_params(self, event):
        """ Load param on ROS server """
        
        self.robot_type     = rospy.get_param("robot_type",  'pendulum'  )

        
    #######################################   
    def callback( self, event ):
        """ Timed controller response """
        
        # State feedback
        x = self.x
        
        # Get time
        t_ros = rospy.get_rostime() - self.t_zero
        t     = t_ros.to_sec()
        
        # Compute u
        if self.control_type == 'acc':
            
            u = self.CTC.manual_acc_ctl(  x , t )
            
            # Debug
            self.actual = x[0]
            
        elif self.control_type == 'spd':
            
            # Update setpoint to actual position + desired speed
            [ q , dq ]    = self.R.x2q( self.x  )
            self.CTC.goal = self.R.q2x( q , self.dq_d )
            
            u = self.CTC.fixed_goal_ctl( x  , t )

            # Debug
            self.actual = dq[0]
            self.e = self.CTC.dq_e[0]
            
            
        elif self.control_type == 'pos':
            
            [ q , dq ]    = self.R.x2q( self.x  )
            self.CTC.goal = self.R.q2x( self.q_d , self.dq_d )
            
            u = self.CTC.fixed_goal_ctl( x  , t )

            # Debug
            self.actual = q[0]
            self.e = self.CTC.q_e[0]
        
        # always high-force
        u[ self.R.dof ] = 0
        
        # Publish u
        self.pub_u( u )
        
        ##################
        if self.verbose:
            rospy.loginfo("Controller: e = " + str(self.CTC.q_e) + "de = " + str(self.CTC.dq_e) + "ddr =" + str(self.CTC.ddq_r) + " U = " + str(u) )
        
        
    #######################################   
    def update_state( self, msg ):
        """ state feedback """
        
        
        self.x = msg.data
        
        self.callback( None )
        
        
    #######################################   
    def update_enable( self, msg ):
        """ state feedback """
        
        
        self.enable = msg.data
        
        
        
    #######################################   
    def update_setpoint( self, msg ):
        """ Load Ref setpoint """
        
        if self.control_type == 'acc':
        
            self.ddq_d = msg.ddq
            
            self.CTC.ddq_manual_setpoint = self.ddq_d
            
            # Debug
            self.setpoint = self.ddq_d[0]
            
        elif self.control_type == 'spd':
            
            # read setpoint as target speed and actual position
            self.dq_d = msg.ddq
            
            # Debug
            self.setpoint = self.dq_d[0]
            
        elif self.control_type == 'pos':
            
            # read setpoint as target position
            self.q_d = msg.ddq
            
            # Debug
            self.setpoint = self.q_d[0]
            
            
    #######################################   
    def pub_u( self, u ):
        """ pub control inputs """
        
        #msg = dsdm_robot_control_inputs()
        
        
        #self.pub_control.publish( msg )
        
        # Testing 1-DOF 1-DSDM
        msg = dsdm_actuator_control_inputs()
        
        if self.enable:
        
            msg.f  = u[0] 
            msg.k  = u[3]  # mode
            
        else:
            
            msg.f  = 0
            msg.k  = 1
        
        
        self.pub_control.publish( msg )
        
        # Publish error data
        msg              = ctl_error()
        msg.header.stamp = rospy.Time.now()
        msg.set_point    = self.setpoint
        msg.actual       = self.actual
        msg.e            = self.e  #self.CTC.q_e[0]
        msg.de           = self.de #self.CTC.dq_e[0]
        
        self.pub_e.publish( msg )
        
        if self.verbose:
            print('Error:', self.CTC.q_e[0] )
        
        
            
    
        
        
        
#########################################
if __name__ == '__main__':
    
    rospy.init_node('Master_Controller',anonymous=False)
    node = CTC_controller()
    rospy.spin()
