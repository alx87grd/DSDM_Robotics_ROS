#!/usr/bin/env python
import rospy
import numpy as np

# Ros stuff
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg    import Float64MultiArray, Bool
from dsdm_msgs.msg   import joint_position
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
        
        # Sub / pub
        self.sub_ddq            = rospy.Subscriber("ddq_setpoint", joint_position , self.update_setpoint ,  queue_size=1 )
        self.sub_enable         = rospy.Subscriber("enable"      , Bool           , self.update_enable   ,  queue_size=1 )
        #self.pub_control        = rospy.Publisher("u", dsdm_robot_control_inputs , queue_size=1   )     
        self.pub_control        = rospy.Publisher("a0/u", dsdm_actuator_control_inputs , queue_size=1   )  # Temp for testing, only one actuator
        
        
        # Timer
        #self.timer              = rospy.Timer( rospy.Duration.from_sec(0.01),    self.callback  )
            
        self.sub_state_feedback = rospy.Subscriber("x_hat", Float64MultiArray , self.update_state ,  queue_size=1      )
        
        # Load ROS params
        self.load_params( None )
        
        #self.control_type = 'acc'
        self.control_type = 'spd'
        
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
        self.t_zero =  rospy.get_rostime()
        self.x      =  np.array([ 0 , 0 , 0 , 0 , 0 , 0])
        self.ddq_d  =  np.array([ 0 , 0 , 0 ])
        self.dq_d   =  np.array([ 0 , 0 , 0 ])
        self.enable =  False
        
        
    ###########################################
    def load_params(self, event):
        """ Load param on ROS server """
        
        self.robot_type     = rospy.get_param("robot_type",  'BoeingArm'  )

        
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
            
        elif self.control_type == 'spd':
            
            # Update setpoint to actual position + desired speed
            [ q , dq ]    = self.R.x2q( self.x  )
            self.CTC.goal = self.R.q2x( q , self.dq_d )
            
            u = self.CTC.fixed_goal_ctl( x  , t )
            
        u[self.R.dof] = 1
        
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
            
        elif self.control_type == 'spd':
            
            # read setpoint as target speed and actual position
            self.dq_d = msg.ddq
            
            
    #######################################   
    def pub_u( self, u ):
        """ pub control inputs """
        
        #msg = dsdm_robot_control_inputs()
        
        
        #self.pub_control.publish( msg )
        
        # Testing 1-DOF 1-DSDM
        msg = dsdm_actuator_control_inputs()
        
        if self.enable:
        
            msg.f  = -u[0] 
            msg.k  = u[3]  # mode
            
        else:
            
            msg.f  = 0
            msg.k  = 1
        
        
        self.pub_control.publish( msg )
        
        
            
    
        
        
        
#########################################
if __name__ == '__main__':
    
    rospy.init_node('Master_Controller',anonymous=False)
    node = CTC_controller()
    rospy.spin()
