#!/usr/bin/env python
import rospy
import numpy as np

# Ros stuff
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg    import Float64MultiArray
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
        #self.pub_control        = rospy.Publisher("u", dsdm_robot_control_inputs , queue_size=1   )
        self.pub_control        = rospy.Publisher("a0/u", dsdm_actuator_control_inputs , queue_size=1   )
        # Timer
        self.timer              = rospy.Timer( rospy.Duration.from_sec(0.1),    self.callback  )
        
        # Temp for testing, only one actuator
        self.sub_state_feedback = rospy.Subscriber("x_hat", Float64MultiArray , self.update_state ,  queue_size=1      )
        
        # Assign controller
        self.R       = CM.BoeingArm()
        self.CTC     = RminCTC.RminComputedTorqueController( self.R )
        
        # Load params
        self.CTC.w0            = 2.0
        self.CTC.zeta          = 0.7
        self.CTC.n_gears       = 2
        
        self.n_DOF = 3
        
        # INIT
        self.t_zero =  rospy.get_rostime()
        self.x      =  np.array([ 0 , 0 , 0 , 0 , 0 , 0])
        self.ddq_d  =  np.array([ 0 , 0 , 0 ])
        
        
    #######################################   
    def update_state( self, msg ):
        """ state feedback """
        
        
        self.x = msg.data
        
        
        
    #######################################   
    def update_setpoint( self, msg ):
        """ Load Ref setpoint """
        
        self.ddq_d = msg.ddq
        
        self.CTC.ddq_manual_setpoint = self.ddq_d
        
            
    #######################################   
    def pub_u( self, u ):
        """ pub control inputs """
        
        #msg = dsdm_robot_control_inputs()
        
        
        #self.pub_control.publish( msg )
        
        # Testing 1-DOF 1-DSDM
        msg = dsdm_actuator_control_inputs()
        
        msg.f  = u[0] * 10000 / 0.03  # effort
        msg.k  = u[3]  # mode
        
        
        self.pub_control.publish( msg )
        
        
            
    #######################################   
    def callback( self, event ):
        """ Timed controller response """
        
        # State feedback
        x = self.x
        
        # Get time
        t_ros = rospy.get_rostime() - self.t_zero
        t     = t_ros.to_sec()
        
        # Compute u
        u = self.CTC.manual_acc_ctl(  x , t )
        
        # Publish u
        self.pub_u( u )
        
        ##################
        if self.verbose:
            rospy.loginfo("Controller: Published u = " + str(u) )
        
        
        
#########################################
if __name__ == '__main__':
    
    rospy.init_node('Master_Controller',anonymous=False)
    node = CTC_controller()
    rospy.spin()
