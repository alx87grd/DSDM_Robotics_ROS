#!/usr/bin/env python
import rospy
import numpy as np

# Ros stuff
from std_msgs.msg    import Float64MultiArray, Bool, Int32
from dsdm_msgs.msg   import joint_position, ctl_error
from dsdm_msgs.msg   import dsdm_actuator_sensor_feedback, dsdm_actuator_control_inputs
from std_msgs.msg    import Float64MultiArray
from dsdm_msgs.msg   import ctl_error


from flexsea_execute.msg import inputs
from flexsea_execute.msg import outputs




#########################################
class logger(object):
    """
    navigation
    """
    def __init__(self):
        
        self.verbose = True
        
        # Sub 
        self.sub_set            = rospy.Subscriber("setpoint"    , joint_position                , self.update_setpoint      ,  queue_size=1 )
        self.sub_enable         = rospy.Subscriber("enable"      , Bool                          , self.update_enable        ,  queue_size=1 )
        self.sub_mode           = rospy.Subscriber("ctl_mode"    , Int32                         , self.update_mode          ,  queue_size=1 )
        self.sub_state_feedback = rospy.Subscriber("x_hat"       , Float64MultiArray             , self.update_state         ,  queue_size=1 )
        self.sub_u              = rospy.Subscriber("a0/u"        , dsdm_actuator_control_inputs  , self.update_u             , queue_size=1 )
        self.sub_a0             = rospy.Subscriber("a0/y"        , dsdm_actuator_sensor_feedback , self.update_actuator_data , queue_size=1 )
        self.sub_error          = rospy.Subscriber("ctc_error"   , ctl_error                     , self.update_error         , queue_size=1 )
        self.sub_y_m1           = rospy.Subscriber("a0/M1/y"     , outputs                       , self.update_y_m1          , queue_size=1 )
        self.sub_y_m2           = rospy.Subscriber("a0/M2/y"     , outputs                       , self.update_y_m2          , queue_size=1 )
        self.pub_cmd_m1         = rospy.Subscriber("a0/M1/u"     , inputs                        , self.update_u_m1          , queue_size=1      )
        self.pub_cmd_m2         = rospy.Subscriber("a0/M2/u"     , inputs                        , self.update_u_m2          , queue_size=1      )
        
        # Timer
        self.timer              = rospy.Timer( rospy.Duration.from_sec(1.0),    self.save )
        
        
        # INIT
        self.DATA = np.zeros((1,13))
        
        
        name      = 'test'
        
        self.name = '/home/alex/ROS_WS/src/dsdm_robotics/dsdm_sensing/data/' + name + '.npy'
        
    #######################################   
    def save( self , event ):
        """ SAVE DATA at each sec  """ 
        
        np.save( self.name , self.DATA )
        
        print "DATA saved"
        
        
    #######################################   
    def logging( self  ):
        """  """
         
        t     = rospy.get_rostime().to_sec()
        
        data = np.array([[ self.q , self.dq , self.q_d , self.f , self.k , t, self.w1 , self.w2 , self.i1 , self.i2 , self.brake , self.id1 , self.id2 ]])
        
        self.DATA = np.concatenate( ( self.DATA , data )  )
        
    
    #######################################
    def update_u( self, msg ):
        
        self.f = msg.f
        self.k = msg.k
        
        
    #######################################
    def update_actuator_data( self, msg ):
        """ """
        
        self.act_msg = msg
        
        self.w1  = msg.w1
        self.w2  = msg.w2
        self.id1 = msg.id1
        self.id2 = msg.id2
        
    #######################################   
    def update_state( self, msg ):
        """ state feedback """
        
        x = msg.data
        
        self.q  = x[0]
        self.dq = x[3] 
        
    #######################################   
    def update_enable( self, msg ):
        """ enable  """
        
        self.enable = msg.data
        
        
    #######################################   
    def update_mode( self, msg ):
        """ mode """
        
        self.ctl_mode = msg.data
        
    #######################################   
    def update_setpoint( self, msg ):
        """ Load Ref setpoint """
        
        self.ddq_d = np.array( msg.ddq )
        self.dq_d  = np.array( msg.dq  )
        #self.q_d   = np.array( msg.q   )
        
    #####################################
    def update_error ( self, msg ):
        """ Load Ref setpoint """
        
        self.q_d   = msg.actual - msg.e
        self.e     = msg.e
        
    #####################################
    def update_y_m1 ( self, msg ):
        
        self.i1 = msg.current
        
    
    #####################################
    def update_y_m2 ( self, msg ):
        
        self.i2 = msg.current
        
    
    #####################################
    def update_u_m1 ( self, msg ):
        
        pass
        
    
    #####################################
    def update_u_m2 ( self, msg ):
        
        
        self.brake = msg.brake_pwm
        
        self.logging()
            
    
        
        
        
#########################################
if __name__ == '__main__':
    
    rospy.init_node('logger',anonymous=False)
    node = logger()
    rospy.spin()
