import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import OverrideRCIn
import numpy as np
import time

# NOTE: this only works for one particular angle convention
def euler_to_quaternion(a):
    x = np.cos(a[0]/2.)*np.cos(a[1]/2.)*np.cos(a[2]/2.) + np.sin(a[0]/2.)*np.sin(a[1]/2.)*np.sin(a[2]/2.)
    y = np.sin(a[0]/2.)*np.cos(a[1]/2.)*np.cos(a[2]/2.) - np.cos(a[0]/2.)*np.sin(a[1]/2.)*np.sin(a[2]/2.)
    z = np.cos(a[0]/2.)*np.sin(a[1]/2.)*np.cos(a[2]/2.) + np.sin(a[0]/2.)*np.cos(a[1]/2.)*np.sin(a[2]/2.)
    w = np.cos(a[0]/2.)*np.cos(a[1]/2.)*np.sin(a[2]/2.) - np.sin(a[0]/2.)*np.sin(a[1]/2.)*np.cos(a[2]/2.)
    

    return [x,y,z,w]

def yaw_to_quaternion(yaw):
    x = np.cos(yaw/2.)
    y = 0
    z = 0
    w = np.sin(yaw/2.)

    return [x,y,z,w]

class Quadcopter(object):

    def __init__(self, disable_signals=False, control_style='position',
                 do_arming=False):
        self.control_style = control_style
        
        self.position = [0,0,0]
        self.vel = [0,0,0]

        self.orientation = [0,0,0,0]
        self.ang_vel = [0,0,0]

        rospy.init_node('quadcopter', anonymous=True,
                        disable_signals=disable_signals)

        # Only arm stuff if the flag is set. Otherwise assume it has already
        # been taken care of elsewhere and is good to go
        if do_arming:
            rospy.wait_for_service('/mavros/cmd/arming')
            rospy.wait_for_service('/mavros/set_mode')

            self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            self.mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        

        if self.control_style == 'position':
            if do_arming:
                # Set the mode to GUIDED
                self.mode_service(0, 'GUIDED')


            self.waypoint_msg = PoseStamped()

            self.pub_waypoint = rospy.Publisher('/mavros/setpoint_position/local',
                                                PoseStamped,
                                                queue_size=10
                                               )
        elif self.control_style == 'velocity':
            if do_arming:
                # Set the mode to STABILIZE
                #self.mode_service(0, 'STABILIZE')
                self.mode_service(0, 'GUIDED')
            
            self.velocity_msg = TwistStamped()

            self.pub_velocity = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',
                                                TwistStamped,
                                                queue_size=10
                                               )
        elif self.control_style == 'alt_hold':
            if do_arming:
                # Set the mode to ALT_HOLD
                self.mode_service(0, 'ALT_HOLD')
            
            self.rc_msg = OverrideRCIn()
            # Initialize all control channels to the centered position
            self.rc_msg.channels[0] = 1500
            self.rc_msg.channels[1] = 1500
            self.rc_msg.channels[2] = 1500
            # Initialize all unused channels to 'ignore and use previous value'
            self.rc_msg.channels[3] = 65535
            self.rc_msg.channels[4] = 65535
            self.rc_msg.channels[5] = 65535
            self.rc_msg.channels[6] = 65535
            self.rc_msg.channels[7] = 65535

            self.pub_rc = rospy.Publisher('/mavros/rc/override',
                                          OverrideRCIn,
                                          queue_size=10
                                         )

        self.sub_odometry = rospy.Subscriber('/mavros/local_position/odom',
                                             Odometry,
                                             self.odo_callback
                                            )
        
        if do_arming:
            # Arm the quadcopter
            self.arm_service(True)

            if self.control_style == 'alt_hold':
                # publish an initial message to prevent the motors from automatically disarming
                self.rc_msg.channels[3] = 1600 # Make it move up a bit to prevent disarm
                self.pub_rc.publish(self.rc_msg)
                time.sleep(.5) # space the messages out slightly
                self.rc_msg.channels[3] = 1600 # Level it out again
                self.pub_rc.publish(self.rc_msg)

        self.step = 0

    def odo_callback(self, data):

        self.position[0] = data.pose.pose.position.x
        self.position[1] = data.pose.pose.position.y
        self.position[2] = data.pose.pose.position.z
        self.vel = data.twist.twist.linear

        self.orientation = data.pose.pose.orientation
        self.ang_vel = data.twist.twist.angular

    def __call__(self, t, x):
        if self.step >= 100:
            if self.control_style == 'position':
                self.waypoint_msg.pose.position.x = x[0]
                self.waypoint_msg.pose.position.y = x[1]
                self.waypoint_msg.pose.position.z = x[2]
                if len(x) > 3:
                    # Set the yaw using quaternions
                    self.waypoint_msg.pose.orientation.x = np.cos(x[3]/2.)
                    self.waypoint_msg.pose.orientation.w = np.sin(x[3]/2.)

                self.pub_waypoint.publish(self.waypoint_msg)
            elif self.control_style == 'velocity':
                self.velocity_msg.twist.linear.x = x[0]
                self.velocity_msg.twist.linear.y = x[1]
                self.velocity_msg.twist.linear.z = x[2]
                if len(x) > 3:
                    self.velocity_msg.twist.angular.z = x[3]

                self.pub_velocity.publish(self.velocity_msg)
            elif self.control_style == 'alt_hold':
                # Scale an input between -1 and 1 to 1000, 2000
                # Make sure the input does not exceed those bounds
                self.rc_msg.channels[0] = max(1000,min(x[0]*500+1500,2000))
                self.rc_msg.channels[1] = max(1000,min(x[1]*500+1500,2000))
                self.rc_msg.channels[2] = max(1000,min(x[2]*500+1500,2000))

                self.pub_rc.publish(self.rc_msg)
            self.step = 0
        self.step += 1

        return self.position
        #return [self.position[0], self.position[1], self.position[2]]
