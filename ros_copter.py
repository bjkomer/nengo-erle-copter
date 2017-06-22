import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
import numpy as np

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

    def __init__(self, disable_signals=False, control_style='position'):
        self.control_style = control_style
        
        self.position = [0,0,0]
        self.vel = [0,0,0]

        self.orientation = [0,0,0,0]
        self.ang_vel = [0,0,0]

        rospy.init_node('quadcopter', anonymous=True,
                        disable_signals=disable_signals)

        if self.control_style == 'position':
            self.waypoint_msg = PoseStamped()

            self.pub_waypoint = rospy.Publisher('/mavros/setpoint_position/local',
                                                PoseStamped,
                                                queue_size=10
                                               )
        elif self.control_style == 'velocity':
            self.velocity_msg = TwistStamped()

            self.pub_velocity = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',
                                                TwistStamped,
                                                queue_size=10
                                               )

        self.sub_odometry = rospy.Subscriber('/mavros/local_position/odom',
                                             Odometry,
                                             self.odo_callback
                                            )

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
            self.step = 0
        self.step += 1

        return self.position
        #return [self.position[0], self.position[1], self.position[2]]
