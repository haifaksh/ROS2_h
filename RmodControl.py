#! /usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from nav_msgs.msg import Odometry
import numpy as np

TIME_PERIOD = 5
LASER_THRESHOLD = 1.5

class ControlRmod3(Node):
    def __init__(self):
        super().__init__("rmod3_node")
        # Create publisher
        self.pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.create_timer(TIME_PERIOD, self.move_callback)
        self.cmd = Twist()

        self.robot_states_dict = {
            0: 'Find the wall',
            1: 'Turn Left',
            2: 'Turn Right',
            3: 'Weired situation'
        }

        self.robot_states = 0
        
        
        # Create subscriber for odom
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.update_angle_callback, 1)
        
        # Initialize angles
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        # Create subscriber for laserscan
        self.sub = self.create_subscription(LaserScan, 
                                            '/rmod3/laser_scan', 
                                            self.laser_callback, 
                                            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    def laser_callback(self, laser_msg):
        self.regions = {
            'right': min(min(laser_msg.ranges[0:45]),10),
            'fright': min(min(laser_msg.ranges[46:60]),10),
            'front': min(min(laser_msg.ranges[61:100]),10),
            'fleft': min(min(laser_msg.ranges[101:134]),10),
            'left': min(min(laser_msg.ranges[135:180]),10)
        }
        

    def update_angle_callback(self, odom_msg):
        self.current_position = odom_msg.pose.pose
        orientation_list = [self.current_position.orientation.x,
                            self.current_position.orientation.y,
                            self.current_position.orientation.z,
                            self.current_position.orientation.w
                            ]
        (self.roll, self.pitch, self.yaw) = self.euler_from_quaternion(orientation_list)
        #self.get_logger().info('yaw angle: "%s"'%self.yaw)

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """

        sinr_cosp = 2 * (quaternion[3] * quaternion[0] + quaternion[1] * quaternion[2])
        cosr_cosp = 1 - 2 * (quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1])
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (quaternion[3] * quaternion[1] - quaternion[2] * quaternion[0])
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (quaternion[3] * quaternion[2] + quaternion[0] * quaternion[1])
        cosy_cosp = 1 - 2 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2])
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    
    def turn(self, target_angle):
        # Convert from degrees to radians
        target_angle = target_angle*np.pi/180

        if target_angle > 2*np.pi:
            target_angle -= 2*np.pi
        self.error_angle = target_angle - self.yaw
        self.get_logger().info('error_angle: "%s"'%self.error_angle)
        command_angle = 0.5*self.error_angle
        command_vel = 0.0

    def move_robot(self):
        if self.robot_states == 0:
            self.cmd.linear.x = 0.5
            self.cmd.angular.z = -0.8
        elif self.robot_states == 1:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 1.8
        elif self.robot_states == 2:
            self.cmd.linear.x = 0.5
            self.cmd.angular.z = 0.0
        elif self.robot_states == 3:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0

        #self.get_logger().info(str(self.robot_states_dict[self.robot_states_dict]))
        self.pub.publish(self.cmd)
        self.get_logger().info('Sending "%s"'%self.cmd) 

        

    def move_callback(self):

        if self.regions['front'] > LASER_THRESHOLD and self.regions['fleft'] > LASER_THRESHOLD and self.regions['fright'] > LASER_THRESHOLD:
            self.robot_states = 0
        elif self.regions['front'] < LASER_THRESHOLD and self.regions['fleft'] > LASER_THRESHOLD and self.regions['fright'] > LASER_THRESHOLD:
            self.robot_states = 1
        elif self.regions['front'] > LASER_THRESHOLD and self.regions['fleft'] > LASER_THRESHOLD and self.regions['fright'] < LASER_THRESHOLD:
            self.robot_states = 2
        elif self.regions['front'] > LASER_THRESHOLD and self.regions['fleft'] < LASER_THRESHOLD and self.regions['fright'] > LASER_THRESHOLD:
            self.robot_states = 0
        elif self.regions['front'] < LASER_THRESHOLD and self.regions['fleft'] > LASER_THRESHOLD and self.regions['fright'] < LASER_THRESHOLD:
            self.robot_states = 1
        elif self.regions['front'] < LASER_THRESHOLD and self.regions['fleft'] < LASER_THRESHOLD and self.regions['fright'] > LASER_THRESHOLD:
            self.robot_states = 1
        elif self.regions['front'] < LASER_THRESHOLD and self.regions['fleft'] < LASER_THRESHOLD and self.regions['fright'] < LASER_THRESHOLD:
            self.robot_states = 1
        elif self.regions['front'] > LASER_THRESHOLD and self.regions['fleft'] < LASER_THRESHOLD and self.regions['fright'] < LASER_THRESHOLD:
            self.robot_states = 0
        else:
            self.get_logger().info('Weired Situation H')
            self.robot_states = 3

        self.move_robot()

    
            
        
def main():
    rclpy.init()
    control_rmod =ControlRmod3()

    try:
        rclpy.spin(control_rmod)
    except KeyboardInterrupt:
        control_rmod.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

