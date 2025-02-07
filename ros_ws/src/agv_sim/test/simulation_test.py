#!/usr/bin/env python3

import unittest
import rospy
import rostest
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import time

class SimulationTest(unittest.TestCase):
    def setUp(self):
        rospy.init_node('simulation_test')
        self.cmd_vel_pub = rospy.Publisher('/agv/cmd_vel', Twist, queue_size=1)
        self.odom_received = False
        self.tf_received = False
        
        # Subscribe to odometry and tf
        rospy.Subscriber('/agv/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/tf', TFMessage, self.tf_callback)
        
        # Wait for publishers and subscribers to be ready
        time.sleep(5)
        rospy.loginfo('Waiting for topics to be ready...')

    def odom_callback(self, msg):
        self.odom_received = True

    def tf_callback(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == 'odom' and transform.child_frame_id == 'base_link':
                self.tf_received = True

    def test_robot_movement(self):
        # Create movement command
        twist = Twist()
        twist.linear.x = 0.5
        
        # Publish command
        self.cmd_vel_pub.publish(twist)
        
        # Wait for response
        timeout = time.time() + 10  # 10 seconds timeout
        while not (self.odom_received and self.tf_received) and time.time() < timeout:
            time.sleep(0.1)
        
        rospy.loginfo('Odometry received: %s, TF received: %s', self.odom_received, self.tf_received)
        
        # Check if we received odometry and tf data
        self.assertTrue(self.odom_received, "No odometry data received")
        self.assertTrue(self.tf_received, "No transform data received")

if __name__ == '__main__':
    rostest.rosrun('agv_sim', 'simulation_test', SimulationTest)
