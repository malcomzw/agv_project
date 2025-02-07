#!/usr/bin/env python3

import unittest
import rospy
import rostest
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import time
import sys

class SimulationTest(unittest.TestCase):
    def setUp(self):
        rospy.init_node('simulation_test', anonymous=True)
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
        rospy.loginfo(f"Odometry received: Position x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}")

    def tf_callback(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == 'odom' and transform.child_frame_id == 'base_link':
                self.tf_received = True
                rospy.loginfo(f"TF transform received: {transform}")

    def test_robot_movement(self):
        # Create movement command
        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = 0.2
        
        # Publish command
        self.cmd_vel_pub.publish(twist)
        
        # Wait for response
        timeout = time.time() + 30  # 30 seconds timeout
        while not (self.odom_received and self.tf_received) and time.time() < timeout:
            time.sleep(0.1)
        
        rospy.loginfo(f'Odometry received: {self.odom_received}, TF received: {self.tf_received}')
        
        # Check if we received odometry and tf data
        self.assertTrue(self.odom_received, "No odometry data received within timeout")
        self.assertTrue(self.tf_received, "No transform data received within timeout")

    def test_simulation_startup(self):
        # Verify basic simulation startup conditions
        self.assertTrue(rospy.get_master().getPid() != 0, "ROS Master not running")
        self.assertTrue(self.cmd_vel_pub.get_num_connections() > 0, "No subscribers for cmd_vel")

def simulation_test():
    rostest.rosrun('agv_sim', 'simulation_test', SimulationTest)

if __name__ == '__main__':
    try:
        simulation_test()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Test Interrupted")
        sys.exit(1)
