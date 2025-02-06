#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import pygame
import sys

def init_pygame():
    pygame.init()
    window = pygame.display.set_mode((300, 200))
    pygame.display.set_caption("AGV Arrow Control")
    return window

def main():
    rospy.init_node('arrow_teleop')
    pub = rospy.Publisher('/agv/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    window = init_pygame()
    linear_speed = 0.5
    angular_speed = 1.0

    print("AGV Arrow Key Control")
    print("--------------------")
    print("Use arrow keys to control the AGV:")
    print("↑: Move forward")
    print("↓: Move backward")
    print("←: Turn left")
    print("→: Turn right")
    print("Spacebar: Stop")
    print("Q: Quit")

    try:
        while not rospy.is_shutdown():
            cmd_vel = Twist()
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    sys.exit()
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_q:
                    sys.exit()

            keys = pygame.key.get_pressed()
            
            if keys[pygame.K_UP]:
                cmd_vel.linear.x = linear_speed
            elif keys[pygame.K_DOWN]:
                cmd_vel.linear.x = -linear_speed
            
            if keys[pygame.K_LEFT]:
                cmd_vel.angular.z = angular_speed
            elif keys[pygame.K_RIGHT]:
                cmd_vel.angular.z = -angular_speed
            
            if keys[pygame.K_SPACE]:
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = 0

            pub.publish(cmd_vel)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        pygame.quit()

if __name__ == '__main__':
    main()
