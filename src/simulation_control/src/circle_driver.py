#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist


def move():
    rospy.init_node('circle_driver', anonymous=True)
    vel_pub = rospy.Publisher('husky_velocity_controller/cmd_vel', Twist, queue_size=100)
    vel_msg = Twist()

    rate = rospy.Rate(10)

    start_time = time.time()

    while not rospy.is_shutdown():
        current_time = time.time()
        elapsed_time = current_time - start_time

        if elapsed_time <= 10.0:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = -2.0
            vel_pub.publish(vel_msg)

        elif elapsed_time > 10.0 and elapsed_time <= 20.0:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 2.0
            vel_pub.publish(vel_msg)

        else:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            vel_pub.publish(vel_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException: 
        pass