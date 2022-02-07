#!/usr/bin/env python

import rospy
import time
import csv
from move_base_msgs.msg import MoveBaseActionGoal

def get_coord():
    coords = []
    with open('coordinates.csv', 'r') as file:
        csvFile = csv.reader(file)
    
        for lines in csvFile:
            print(lines)

    # print(lines)
    coords = list(map(float, lines))
    # for x in lines:
    #     coords.append(float(x))
    
    return coords

def use_coord():
    rospy.init_node('coord_nav', anonymous=True)
    coord_pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=100)
    coord_msg = MoveBaseActionGoal()

    rate = rospy.Rate(10)

    coords = get_coord()
    
    pos_x = coords[0]
    pos_y = coords[1]
    ori_z = coords[2]
    ori_w = coords[3]

    time.sleep(0.4)

    coord_msg.goal.target_pose.header.frame_id = 'odom'
    coord_msg.goal.target_pose.pose.position.x = pos_x
    coord_msg.goal.target_pose.pose.position.y = pos_y
    coord_msg.goal.target_pose.pose.orientation.z = ori_z
    coord_msg.goal.target_pose.pose.orientation.w = ori_w

    coord_pub.publish(coord_msg)

    coord_list = [pos_x, pos_y, ori_z, ori_w]
    coord_list = list(map(str, coord_list))

    with open('path/to/csv_file', 'w') as file:
        writer = csv.writer(file)

        writer.writerow(coord_list)

        

if __name__ == '__main__':
    try:
        use_coord()
    except rospy.ROSInterruptException: pass