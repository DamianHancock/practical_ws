#!/usr/bin/env python

import rospy
import coord_nav
import actionlib
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseActionResult, MoveBaseActionFeedback


def main():
    cn = coord_nav.CoordNav()
    publish = True
    i = 0
    cn.set_goals()

    rospy.init_node('coord_nav', anonymous=True)
    coord_pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=100)
    rospy.Subscriber('move_base/result', MoveBaseActionResult, cn.status_cb)
    rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, cn.pos_cb)
   
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    coord_msg = MoveBaseActionGoal()
    goal = coord_msg.goal.target_pose
    goals = cn.get_coord()
    pos_x, pos_y, ori_z, ori_w = goals[i]
    
    rate = rospy.Rate(20)

    start_time = time.time()

    while not rospy.is_shutdown():
        current_time = time.time()
        cn.elapsed_time = current_time - start_time
        if publish:
            goal.header.frame_id = 'odom'
            goal.pose.position.x = pos_x
            goal.pose.position.y = pos_y
            goal.pose.orientation.z = ori_z
            goal.pose.orientation.w = ori_w
            coord_pub.publish(coord_msg)
            publish = False
            rospy.loginfo("Received new goal.")
            

        elif cn.status >= 3:
            rospy.loginfo("Saved coordinates successfully.")
            rospy.loginfo(cn.text)
            cn.save_coord()
            cn.status = 0
            i += 1
            if i < len(goals):
                pos_x, pos_y, ori_z, ori_w = goals[i]
                publish = True
            
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: 
        pass