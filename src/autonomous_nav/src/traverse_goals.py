#!/usr/bin/env python

import rospy
import csv
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseActionResult, MoveBaseActionFeedback
import actionlib
import os, rospkg

def main():
    cn = CoordNav()
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

    while not rospy.is_shutdown():
        if publish:
            goal.header.frame_id = 'odom'
            goal.pose.position.x = pos_x
            goal.pose.position.y = pos_y
            goal.pose.orientation.z = ori_z
            goal.pose.orientation.w = ori_w
            coord_pub.publish(coord_msg)
            publish = False
            rospy.loginfo("\nReceived new goal.")

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

class CoordNav:
    def __init__(self):
        self.rospack = rospkg.RosPack()
        self.status = 0
        self.final_coords = []
        self.text = None
        self.fname = "coordinates.csv"

    def set_goals(self):
        with open(os.path.join(self.rospack.get_path("autonomous_nav"), "resources", self.fname), 'w') as f:
            writer = csv.writer(f)
            writer.writerows([
                [7.0, 8.0, 0.75, 0.66],
                [5.0, 4.0, 0.75, 0.66],
                [1.0, 1.0, 0.75, 0.66]
            ])

    def status_cb(self, msg):
        if msg.status.status >= 3:
            self.status = msg.status.status  
            self.text = msg.status.text

    def pos_cb(self, msg):
        self.final_coords = list(map(str, [
            msg.feedback.base_position.pose.position.x,
            msg.feedback.base_position.pose.position.y,
            msg.feedback.base_position.pose.orientation.z,
            msg.feedback.base_position.pose.orientation.w
        ]))

    def get_coord(self):
        goals = []
        with open(os.path.join(self.rospack.get_path("autonomous_nav"), "resources", self.fname), 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                goals.append(list(map(float, row)))

        return goals

    def save_coord(self):
        with open(os.path.join(self.rospack.get_path("autonomous_nav"), "resources", self.fname), 'a+') as f:
            writer = csv.writer(f)
            self.final_coords.append(self.text)
            writer.writerow(self.final_coords)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: 
        pass