#include "nav_util.h"

int main(int argc, char **argv)
{
    CoordNav cn;
    bool publish = true;
    int i = 0;
    int start, current;
    
    cn.set_goals();

    ros::init(argc, argv, "coord_nav");
    MoveBaseClient ac("move_base", true);
    
    ros::NodeHandle n;
    ros::Publisher coord_pub = n.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 100);
    
    ros::Subscriber status_sub = n.subscribe("/move_base/result", 100, &CoordNav::status_cb, &cn);
    ros::Subscriber pos_sub = n.subscribe("/move_base/feedback", 100, &CoordNav::pos_cb, &cn);

    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    
    move_base_msgs::MoveBaseActionGoal coord_msg;

    vector<vector<string>> goals = cn.get_coord();

	string pos_x = goals[i][0];
    string pos_y = goals[i][1];
    string ori_z = goals[i][2];
    string ori_w = goals[i][3];

    ros::Rate rate(20);

    start = time(0);

    while(ros::ok())
    {
        current = time(0);
        cn.setElapsed(current - start);
        if(publish == true){
            coord_msg.goal.target_pose.header.frame_id = "odom";
            coord_msg.goal.target_pose.pose.position.x = std::stod(pos_x);
            coord_msg.goal.target_pose.pose.position.y = std::stod(pos_y);
            coord_msg.goal.target_pose.pose.orientation.z = std::stod(ori_z);
            coord_msg.goal.target_pose.pose.orientation.w = std::stod(ori_w);

            coord_pub.publish(coord_msg);
            publish = false;
            ROS_INFO("Received new goal.");
        }
        else if(cn.getStatus() >= 3)
        {
            ROS_INFO("Saved coordinates successfully.");
            cn.save_coord();
            cn.setStatus(0);
            i++;
            if(i < goals.size())
            {
                pos_x = goals[i][0];
                pos_y = goals[i][1];
                ori_z = goals[i][2];
                ori_w = goals[i][3];
                publish = true;
            }
        }
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}

