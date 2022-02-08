#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>

#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class CoordNav
{
  public:
    int status = 0;
    vector<string> final_coords;
    string text;
    string fname = "src/autonomous_nav/src/resources/coordinates.csv";

    vector<vector<string>> get_coord();
    void set_goals();
    void save_coord();
    void status_cb(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);
    void pos_cb(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg);
};

vector<vector<string>> CoordNav::get_coord()
{
    vector<vector<string>> content;
	vector<string> row;
	string line, word;

	fstream file(fname, ios::in);
	if(file.is_open())
	{
		while(getline(file, line))
		{
			row.clear();

			stringstream str(line);

			while(getline(str, word, ','))
				row.push_back(word);
			content.push_back(row);
		}
        file.close();
	}
	else
		ROS_INFO("Could not open the file\n");
        
    return content;
}

void CoordNav::set_goals()
{
    ofstream file;
    file.open("coordinates.csv");
    file << "7.0,3.0,0.75,0.66" << endl;
    file << "5.0,4.0,0.75,0.66" << endl;
    file << "1.0,1.0,0.75,0.66" << endl;
    file.close();
}

void CoordNav::status_cb(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
{
    if(msg->status.status >= 3)
    {
        status = msg->status.status;
        text = msg->status.text;
    }
}

void CoordNav::pos_cb(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg)
{
    final_coords = {
        to_string(msg->feedback.base_position.pose.position.x),
        to_string(msg->feedback.base_position.pose.position.y),
        to_string(msg->feedback.base_position.pose.orientation.z),
        to_string(msg->feedback.base_position.pose.orientation.w)
    };
}

void CoordNav::save_coord()
{
    int i;
    ofstream file;
    file.open(fname, std::ios_base::app);
    
    final_coords.push_back(text);

    for(i=0; i<final_coords.size()-1; i++)
	{
		file << final_coords[i] << ',';
	}
    file << final_coords[i] << endl;
    file.close();
}

int main(int argc, char **argv)
{
    CoordNav cn;
    bool publish = true;
    int i = 0;
    
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

    while(ros::ok())
    {
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
        else if(cn.status >= 3)
        {
            ROS_INFO("Saved coordinates successfully.");
            cn.save_coord();
            cn.status = 0;
            i++;
            if(i < goals.size())
            {
                pos_x = goals[i][0];
                pos_y = goals[i][1];
                ori_z = goals[i][2];
                ori_w = goals[i][3];
                publish = true;
            }
            else
            {
                ros::shutdown();
            }
        }
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}

