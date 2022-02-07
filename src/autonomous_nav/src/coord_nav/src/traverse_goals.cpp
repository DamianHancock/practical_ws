#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

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
    string text = NULL;

    vector<vector<string>> get_coord();
    void set_goals();
    void status_cb();
    void pos_cb();
    void save_coord();
};

vector<vector<string>> CoordNav::get_coord()
{
    string fname = "coordinates.csv";

    vector<vector<string>> content;
	vector<string> row;
	string line, word;

	fstream file (fname, ios::in);
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
	}
	else
		ROS_INFO("Could not open the file\n");
        
    return content;
}

void CoordNav::set_goals()
{
    std::ofstream file;
    file.open("coordinates.csv");
    file << "7.0,8.0,0.75,0.66,\n";
    file << "5.0,4.0,0.8,0.5,\n";
    file << "4.0,2.0,0.8,0.5";
    file.close();
}

// TO DO: fix syntax
void CoordNav::status_cb(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
{
    // ROS_INFO("I heard: [%s]", msg->data.c_str());

}

// TO DO: fix syntax
void CoordNav::pos_cb(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg)
{
    // ROS_INFO("I heard: [%s]", msg->data.c_str());

}

void CoordNav::save_coord()
{
    std::ofstream file("coordinates.csv", std::ios::app);
    // TO DO: save coordinates here
}

int main(int argc, char **argv)
{
    CoordNav cn;
    bool publish = true;
    int i = 0;

    ros::init(argc, argv, "coord_nav");
    MoveBaseClient ac("move_base", true);

    ros::NodeHandle n;
    ros::Publisher coord_pub = n.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 100);

    // TO DO: figure out syntax later
    ros::Subscriber status_sub = n.subscribe<move_base_msgs::MoveBaseActionResult>("move_base/result", 100, cn.status_cb());
    ros::Subscriber status_sub = n.subscribe<move_base_msgs::MoveBaseActionFeedback>("move_base/feedback", 100, cn.pos_cb());

    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseActionGoal coord_msg;
    geometry_msgs::PoseStamped goal = coord_msg.goal.target_pose;

    vector<vector<string>> goals = cn.get_coord();
    vector<string> pos_x, pos_y, ori_z, ori_w = goals[i];

    ros::Rate rate(20);

    while(ros::ok())
    {
        if(publish == true){
            goal.header.frame_id = "odom";
            // TO DO: fix syntax
            goal.pose.position.x = pos_x;
            goal.pose.position.y = pos_y;
            goal.pose.orientation.z = ori_z;
            goal.pose.orientation.w = ori_w;
            coord_pub.publish(coord_msg);
            publish = false;
        }
        else if(cn.status >= 3)
        {
            cn.save_coord();
            cn.status = 0;
            i++;
            if(i < goals.size())
            {
                pos_x, pos_y, ori_z, ori_w = goals[i];
                publish = true;
            }
        }
        rate.sleep();
    }
    

    
	// for(int i=0;i<content.size();i++)
	// {
	// 	for(int j=0;j<content[i].size();j++)
	// 	{
	// 		cout<<content[i][j]<<" ";
	// 	}
	// 	cout<<"\n";
	// }

    return 0;
}

