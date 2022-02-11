#ifndef COORD_NAV_H
#define COORD_NAV_H

#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <time.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class CoordNav
{
    private:
        int status = 0;
        vector<string> final_coords;
        string text;
        int elapsed;

    public:
        void setStatus(int s) { status = s; }
        void setElapsed(int e) { elapsed = e; }
        
        int getStatus() { return status; }
        int getElapsed() { return elapsed; }

        const string fname = (ros::package::getPath("autonomous_nav") + "/resources/coordinates.csv").c_str();
        vector<vector<string>> get_coord();
        void set_goals();
        void save_coord();
        void status_cb(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);
        void pos_cb(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg);
};

#endif