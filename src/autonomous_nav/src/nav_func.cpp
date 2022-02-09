#include "coord_nav.h"

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
    file.open(fname);
    file << "7.0,3.0,0.75,0.66" << endl;
    file << "5.0,4.0,0.75,0.66" << endl;
    file << "1.0,1.0,0.75,0.66" << endl;
    file.close();
}

void CoordNav::status_cb(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
{
    if(msg->status.status >= 3)
    {
        setStatus(msg->status.status);
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
    final_coords.push_back(to_string(getElapsed()));

    for(i=0; i<final_coords.size()-1; i++)
	{
		file << final_coords[i] << ',';
	}
    file << final_coords[i] << endl;
    file.close();
}