#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <time.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "husky_controller");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("husky_velocity_controller/cmd_vel", 100);
  ros::Rate rate(10);

  int start, current, elapsed;
  start = time(0);

  while(ros::ok()) {
    geometry_msgs::Twist msg;
    current = time(0);
    elapsed = current - start;
    
    if(elapsed <= 10){
      msg.linear.x = 0.0;
      msg.angular.z = -2.0;
      pub.publish(msg);
    }
    else if(elapsed > 10 && elapsed <= 20){
      msg.linear.x = 0.0;
      msg.angular.z = 2.0;
      pub.publish(msg);
    }
    else{
      msg.linear.x = 0.0;
      msg.angular.z = 0.0;
      pub.publish(msg);
    }
    
    rate.sleep();
  }
}