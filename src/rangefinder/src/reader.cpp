#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

void distance_reading_callback(const std_msgs::Float64::ConstPtr& val){
	ROS_INFO("Distance Value: %f",val->data);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "reader");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("distance",1000,distance_reading_callback);
	ros::spin();

	return 0;
}
