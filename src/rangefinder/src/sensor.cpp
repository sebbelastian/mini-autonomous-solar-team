#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

int main( int argc, char **argv){

	ros::init(argc, argv, "sensor");
	ros::NodeHandle n;
	ros::Publisher distance_pub = n.advertise<std_msgs::Float64>("distance",1000);
	//ros::Publisher distance_pub = n.advertise<std_msgs::String>("distance",100);
	ros::Rate loop_rate(5); // 80Hz

	int count = 0;

	while(ros::ok()){
		std_msgs::Float64 val; //64 bit float
		val.data = 2 + ( std::rand() % ( 60 - 2 + 1 ) ); // generate number 
		ROS_INFO("sensor value: %f", val.data,count); // printf
		distance_pub.publish(val); // publish on topic distance_pub

		ros::spinOnce();
		loop_rate.sleep(); // ensure 80Hz
		++count; 	
	}
return 0;
}
