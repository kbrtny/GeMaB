#include <ros/ros.h>
#include "gemab_msgs/location.h"
#include "nav_msgs/Odometry.h"
//#include "geometry_msgs/PoseStamped.h"
#include "room_manager.h"
#include "std_msgs/String.h"


RoomManager my_home;
gemab_msgs::location current_room;
std::string currentRoomName;
int currentRoomId;

ros::Publisher room_pub;

ros::Subscriber odomsub;

void odom_Cb(const nav_msgs::Odometry& odom_msg){
    float x = odom_msg.pose.pose.position.x;
    float y = odom_msg.pose.pose.position.y;
    //update msg to publish
	current_room.header.stamp = ros::Time::now();
	current_room.header.frame_id = "base_footprint";

	currentRoomId = my_home.GetCurrentRoomId(x , y);
	currentRoomName = my_home.GetRoomNameFromId(currentRoomId);
	current_room.name = currentRoomName;

	//ROS_INFO("%s", current_room.data.c_str());

	room_pub.publish(current_room);
}



int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "room_detector");
	ros::NodeHandle nh;

	room_pub = nh.advertise<gemab_msgs::location>("current_room", 10000);
	
	//load in room config file, make it into something useful [TODO: move this into a class]
	my_home.ImportFromJSON("house_semantic_map.json");
	
	odomsub = nh.subscribe("odom", 10, odom_Cb);
	//set up subscribers for odom
	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		//pull odom data into local variables
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}