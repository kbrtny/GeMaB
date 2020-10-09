#include <ros/ros.h>
#include "gemab_msgs/location.h"
//#include "nav_msgs/Odometry.h"
//#include "geometry_msgs/PoseStamped.h"
#include "room_manager.h"
#include "std_msgs/String.h"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "room_detector");

	ros::NodeHandle nh;

	RoomManager my_home;

	std::vector<Point>  roomacoords {
		{0, 0, 0},
		{10, 0, 0},
		{10, 5, 0},
		{0, 5, 0}
	};
	my_home.CreateRoom("rooma", roomacoords);
	//load in room config file, make it into something useful [TODO: move this into a class]

	//publish the room using the gemab_msg
	ros::Publisher room_pub = nh.advertise<gemab_msgs::location>("current_room", 1000);

	//set up subscribers for odom
	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		gemab_msgs::location current_room;

		//pull odom data into local variables


		//update msg to publish
		current_room.header.stamp = ros::Time::now();
		current_room.header.frame_id = "base_footprint";
		current_room.name = my_home.GetRoomNameFromId(my_home.GetCurrentRoomId(1 , 2));

		//ROS_INFO("%s", current_room.data.c_str());

		room_pub.publish(current_room);

		ros::spinOnce();

		loop_rate.sleep();

	}

	return 0;
}