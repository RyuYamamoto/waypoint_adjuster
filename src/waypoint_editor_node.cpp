#include <ros/ros.h>

#include <waypoint_editor/waypoint_editor.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "waypoint_editor_node");
	ros::NodeHandle nh;
	waypoint_marker::WaypointEditor wpa(nh);
	ros::spin();
	return 0;
}
