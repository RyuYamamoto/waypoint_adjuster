#include <ros/ros.h>

#include <waypoint_adjuster/waypoint_adjuster.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "waypoint_adjuster_node");
	ros::NodeHandle nh;
	waypoint_marker::WaypointAdjuster wpa(nh);
	ros::spin();
	return 0;
}
