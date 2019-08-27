#ifndef _WAYOINT_EDITOR_H_
#define _WAYOINT_EDITOR_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

namespace waypoint_marker
{

class WaypointAdjuster
{
	public:
		WaypointAdjuster(ros::NodeHandle nh);
		~WaypointAdjuster(){}
		void run();
	private:
		std::vector<geometry_msgs::Pose> parse_waypoint();
		void display_waypoint(std::vector<geometry_msgs::Pose> waypoint_list);
		void display_waypoint_handler(std::vector<geometry_msgs::Pose> waypoint_list);
		void process_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
		void save_waypoint(ros::Time timestamp);
		void auto_save_time_thread();
		void publish_waypoint_server_thread();

		ros::Time old_timestamp;
		ros::Duration timer;
		ros::NodeHandle _nh;
		ros::Publisher _waypoint_marker_pub;

		boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

		std::vector<geometry_msgs::Pose> waypoint_list;
		std::string _waypoint_filename;
		std::string _marker_topic_name;
		double _save_timeout;
		double _save_after_delay;
		bool _changed_waypoint;
};
}

#endif
