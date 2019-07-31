#include <waypoint_editor/waypoint_editor.hpp>

namespace waypoint_marker
{

const char *row[] = {"x","y","z","yaw","velocity","change_flag"};


WaypointEditor::WaypointEditor(ros::NodeHandle nh)
	: _nh(nh),
	  old_timestamp(ros::Time::now()),
	  _changed_waypoint(false)
{
	ros::NodeHandle pnh("~");
	pnh.param<std::string>("waypoint_filename", _waypoint_filename, "");
	pnh.param<std::string>("marker_topic_name", _marker_topic_name, "/waypoint_adjust");
	pnh.param<double>("save_timeout", _save_timeout, 30);
	pnh.param<double>("save_after_delay", _save_after_delay, 3);

	_waypoint_marker_pub = _nh.advertise<visualization_msgs::MarkerArray>("/waypoint_adjust", 10, true);
	
	boost::thread auto_save_thread(boost::bind(&WaypointEditor::auto_save_time_thread, this));
	boost::thread publish_waypoint_thread(boost::bind(&WaypointEditor::publish_waypoint_server_thread, this));
}

void WaypointEditor::process_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
	switch (feedback->event_type)
	{
		case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
		{
			int index = std::stoi(feedback->marker_name);
			waypoint_list[index].position.x = feedback->pose.position.x;
			waypoint_list[index].position.y = feedback->pose.position.y;
			waypoint_list[index].position.z = feedback->pose.position.z;
			waypoint_list[index].orientation.w = feedback->pose.orientation.w;
			waypoint_list[index].orientation.x = feedback->pose.orientation.x;
			waypoint_list[index].orientation.y = feedback->pose.orientation.y;
			waypoint_list[index].orientation.z = feedback->pose.orientation.z;
			display_waypoint(waypoint_list);
			break;
		}
		case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
			old_timestamp = ros::Time::now();
			if(!_changed_waypoint) _changed_waypoint = true;
			break;
	}
}

std::vector<geometry_msgs::Pose> WaypointEditor::parse_waypoint()
{
	std::vector<geometry_msgs::Pose> waypoint_list;
	std::ifstream ifs(_waypoint_filename.c_str());
	std::string line;

	std::string dummy_line;
	std::getline(ifs, dummy_line);
	while(std::getline(ifs, line))
	{
		std::string column_data;
		std::istringstream stream(line);
		std::size_t index=0;
		std::map<std::string, double> column;
		geometry_msgs::Pose pose;
		while(std::getline(stream, column_data, ','))
		{
			column[row[index]] = std::stod(column_data);
			index++;
		}
		pose.position.x = column["x"]; 
		pose.position.y = column["y"]; 
		pose.position.z = column["z"];
		
		tf2::Quaternion quat;
		quat.setRPY(0.0,0.0,column["yaw"]);
		pose.orientation.w = quat.w();
		pose.orientation.x = quat.x();
		pose.orientation.y = quat.y();
		pose.orientation.z = quat.z();

		waypoint_list.push_back(pose);
	}
	return waypoint_list;
}

void WaypointEditor::save_waypoint(ros::Time timestamp)
{
	ROS_INFO("saved waypoint.");

	std::string package_path = ros::package::getPath("waypoint_editor");
	std::string filename = "/config/saved_waypoints_update.csv";
	std::ofstream output_file;
	output_file.open(package_path+filename, std::ios::out);

	output_file << "x,y,z,yaw,velocity,change_flag" << std::endl;
	for(std::size_t index=0;index<waypoint_list.size();index++)
	{
		geometry_msgs::Pose pose = waypoint_list[index];
		output_file << pose.position.x << ",";
		output_file << pose.position.y << ",";
		output_file << pose.position.z << ",";
		tf2::Quaternion quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
		tf2::Matrix3x3 m(quat);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		output_file << yaw << ",";
		output_file << 0.0 << ",";
		output_file << 0 << std::endl;
	}
	output_file.close();

	old_timestamp = timestamp;
}

void WaypointEditor::publish_waypoint_server_thread()
{
	server.reset(new interactive_markers::InteractiveMarkerServer("interactive_tf"));
	waypoint_list = parse_waypoint();
	display_waypoint(waypoint_list);
	display_waypoint_handler(waypoint_list);
	server->applyChanges();
}

void WaypointEditor::auto_save_time_thread()
{
	ros::Rate rate(1);
	while(1)
	{
		rate.sleep();
		ros::Time now = ros::Time::now();
		timer = now-old_timestamp;
		if(_save_timeout <= timer.toSec() && !_changed_waypoint) save_waypoint(now);
		if(_save_after_delay <= timer.toSec() && _changed_waypoint)
		{
			_changed_waypoint = false;
			save_waypoint(now);
		}
	}
}

void WaypointEditor::display_waypoint(std::vector<geometry_msgs::Pose> waypoint_list)
{
	visualization_msgs::MarkerArray marker_list;
	visualization_msgs::Marker marker;

	int id=0;

	for(std::size_t index=0;index<waypoint_list.size();index++)
	{
		marker.id = id;
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time();
		marker.frame_locked = true;

		marker.scale.x = 0.3;
		marker.scale.y = 0.05;
		marker.scale.z = 0.05;
		marker.color.a = 1.0;
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 1.0;
		marker.ns = "waypoint_allow";
		marker.type = visualization_msgs::Marker::ARROW;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose = waypoint_list[id];
		marker_list.markers.push_back(marker);
		id++;
	}

	_waypoint_marker_pub.publish(marker_list);
}

void WaypointEditor::display_waypoint_handler(std::vector<geometry_msgs::Pose> waypoint_list)
{
	for(std::size_t index=0;index<waypoint_list.size();index++)
	{
		visualization_msgs::InteractiveMarker marker;
		marker.header.frame_id = "map";
		marker.pose.position = waypoint_list[index].position;

		geometry_msgs::Pose pose = waypoint_list[index];
		marker.pose.orientation.w = pose.orientation.w;
		marker.pose.orientation.x = pose.orientation.x;
		marker.pose.orientation.y = pose.orientation.y;
		marker.pose.orientation.z = pose.orientation.z;

		marker.scale = 0.5;
		marker.name = std::to_string(index);
		marker.description = "handle_No."+std::to_string(index);

		tf2::Quaternion quat;
		visualization_msgs::InteractiveMarkerControl control;

		control.name = "move_y";
		quat.setRPY(0.0,0,M_PI/2);
		control.orientation.w = quat.w();
		control.orientation.x = quat.x();
		control.orientation.y = quat.y();
		control.orientation.z = quat.z();
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
		marker.controls.push_back(control);
		
		control.name = "rotate_z";
		quat.setRPY(0.0,M_PI/2,0.0);
		control.orientation.w = quat.w();
		control.orientation.x = quat.x();
		control.orientation.y = quat.y();
		control.orientation.z = quat.z();
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
		marker.controls.push_back(control);

		server->insert(marker);
		server->setCallback(marker.name, boost::bind(&WaypointEditor::process_feedback, this, _1));
	}
}

}
