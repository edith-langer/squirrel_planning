#include <vector>

#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <squirrel_planning_execution/ViewConeGenerator.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv) {

	if (argc < 7) {
		std::cout << "Usage: ./view_cone_test {occupancy_threshold} {nr_view_cones} {view_distance} {fov} {sample_size} {safe_distance}." << std::endl;
		exit(0);
	}
	
	std::vector<tf::Vector3> bounding_box;
	//bounding_box.push_back(tf::Vector3(-2.0, 0.9, 0.0));
	//bounding_box.push_back(tf::Vector3(-2.0, -9.54, 0.0));
	//bounding_box.push_back(tf::Vector3(6.05, -9.54, 0.0));
	//bounding_box.push_back(tf::Vector3(6.05, 0.9, 0.0));
	
	bounding_box.push_back(tf::Vector3(2.7, -6.2, 0.0));
	bounding_box.push_back(tf::Vector3(-2.0, -3.5, 0.0));
	bounding_box.push_back(tf::Vector3(1.0, 1.4, 0.0));
	bounding_box.push_back(tf::Vector3(6.0, -0.7, 0.0));
	
	int occupancy_threshold = ::atoi(argv[1]); // 2
	int nr_view_cones = ::atoi(argv[2]);       // 20
	float view_distance = ::atof(argv[3]);     //2.0f;
	float fov = ::atof(argv[4]);               //70.0f * M_PI / 180.0f;
	unsigned int sample_size = ::atoi(argv[5]); //1000;
	float safe_distance = ::atof(argv[6]);      //0.5f;
	
	ros::init(argc, argv, "rosplan_interface_view_cone_test_suite_ViewConeCaller");
	ros::NodeHandle nh;

	// create PDDL action subscriber
	KCL_rosplan::ViewConeGenerator vg(nh, "/map");
	ROS_INFO("Waiting for the occupancy grid to be published...");
	while (!vg.hasReceivedOccupancyGrid() && ros::ok()) {
		ros::spinOnce();
	}
	
	ROS_INFO("Occupancy grid published!");
	std::vector<geometry_msgs::Pose> poses;
	/*
	int occupancy_threshold = 2;
	int nr_view_cones = 20;
	float view_distance = 2.0f;
	float fov = 70.0f * M_PI / 180.0f;
	unsigned int sample_size = 1000;
	float safe_distance = 0.5f;
	*/
	vg.createViewCones(poses, bounding_box, nr_view_cones, occupancy_threshold, fov, view_distance, sample_size, safe_distance);

	ROS_INFO("Got the view cones, there are %d!", poses.size());

	// Create a broadcast of TF.
	tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(0, 0, 0));
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	transform.setRotation(q);
	
	while (ros::ok()) {
		ros::spinOnce();
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "map"));
		ros::Duration(1.0).sleep();
	}
	
	return 0;
}
