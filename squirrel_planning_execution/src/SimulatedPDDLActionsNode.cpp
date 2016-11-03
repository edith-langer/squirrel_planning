#include <ros/ros.h>

#include "pddl_actions/GotoPDDLAction.h"
#include "pddl_actions/ExploreWaypointPDDLAction.h"
#include "pddl_actions/ClearObjectPDDLAction.h"
#include "pddl_actions/ClassifyObjectPDDLAction.h"
#include "pddl_actions/PutObjectInBoxPDDLAction.h"
#include "pddl_actions/TidyObjectPDDLAction.h"
#include "pddl_actions/PickupPDDLAction.h"
#include "pddl_actions/PushObjectPDDLAction.h"
#include "pddl_actions/DropObjectPDDLAction.h"

int main(int argc, char **argv) {

	ros::init(argc, argv, "rosplan_interface_SimluatedPDDLActionsNode");
	ros::NodeHandle nh("~");

	bool goto_waypoint,
		explore_waypoint,
		clear_object,
		classify_object,
		put_object_in_box,
		pickup_object,
		drop_object;

	nh.getParam("simulate_goto_waypoint", goto_waypoint);
	nh.getParam("simulate_explore_waypoint", explore_waypoint);
	nh.getParam("simulate_clear_object", clear_object);
	nh.getParam("simulate_classify_object", classify_object);
	nh.getParam("simulate_put_object_in_box", put_object_in_box);
	nh.getParam("simulate_pickup_object", pickup_object);
	nh.getParam("simulate_drop_object", drop_object);

	KCL_rosplan::GotoPDDLAction* goto_action;
	KCL_rosplan::ExploreWaypointPDDLAction* explore_waypoint_action;
	KCL_rosplan::ClearObjectPDDLAction* clear_object_action;
	KCL_rosplan::ClassifyObjectPDDLAction* classify_object_action;
	KCL_rosplan::PutObjectInBoxPDDLAction* put_object_in_box_action;
	KCL_rosplan::PickupPDDLAction* pickup_action;
	KCL_rosplan::DropObjectPDDLAction* drop_object_action;

	// Setup all the simulated actions.
	if(goto_waypoint) {
		ROS_INFO("KCL: (SimulatedPDDLActionsNode) Simulating: goto_waypoint");
		goto_action = new KCL_rosplan::GotoPDDLAction(nh);
	}

	if(explore_waypoint) {
		ROS_INFO("KCL: (SimulatedPDDLActionsNode) Simulating: explore_waypoint");
		explore_waypoint_action = new KCL_rosplan::ExploreWaypointPDDLAction(nh);
	}
	if(clear_object) {
		ROS_INFO("KCL: (SimulatedPDDLActionsNode) Simulating: clear_object");
		clear_object_action = new KCL_rosplan::ClearObjectPDDLAction(nh);
	}
	if(classify_object) {
		ROS_INFO("KCL: (SimulatedPDDLActionsNode) Simulating: classify_object");
		classify_object_action = new KCL_rosplan::ClassifyObjectPDDLAction(nh, 0.5f);
	}
	if(put_object_in_box) {
		ROS_INFO("KCL: (SimulatedPDDLActionsNode) Simulating: put_object_in_box");
		put_object_in_box_action = new KCL_rosplan::PutObjectInBoxPDDLAction(nh);
	}
	if(pickup_object) {
		ROS_INFO("KCL: (SimulatedPDDLActionsNode) Simulating: pickup_object");
		pickup_action = new KCL_rosplan::PickupPDDLAction(nh);
	}	
	if(drop_object) {
		ROS_INFO("KCL: (SimulatedPDDLActionsNode) Simulating: drop_object");
		drop_object_action = new KCL_rosplan::DropObjectPDDLAction(nh);
	}

	KCL_rosplan::TidyObjectPDDLAction tidy_object_action(nh);
	// KCL_rosplan::PushObjectPDDLAction push_object_action;

	ROS_INFO("KCL: (SimulatedPDDLActionsNode) All simulated actions are ready to receive.");
	
	while(ros::ok() && ros::master::check()){ros::spinOnce();}
	return 0;
}
