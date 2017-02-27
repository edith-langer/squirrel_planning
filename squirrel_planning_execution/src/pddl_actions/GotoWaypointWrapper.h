#ifndef SQUIRRELPLANNINGEXECUTION_PDDLACTIONS_GOTOWAYPOINTWRAPPER_H
#define SQUIRRELPLANNINGEXECUTION_PDDLACTIONS_GOTOWAYPOINTWRAPPER_H

#include <ros/ros.h>
#include <rosplan_dispatch_msgs/ActionDispatch.h>
#include <squirrel_object_perception_msgs/CheckWaypoint.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <mongodb_store/message_store.h>
#include <actionlib/client/simple_action_client.h>

namespace KCL_rosplan
{

/**
 * A wrapper for the goto_waypoint action. It first check whether the waypoint should be explored
 * or whether we can leave it.
 */
class GotoWaypointWrapper
{
public:
	
	/**
	 * Constructor.
	 * @param node_handle An existing and initialised ros node handle.
	 * @param classification_probability A number between 0 and 1 that determines how likely it is to classify an object.
	 * @param fov The field of view of the view cones we generate.
	 * @param view_distance The viewing distance of  the view cones we generate.
	 */
	GotoWaypointWrapper(ros::NodeHandle& nh, const std::string& actionserver, float fov, float view_distance);
	
	/**
	 * Destructor
	 */
	~GotoWaypointWrapper();
	
	/**
	 * Called when this action needs to be executed.
	 * @param msg The dispatch message sent by ROSPlan.
	 * @return True if the action was successfull, false otherwise.
	 */
	void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	
	/**
	 * Set whether we should check the view cones or not.
	 */
	static void enableCheck(bool check_view_cones_);
	
private:
	mongodb_store::MessageStoreProxy message_store;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client;
	ros::ServiceClient clear_costmaps_client;
	ros::ServiceClient check_waypoint_;
	ros::ServiceClient update_knowledge_client_;
	ros::Subscriber dispatch_sub_;               // Subscriber to the dispatch topic of ROSPlan.
	ros::Publisher action_feedback_pub_;
	float fov_, view_distance_;
	ros::Subscriber ds_;
	
	static bool check_view_cones_;
};

};

#endif
