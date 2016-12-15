#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <actionlib/client/simple_action_client.h>
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "squirrel_planning_knowledge_msgs/AddObjectService.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "mongodb_store/message_store.h"

#ifndef KCL_perception
#define KCL_perception

/**
 * This file defines the RPPerceptionAction class.
 * RPPerceptionAction is used to connect ROSPlan to the object perception in SQUIRREL
 */
namespace KCL_rosplan {

	class RPPerceptionAction
	{

	private:

		mongodb_store::MessageStoreProxy message_store;

		actionlib::SimpleActionClient<squirrel_object_perception_msgs::LookForObjectsAction> examine_action_client;
		ros::ServiceClient find_dynamic_objects_client;
		ros::ServiceClient add_object_client;
		ros::ServiceClient update_knowledge_client;
		ros::ServiceClient get_instance_client;

		ros::Publisher action_feedback_pub;

		std::map<std::string,std::string> db_name_map;
		
		bool use_dynamic_object_finding;

		void publishFeedback(int action_id, std::string feedback);

		/* actions */
		void exploreActionDynamic(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
		void exploreActionStatic(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
		void examineAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);

		/* objects to database */
		void addObject(squirrel_object_perception_msgs::SceneObject &object);
		void updateObject(squirrel_object_perception_msgs::SceneObject &object, std::string newWaypoint);
		void removeObject(squirrel_object_perception_msgs::SceneObject &object);
		void updateType(squirrel_object_perception_msgs::SceneObject &object);

	public:

		/* constructor */
		RPPerceptionAction(ros::NodeHandle &nh, std::string &actionserver);

		/* listen to and process action_dispatch topic */
		void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif
