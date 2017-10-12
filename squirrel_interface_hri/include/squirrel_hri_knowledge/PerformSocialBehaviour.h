#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "mongodb_store/message_store.h"
#include "squirrel_hri_msgs/FollowChildAction.h"
#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>

#ifndef SQUIRREL_INTERFACE_HRI_PERFORM_SOCIAL_BEHAVIOUR_H
#define SQUIRREL_INTERFACE_HRI_PERFORM_SOCIAL_BEHAVIOUR_H

/**
 * This file defines the PerformSocialBehaviour class.
 * It executes the 'perform_social_behaviour' planning action.
 *
 * Parameters:
 * - c (if it is CLOSEST_CHILD it is the closest child. Otherwise it 
 * should follow a specific child (as stored in the knowledge base).
 */
namespace KCL_rosplan {

	class PerformSocialBehaviour
	{

	private:
		
		// Scene database
		mongodb_store::MessageStoreProxy message_store;

		// Knowledge base
		ros::ServiceClient knowledgeInterface;

		// action topics
		ros::Publisher action_feedback_pub;
		ros::Publisher head_tilt_pub;
		ros::Publisher head_nod_pub;
		
		ros::Publisher lights_pub_;
		ros::Publisher lights_complex_pub_; // 3 * 42
		ros::Publisher expression_pub_;

		ros::Subscriber arousal_sub_;
		
		float arousal_threshold;
		float current_arousal;
		
		// Movebase action lib.
		actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client;
		void getArousal(const std_msgs::Float32::ConstPtr& msg);

	public:

		/* constructor */
		PerformSocialBehaviour(ros::NodeHandle &nh, const std::string& move_base_action_name);

		void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif

