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
#include <geometry_msgs/PointStamped.h>
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
		
		ros::Publisher lights_pub_;
		ros::Publisher lights_complex_pub_; // 3 * 42
		ros::Publisher expression_pub_;

		/**
		 * Control the tilt angle of the kinect and head.
		 */
		ros::ServiceClient view_controller_client_; // Controls where the robot looks at.
		ros::Publisher neck_tilt_pub_; // Tilt the kinect.
		float head_up_angle_;          // Angle at which the kinect looks down.
		float head_down_angle_;        // Angle at which the kineect looks up.

		/**
		 * The the emotional state of the children.
		 */
		ros::Subscriber arousal_sub_;

		/**
		 * Get the location where a child is pointing.
		 */
		ros::Subscriber point_pose_sub_;
		void getPointingPose(const geometry_msgs::PointStamped::ConstPtr& msg);
		geometry_msgs::PointStamped child_pointing_location_;
		bool has_received_pointing_location_;
		
		float arousal_threshold;
		float current_arousal;
		
		// Movebase action lib.
		actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client;
		void getArousal(const std_msgs::Float32::ConstPtr& msg);



		// Shuff an object forward a bit.
		void PushingSuggestObject();

		// Push an object towards a child.
		void performPushToShare();

		// Push an object away from a child.
		void PushToDeny();

		/**
		 * Display light show.
		 * @param delay The delay before the lights move on.
		 * @param duration How long the lights should be displayed.
		 */
		void displayRainbow(float delay, float duration);

		/**
		 * Set the colours to a certain value.
		 */
		void displayLights(unsigned int r, unsigned int g, unsigned int b);

	public:

		/* constructor */
		PerformSocialBehaviour(ros::NodeHandle &nh, const std::string& move_base_action_name);

		void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);

		// Have the robot look at where a child is pointing or looking.
		void performSocialGaze();

		// Look at an object that the robot wants.
		// Look twice, very fast.
		void performDeicticGaze(const geometry_msgs::PoseStamped& p);
	};
}
#endif

