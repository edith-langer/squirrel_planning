#include <algorithm>

#include "squirrel_interface_emote/RPEmoteAction.h"

#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_dispatch_msgs/ActionFeedback.h>

#include <std_msgs/String.h>

/* The implementation of RPEmoteAction.h */
namespace KCL_rosplan {

	/* constructor */
	RPEmoteAction::RPEmoteAction(ros::NodeHandle &nh)
		: node_handle_(&nh)
	{
		// knowledge interface
		update_knowledge_client_ = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		get_instance_client_ = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
		get_attribute_client_ = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
		action_feedback_pub_ = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
		
		dispatch_sub_ = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPEmoteAction::dispatchCallback, this);
		
		sound_pub_ = nh.advertise<std_msgs::String>("/expression", 1, true);
		wiggle_pub_ = nh.advertise<std_msgs::String>("/motion_expression", 1, true);
	}
	
	void RPEmoteAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		std::string normalised_action_name = msg->name;
		std::transform(normalised_action_name.begin(), normalised_action_name.end(), normalised_action_name.begin(), tolower);
		
		// Check if this action is to be handled by this class.
		if (normalised_action_name != "emote" || msg->parameters.size() != 3)
		{
			return;
		}
		
		ROS_INFO("KCL: (RPEmoteAction) Process the action: %s", normalised_action_name.c_str());

		
		// Report this action is enabled and completed successfully.
		rosplan_dispatch_msgs::ActionFeedback fb;
		fb.action_id = msg->action_id;
		fb.status = "action enabled";
		action_feedback_pub_.publish(fb);
		
		// Update the domain.
		const std::string& robot = msg->parameters[0].value;
		const std::string& sound = msg->parameters[1].value;
		const std::string& wiggle = msg->parameters[2].value;
		
		ROS_INFO("KCL: (RPEmoteAction) Process the action: %s, %s: emit sound %s and wiggle like %s", normalised_action_name.c_str(), robot.c_str(), sound.c_str(), wiggle.c_str());
		
		// Start the wiggle.
	
		std_msgs::String wiggle_command;
		wiggle_command.data = wiggle.substr(std::string("wiggle_").size());
		wiggle_pub_.publish(wiggle_command);
		
		// Start emiting the sounds.
		std_msgs::String sound_command;
		sound_command.data = sound.substr(std::string("sound_").size());
		std::transform(sound_command.data.begin(), sound_command.data.end(), sound_command.data.begin(), ::toupper);
		sound_pub_.publish(sound_command);
	
		for (unsigned int i = 0; i < 3; ++i)
		{
			ros::spinOnce();
			
			ros::Duration(1).sleep();
			
			ROS_INFO("KCL: (RPEmoteAction) Waiting %d longer before moving on...", i); 
		}
		
		fb.action_id = msg->action_id;
		fb.status = "action achieved";
		action_feedback_pub_.publish(fb);
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_emote");
		ros::NodeHandle nh;

		// create PDDL action subscriber
		KCL_rosplan::RPEmoteAction rpea(nh);
	
		ROS_INFO("KCL: (RPEmoteAction) Ready to receive");

		ros::spin();
		return 0;
	}
