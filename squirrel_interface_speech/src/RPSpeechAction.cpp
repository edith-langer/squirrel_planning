#include "squirrel_interface_speech/RPSpeechAction.h"

#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>

/* The implementation of RPSpeechAction.h */
namespace KCL_rosplan {

	/* constructor */
	RPSpeechAction::RPSpeechAction(ros::NodeHandle &nh)
		: node_handle_(&nh)
	{
		// knowledge interface
		update_knowledge_client_ = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		get_instance_client_ = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
		get_attribute_client_ = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
		
		command_stream_ = nh.subscribe<squirrel_speech_msgs::RecognizedCommand>("/squirrel_speech_rec/squirrel_speech_recognized_commands", 1, &RPSpeechAction::processSpeechCommand, this);
	}
	
	void RPSpeechAction::updateKnowledgeBase(const squirrel_speech_msgs::RecognizedCommand& command, bool add)
	{
		// Remove the old knowledge.
		rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
		rosplan_knowledge_msgs::KnowledgeItem knowledge;
		knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge.attribute_name = "has_commanded";
		knowledge.is_negative = add;
		
		diagnostic_msgs::KeyValue kv;
		kv.key = "k";
		std::stringstream ss;
		ss << "kid_" << command.speaker_ID;
		kv.value = ss.str();
		knowledge.values.push_back(kv);
		
		kv.key = "c";
		kv.value = command.int_command;
		knowledge.values.push_back(kv);
		
		knowledge_update_service.request.knowledge = knowledge;
		if (!update_knowledge_client_.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (RPSpeechAction) Could not update (has_commanded %s %s) predicate from the knowledge base.", ss.str().c_str(), command.int_command.c_str());
			exit(-1);
		}
		
		// Add the new knowledge
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		knowledge.is_negative = !add;
		knowledge_update_service.request.knowledge = knowledge;
		if (!update_knowledge_client_.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (RPSpeechAction) Could not update (has_commanded %s %s) predicate from the knowledge base.", ss.str().c_str(), command.int_command.c_str());
			exit(-1);
		}
		
		if (!add)
		{
			ROS_ERROR("KCL: (RPSpeechAction) (has_commanded %s %s) is set to false in the knowledge base.", ss.str().c_str(), command.int_command.c_str());
		}
		else
		{
			ROS_ERROR("KCL: (RPSpeechAction) (has_commanded %s %s) is set to true in the knowledge base.", ss.str().c_str(), command.int_command.c_str());
		}
	}

	/* action dispatch callback */
	void RPSpeechAction::processSpeechCommand(const squirrel_speech_msgs::RecognizedCommand::ConstPtr& msg)
	{
		ros::Time current_time = ros::Time::now();
		ros::Duration delta = current_time - msg->header.stamp;
		
		// Check if this is a proper command.
		if (msg->is_command)
		{
			// Check if we have a similar command stored.
			for (std::vector<squirrel_speech_msgs::RecognizedCommand>::iterator i = active_commands_.begin(); i != active_commands_.end(); ++i)
			{
				const squirrel_speech_msgs::RecognizedCommand& existing_command = *i;
				
				// Only store the most recent one.
				if (existing_command.int_command == msg->int_command)
				{
					ros::Duration existing_delta = current_time - existing_command.header.stamp;
					if (delta < existing_delta)
					{
						//updateKnowledgeBase(existing_command, true);
						active_commands_.erase(i);
						active_commands_.push_back(*msg);
					}
					return;
				}
			}
		}
		
		updateKnowledgeBase(*msg, true);
		active_commands_.push_back(*msg);
	}
	
	void RPSpeechAction::purgeOldCommands()
	{
		// Remove all commands that have been issued more than 5 seconds ago.
		ros::Time current_time = ros::Time::now();
		

		// Remove all commands that have been given a while ago.
		for (int i = active_commands_.size() - 1; i >= 0; --i)
		{
			const squirrel_speech_msgs::RecognizedCommand& existing_command = active_commands_[i];
			
			ros::Duration existing_delta = current_time - existing_command.header.stamp;
			if (existing_delta.toSec() > 30)
			{
				updateKnowledgeBase(existing_command, false);
				active_commands_.erase(active_commands_.begin() + i);
				break;
			}
		}
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_speech");
		ros::NodeHandle nh;

		// create PDDL action subscriber
		KCL_rosplan::RPSpeechAction rpga(nh);
	
		ROS_INFO("KCL: (RPSpeechAction) Ready to receive");

		ros::Rate loop_rate(100);
		while (ros::ok())
		{
			rpga.purgeOldCommands();
			ros::spinOnce();
			loop_rate.sleep();
		}
		return 0;
	}
