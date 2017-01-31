#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include "mongodb_store/message_store.h"


#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/GetInstanceService.h"
#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "rosplan_knowledge_msgs/GenerateProblemService.h"
#include "rosplan_knowledge_msgs/KnowledgeQueryService.h"


#ifndef KCL_camera_control
#define KCL_camera_control

#include <map>

/**
 * This file defines the RPCameraControl class.
 * RPCameraControl is used to control the tilt angle of the camera.
 */
namespace KCL_rosplan {

	class RPCameraControl
	{

	private:
		
		// Scene database
		mongodb_store::MessageStoreProxy message_store;
		
		// Feedback to the dispatcher.
		ros::Publisher action_feedback_pub;
		void publishFeedback(int action_id, std::string feedback);

		// Knowledge base
		ros::ServiceClient update_knowledge_client;
		ros::ServiceClient get_attribute_client;
		
		// Map ids passed by the caller to ids as stored in mongodb.
		std::multimap<std::string, std::string> mongo_id_mapping;
		
		// Topic to control the camera.
		std::string camera_control_topic_;
		
		// Default angle of the camera.
		float default_camera_angle_;
		
		// Publisher to control the camera.
		ros::Publisher camera_topic_;

		// Publisher to the mapping control.
		ros::Publisher mapping_topic_;
	public:

		/* constructor */
		RPCameraControl(ros::NodeHandle &nh, std::string &camera_control_topic, float default_camera_angle);
		
		/* listen to and process action_dispatch topic */
		void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif
