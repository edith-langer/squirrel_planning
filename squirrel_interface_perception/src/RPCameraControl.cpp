#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>
#include <boost/concept_check.hpp>
#include <std_msgs/Float64.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "squirrel_object_perception_msgs/LookForObjectsAction.h"
#include "squirrel_object_perception_msgs/FindDynamicObjects.h"
#include "squirrel_interface_perception/RPCameraControl.h"
#include "squirrel_planning_knowledge_msgs/AddObjectService.h"
#include "mongodb_store/message_store.h"
#include "geometry_msgs/PoseStamped.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"

/* The implementation of RPMoveBase.h */
namespace KCL_rosplan {

	/* constructor */
	RPCameraControl::RPCameraControl(ros::NodeHandle &nh, std::string &camera_control_topic, float default_camera_angle)
	 : message_store(nh), camera_control_topic_(camera_control_topic), default_camera_angle_(default_camera_angle)
	{
		// create the action feedback publisher
		action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		get_attribute_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
		
		camera_topic_ = nh.advertise<std_msgs::Float64>(camera_control_topic, 10);
		mapping_topic_ = nh.advertise<std_msgs::Bool>("/squirrel_3d_mapping/update", 1);
	}

	/* action dispatch callback; parameters (?v - robot ?wp - waypoint) */
	void RPCameraControl::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		std_msgs::Float64 angle;
		
		// ignore non-perception action.
		if (0==msg->name.compare("aim_camera"))
		{
			rosplan_dispatch_msgs::ActionFeedback fb;
			fb.action_id = msg->action_id;
			fb.status = "action enabled";
			action_feedback_pub.publish(fb);
			
			// Get the name of the location.
			std::string waypoint_name;
			std::string robot_name;
			
			// Consult the message store, the current waypoint should have an associated angle.
			rosplan_knowledge_msgs::GetAttributeService get_attribute;
			get_attribute.request.predicate_name = "robot_at";
			if (!get_attribute_client.call(get_attribute)) {
				ROS_ERROR("KCL: (RPCameraControl) Failed to recieve the attributes of the predicate 'robot_at'");
				
				fb.action_id = msg->action_id;
				fb.status = "action failed";
				action_feedback_pub.publish(fb);
				
				return;
			}
			
			for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = get_attribute.response.attributes.begin(); ci != get_attribute.response.attributes.end(); ++ci) {
				const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item = *ci;
				for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = knowledge_item.values.begin(); ci != knowledge_item.values.end(); ++ci) {
					const diagnostic_msgs::KeyValue& key_value = *ci;
					if ("wp" == key_value.key) {
						waypoint_name = key_value.value;
					}
					if ("v" == key_value.key) {
						robot_name = key_value.value;
					}
				}
			}
				
			ROS_INFO("KCL: (RPCameraControl) Robot's location is: %s", waypoint_name.c_str());
			
			std::stringstream ss;
			ss << waypoint_name << "_angle";
			
			// Get the angle from the message store.
			std::vector<boost::shared_ptr<std_msgs::Float64> > angles;
			if (message_store.queryNamed<std_msgs::Float64>(ss.str(), angles))
			{
				for (std::vector<boost::shared_ptr<std_msgs::Float64> >::const_iterator ci = angles.begin(); ci != angles.end(); ++ci)
				{
					ROS_INFO("KCL: (RPCameraControl) Found angle: %f.", (*ci)->data);
				}
				
				if (angles.size() != 1)
				{
					ROS_ERROR("KCL: (RPCameraControl) Expected a single angle, but received %zd.", angles.size());
					
					fb.action_id = msg->action_id;
					fb.status = "action failed";
					action_feedback_pub.publish(fb);
					
					return;
				}

				std_msgs::Bool mapping_state;
				mapping_state.data = false;
				mapping_topic_.publish(mapping_state);
				
				angle.data = angles[0]->data;
				camera_topic_.publish(angle);

				mapping_state.data = true;
				mapping_topic_.publish(mapping_state);
				
				// Update the knowledge base.
				rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
				knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
				rosplan_knowledge_msgs::KnowledgeItem kenny_knowledge;
				kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				kenny_knowledge.attribute_name = "camera_neutral";
				kenny_knowledge.is_negative = false;
				
				diagnostic_msgs::KeyValue kv;
				kv.key = "v";
				kv.value = robot_name;
				kenny_knowledge.values.push_back(kv);
				
				knowledge_update_service.request.knowledge = kenny_knowledge;
				if (!update_knowledge_client.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (RPSquirrelRecursion) Could not remove the (camera_neutral %s) predicate from the knowledge base.", robot_name.c_str());
					fb.action_id = msg->action_id;
					fb.status = "action failed";
					action_feedback_pub.publish(fb);
					exit(-1);
				}
				ROS_INFO("KCL: (RPSquirrelRecursion) Removed the (camera_neutral %s) predicate from the knowledge base.", robot_name.c_str());
				
				knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
				kenny_knowledge.attribute_name = "camera_aimed";
				
				knowledge_update_service.request.knowledge = kenny_knowledge;
				if (!update_knowledge_client.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the (camera_aimed %s) predicate to the knowledge base.", robot_name.c_str());
					fb.action_id = msg->action_id;
					fb.status = "action failed";
					action_feedback_pub.publish(fb);
					exit(-1);
				}
				ROS_INFO("KCL: (RPSquirrelRecursion) Added the (camera_aimed %s) predicate to the knowledge base.", robot_name.c_str());
				
				fb.action_id = msg->action_id;
				fb.status = "action achieved";
				action_feedback_pub.publish(fb);
			}
			else
			{
				ROS_ERROR("KCL: (RPCameraControl) could not query message store to fetch the location's angle for %s.", ss.str().c_str());
				fb.action_id = msg->action_id;
				fb.status = "action failed";
				action_feedback_pub.publish(fb);
				return;
			}
		}
		else if(0==msg->name.compare("reset_camera"))
		{
			rosplan_dispatch_msgs::ActionFeedback fb;
			fb.action_id = msg->action_id;
			fb.status = "action enabled";
			action_feedback_pub.publish(fb);
			
			// Get the name of the location.
			std::string waypoint_name;
			std::string robot_name;
			
			// Consult the message store, the current waypoint should have an associated angle.
			rosplan_knowledge_msgs::GetAttributeService get_attribute;
			get_attribute.request.predicate_name = "robot_at";
			if (!get_attribute_client.call(get_attribute)) {
				ROS_ERROR("KCL: (RPCameraControl) Failed to recieve the attributes of the predicate 'robot_at'");
				
				fb.action_id = msg->action_id;
				fb.status = "action failed";
				action_feedback_pub.publish(fb);
				
				return;
			}

			std_msgs::Bool mapping_state;
			mapping_state.data = false;
			mapping_topic_.publish(mapping_state);
			
			angle.data = default_camera_angle_;
			camera_topic_.publish(angle);

			mapping_state.data = true;
			mapping_topic_.publish(mapping_state);
			
			// Update the knowledge base.
			rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
			knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
			rosplan_knowledge_msgs::KnowledgeItem kenny_knowledge;
			kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			kenny_knowledge.attribute_name = "camera_aimed";
			kenny_knowledge.is_negative = false;
			
			diagnostic_msgs::KeyValue kv;
			kv.key = "v";
			kv.value = robot_name;
			kenny_knowledge.values.push_back(kv);
			
			knowledge_update_service.request.knowledge = kenny_knowledge;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (RPSquirrelRecursion) Could not remove the (camera_aimed %s) predicate from the knowledge base.", robot_name.c_str());
				fb.action_id = msg->action_id;
				fb.status = "action failed";
				action_feedback_pub.publish(fb);
				exit(-1);
			}
			ROS_INFO("KCL: (RPSquirrelRecursion) Removed the (camera_aimed %s) predicate from the knowledge base.", robot_name.c_str());
			
			knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			kenny_knowledge.attribute_name = "camera_neutral";
			
			knowledge_update_service.request.knowledge = kenny_knowledge;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the (camera_neutral %s) predicate to the knowledge base.", robot_name.c_str());
				fb.action_id = msg->action_id;
				fb.status = "action failed";
				action_feedback_pub.publish(fb);
				exit(-1);
			}
			ROS_INFO("KCL: (RPSquirrelRecursion) Added the (camera_neutral %s) predicate to the knowledge base.", robot_name.c_str());
			
			fb.action_id = msg->action_id;
			fb.status = "action achieved";
			action_feedback_pub.publish(fb);
		}
	}

	void RPCameraControl::publishFeedback(int action_id, std::string feedback) {
		// publish feedback
		rosplan_dispatch_msgs::ActionFeedback fb;
		fb.action_id = action_id;
		fb.status = feedback;
		action_feedback_pub.publish(fb);
	}

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_camera");
		ros::NodeHandle nh;

		std::string camera_control_topic;
		nh.param("camera_control_topic", camera_control_topic, std::string("/tilt_controller/command"));
		
		float default_camera_angle;
		nh.param("default_camera_angle", default_camera_angle, 0.5f);

		// create PDDL action subscriber
		KCL_rosplan::RPCameraControl rpcc(nh, camera_control_topic, default_camera_angle);
	
		// listen for action dispatch
		ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPCameraControl::dispatchCallback, &rpcc);
		ROS_INFO("KCL: (CameraControl) Ready to receive");

		while(ros::ok() && ros::master::check()){ros::spinOnce();}
		return 0;
	}
