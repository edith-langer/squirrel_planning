#include <algorithm>
#include <std_srvs/Empty.h>
#include <rosplan_dispatch_msgs/ActionFeedback.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>

#include "GotoWaypointWrapper.h"

namespace KCL_rosplan
{

	GotoWaypointWrapper::GotoWaypointWrapper(ros::NodeHandle& nh, const std::string& actionserver, float fov, float view_distance)
		: message_store(nh), action_client(actionserver, true), fov_(fov), view_distance_(view_distance)
	{
		// costmap client
		clear_costmaps_client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
		
		update_knowledge_client_ = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		check_waypoint_ = nh.serviceClient<squirrel_object_perception_msgs::CheckWaypoint>("/squirrel_check_viewcone");
		action_feedback_pub_ = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
		ds_ = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::GotoWaypointWrapper::dispatchCallback, this);
	}
	
	GotoWaypointWrapper::~GotoWaypointWrapper()
	{
		
	}
	
	void GotoWaypointWrapper::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		std::string normalised_action_name = msg->name;
		std::transform(normalised_action_name.begin(), normalised_action_name.end(), normalised_action_name.begin(), tolower);
		
		// Check if this action is to be handled by this class.
		if (normalised_action_name != "goto_waypoint")
		{
			return;
		}
		
		ROS_INFO("KCL: (GotoWaypointWrapper) Process the action: %s", normalised_action_name.c_str());
		rosplan_dispatch_msgs::ActionFeedback fb;
		fb.action_id = msg->action_id;
		fb.status = "action enabled";
		action_feedback_pub_.publish(fb);
		
		// get waypoint ID from action dispatch
		std::string wpID;
		bool found = false;
		for(size_t i=0; i<msg->parameters.size(); i++) {
			if(0==msg->parameters[i].key.compare("to")) {
				wpID = msg->parameters[i].value;
				found = true;
			}
		}
		if(!found) {
			ROS_INFO("KCL: (GotoWaypointWrapper) aborting action dispatch; PDDL action missing required parameter ?to");
			fb.action_id = msg->action_id;
			fb.status = "action failed";
			action_feedback_pub_.publish(fb);
			return;
		}
		
		// get pose from message store
		std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results;
		if(message_store.queryNamed<geometry_msgs::PoseStamped>(wpID, results)) {
			if(results.size()<1) {
				ROS_INFO("KCL: (GotoWaypointWrapper) aborting action dispatch; no matching wpID %s", wpID.c_str());
				fb.action_id = msg->action_id;
				fb.status = "action failed";
				action_feedback_pub_.publish(fb);
			}
			if(results.size()>1)
				ROS_INFO("KCL: (GotoWaypointWrapper) multiple waypoints share the same wpID");

			// Check if we want to explore this waypoint.
			squirrel_object_perception_msgs::CheckWaypoint cw_data;
			cw_data.request.waypoint = results[0]->pose;
			cw_data.request.fov = fov_;
			cw_data.request.viewing_distance = view_distance_;
			
			if (!check_waypoint_.call(cw_data))
			{
				ROS_ERROR("KCL: (GotoWaypointWrapper) Failed to call the check waypoint service.");
				exit(-1);
			}
			
			// If we don't then we pretend this action is done and effectively skip it.
			if (!cw_data.response.explore_waypoint.data)
			{
				fb.action_id = msg->action_id;
				fb.status = "action achieved";
				action_feedback_pub_.publish(fb);
				return;
			}
			
			ROS_INFO("KCL: (GotoWaypointWrapper) waiting for move_base action server to start");
			action_client.waitForServer();

			// dispatch MoveBase action
			move_base_msgs::MoveBaseGoal goal;
			geometry_msgs::PoseStamped &pose = *results[0];
			
			std::cout << "Send the pose: (" << pose.pose.position.x << ", " << pose.pose.position.y << ", " << pose.pose.position.z << ") to movebase for waypoint: " << wpID << "." << std::endl;
			
			goal.target_pose = pose;
			action_client.sendGoal(goal);

			bool finished_before_timeout = action_client.waitForResult();
			if (finished_before_timeout) {

				actionlib::SimpleClientGoalState state = action_client.getState();
				ROS_INFO("KCL: (GotoWaypointWrapper) action finished: %s", state.toString().c_str());

				if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {

					// Update the domain.
					const std::string& robot = msg->parameters[0].value;
					const std::string& previous_waypoint = msg->parameters[1].value;
					const std::string& new_waypoint = msg->parameters[2].value;
					
					ROS_INFO("KCL: (GotoWaypointWrapper) Process the action: %s, Move %s from %s to %s", normalised_action_name.c_str(), robot.c_str(), previous_waypoint.c_str(), new_waypoint.c_str());
					
					// Remove the old knowledge.
					rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
					knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
					rosplan_knowledge_msgs::KnowledgeItem kenny_knowledge;
					kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
					kenny_knowledge.attribute_name = "robot_at";
					kenny_knowledge.is_negative = false;
					
					diagnostic_msgs::KeyValue kv;
					kv.key = "v";
					kv.value = robot;
					kenny_knowledge.values.push_back(kv);
					
					kv.key = "wp";
					kv.value = previous_waypoint;
					kenny_knowledge.values.push_back(kv);
					
					knowledge_update_service.request.knowledge = kenny_knowledge;
					if (!update_knowledge_client_.call(knowledge_update_service)) {
						ROS_ERROR("KCL: (GotoWaypointWrapper) Could not remove the previous (robot_at %s %s) predicate from the knowledge base.", robot.c_str(), previous_waypoint.c_str());
						exit(-1);
					}
					ROS_INFO("KCL: (GotoWaypointWrapper) Removed the previous (robot_at %s %s) predicate from the knowledge base.", robot.c_str(), previous_waypoint.c_str());
					kenny_knowledge.values.clear();
					
					// Add the new knowledge.
					knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
					kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
					kenny_knowledge.attribute_name = "robot_at";
					
					kv.key = "v";
					kv.value = robot;
					kenny_knowledge.values.push_back(kv);
					
					kv.key = "wp";
					kv.value = new_waypoint;
					kenny_knowledge.values.push_back(kv);
					
					knowledge_update_service.request.knowledge = kenny_knowledge;
					if (!update_knowledge_client_.call(knowledge_update_service)) {
						ROS_ERROR("KCL: (GotoWaypointWrapper) Could not add the new (robot_at %s %s) predicate to the knowledge base.", robot.c_str(), new_waypoint.c_str());
						exit(-1);
					}
					ROS_INFO("KCL: (GotoWaypointWrapper) Added the new (robot_at %s %s) predicate to the knowledge base.", robot.c_str(), new_waypoint.c_str());
					
					// publish feedback (achieved)
					fb.action_id = msg->action_id;
					fb.status = "action achieved";
					action_feedback_pub_.publish(fb);
					return;

				} else {

					// clear costmaps
					std_srvs::Empty emptySrv;
					clear_costmaps_client.call(emptySrv);

					// publish feedback (failed)
					fb.action_id = msg->action_id;
					fb.status = "action failed";
					action_feedback_pub_.publish(fb);
					return;
				}
			} else {
				// timed out (failed)
				action_client.cancelAllGoals();
				ROS_INFO("KCL: (GotoWaypointWrapper) action timed out");
				fb.action_id = msg->action_id;
				fb.status = "action failed";
				action_feedback_pub_.publish(fb);
				return;
			}
		} else {
			// no KMS connection (failed)
			ROS_INFO("KCL: (GotoWaypointWrapper) aborting action dispatch; query to sceneDB failed; wpID %s", wpID.c_str());
			fb.action_id = msg->action_id;
			fb.status = "action failed";
			action_feedback_pub_.publish(fb);
			return;
		}
	}
};
