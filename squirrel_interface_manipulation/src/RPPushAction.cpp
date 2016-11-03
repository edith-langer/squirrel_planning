#include "squirrel_interface_manipulation/RPPushAction.h"

/* The implementation of RPPushAction.h */
namespace KCL_rosplan {

	/* constructor */
	RPPushAction::RPPushAction(ros::NodeHandle &nh, std::string &pushactionserver, std::string &smashactionserver)
	 : message_store(nh), push_action_client(pushactionserver, true), smash_action_client(smashactionserver, true) {

		// create the push action client
		ROS_INFO("KCL: (PushAction) waiting for action server to start on %s", pushactionserver.c_str());
		push_action_client.waitForServer();
		ROS_INFO("KCL: (PushAction) waiting for action server to start on %s", smashactionserver.c_str());
		smash_action_client.waitForServer();

		// create the action feedback publisher
		action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);

		// create knowledge base link
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");

	}

	/* action dispatch callback */
	void RPPushAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// ignore non-push actions
		if(0==msg->name.compare("push_object")) dispatchPushAction(msg);
		if(0==msg->name.compare("smash_clutter")) dispatchSmashAction(msg);
	}

	/*
	 * smash action dispatch
	 */
	void RPPushAction::dispatchSmashAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		ROS_INFO("KCL: (PushAction) smash action recieved");

		// get waypoint ID from action dispatch
		std::string wpID;
		bool foundWP = false;
		for(size_t i=0; i<msg->parameters.size(); i++) {
			if(0==msg->parameters[i].key.compare("wp")) {
				wpID = msg->parameters[i].value;
				foundWP = true;
			}
		}
		if(!foundWP) {
			ROS_INFO("KCL: (PushAction) aborting action dispatch; malformed parameters");
			return;
		}
		
		// get pose from message store
		std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results;
		if(message_store.queryNamed<geometry_msgs::PoseStamped>(wpID, results)) {
			if(results.size()<1) {
				ROS_INFO("KCL: (PushAction) aborting action dispatch; no matching wpID %s", wpID.c_str());
				return;
			}
			if(results.size()>1)
				ROS_INFO("KCL: (PushAction) multiple waypoints share the same wpID");

			// dispatch Push action
			squirrel_manipulation_msgs::SmashGoal goal;
			goal.pose = results[0]->pose;
			smash_action_client.sendGoal(goal);

			// publish feedback (enabled)
			publishFeedback(msg->action_id, "action enabled");
			
			smash_action_client.waitForResult();

			actionlib::SimpleClientGoalState state = smash_action_client.getState();
			ROS_INFO("KCL: (PushAction) action finished: %s", state.toString().c_str());
							
			if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {

				// publish feedback (achieved)
				publishFeedback(msg->action_id, "action achieved");

			} else {

				// publish feedback (failed)
				publishFeedback(msg->action_id, "action failed");
			}

		} else {
			ROS_INFO("KCL: (PushAction) aborting action dispatch; query to sceneDB failed");
			publishFeedback(msg->action_id, "action failed");
		}
	}

	/* push action dispatch
		:parameters (?v - robot ?ob - object ?from ?to - waypoint)
		:effect (and
				(not (robot_at ?v ?from basic))
				(not (object_at ?ob ?from basic))
				(robot_at ?v ?to basic)
				(object_at ?ob ?to basic)
	 */
	void RPPushAction::dispatchPushAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		ROS_INFO("KCL: (PushAction) push action recieved");

		// get waypoint ID from action dispatch
		std::string wpID, objectID, robotID;
		bool foundWP = false;
		bool foundObject = false;
		for(size_t i=0; i<msg->parameters.size(); i++) {

			if(0==msg->parameters[i].key.compare("v"))
				robotID = msg->parameters[i].value;

			if(0==msg->parameters[i].key.compare("to")) {
				wpID = msg->parameters[i].value;
				foundWP = true;
			}
			if(0==msg->parameters[i].key.compare("ob")) {
				objectID = msg->parameters[i].value;
				foundObject = true;
			}
		}
		if(!foundWP || !foundObject) {
			ROS_INFO("KCL: (PushAction) aborting action dispatch; malformed parameters");
			return;
		}
		
		// get pose from message store
		std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results;
		if(message_store.queryNamed<geometry_msgs::PoseStamped>(wpID, results)) {
			if(results.size()<1) {
				ROS_INFO("KCL: (PushAction) aborting action dispatch; no matching wpID %s", wpID.c_str());
				return;
			}
			if(results.size()>1)
				ROS_INFO("KCL: (PushAction) multiple waypoints share the same wpID");

			// dispatch Push action
			squirrel_manipulation_msgs::PushGoal goal;
			goal.pose = results[0]->pose;
			goal.object_id = objectID;
			push_action_client.sendGoal(goal);

			// publish feedback (enabled)
			publishFeedback(msg->action_id, "action enabled");

			push_action_client.waitForResult();

			actionlib::SimpleClientGoalState state = push_action_client.getState();
			ROS_INFO("KCL: (PushAction) action finished: %s", state.toString().c_str());
							
			if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {

				diagnostic_msgs::KeyValue pairWP;
				pairWP.key = "to";
				pairWP.value = wpID;

				diagnostic_msgs::KeyValue pair;
				pair.key = "v";
				pair.value = robotID;

				// remove old robot_at
				rosplan_knowledge_msgs::KnowledgeUpdateService updatePredSrv;
				updatePredSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
				updatePredSrv.request.knowledge.attribute_name = "robot_at";
				updatePredSrv.request.knowledge.values.push_back(pair);
				update_knowledge_client.call(updatePredSrv);

				// predicate robot_at
				updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
				updatePredSrv.request.knowledge.attribute_name = "robot_at";
				updatePredSrv.request.knowledge.values.push_back(pairWP);
				update_knowledge_client.call(updatePredSrv);

				// remove old object_at
				updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
				updatePredSrv.request.knowledge.attribute_name = "object_at";
				updatePredSrv.request.knowledge.values.clear();
				pair.key = "o";
				pair.value = objectID;
				updatePredSrv.request.knowledge.values.push_back(pair);
				update_knowledge_client.call(updatePredSrv);

				// predicate object_at
				updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
				updatePredSrv.request.knowledge.attribute_name = "object_at";
				updatePredSrv.request.knowledge.values.push_back(pairWP);
				update_knowledge_client.call(updatePredSrv);

				// publish feedback (achieved)
				publishFeedback(msg->action_id, "action achieved");

			} else {

				// publish feedback (failed)
				publishFeedback(msg->action_id, "action failed");
			}

		} else {
			ROS_INFO("KCL: (PushAction) aborting action dispatch; query to sceneDB failed");
			publishFeedback(msg->action_id, "action failed");
		}
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_pushaction");
		ros::NodeHandle nh;

		std::string pushactionserver, smashactionserver;
		nh.param("push_action_server", pushactionserver, std::string("/push"));
		nh.param("smash_action_server", smashactionserver, std::string("/smash"));

		// create PDDL action subscriber
		KCL_rosplan::RPPushAction rppa(nh, pushactionserver, smashactionserver);
	
		// listen for action dispatch
		ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPPushAction::dispatchCallback, &rppa);
		ROS_INFO("KCL: (PushAction) Ready to receive");

		while(ros::ok() && ros::master::check()){ros::spinOnce();}
		return 0;
	}
