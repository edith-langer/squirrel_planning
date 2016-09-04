#include <std_msgs/Int8.h>

#include <map>
#include <algorithm>
#include <string>
#include <sstream>

#include "squirrel_planning_execution/RPSquirrelRecursion.h"
#include "squirrel_planning_execution/ContingentStrategicClassifyPDDLGenerator.h"
#include "squirrel_planning_execution/ContingentTacticalClassifyPDDLGenerator.h"
#include "squirrel_planning_execution/ContingentTidyPDDLGenerator.h"
#include "squirrel_planning_execution/ViewConeGenerator.h"
#include "squirrel_planning_execution/ClassicalTidyPDDLGenerator.h"
#include "pddl_actions/ShedKnowledgePDDLAction.h"
#include "pddl_actions/FinaliseClassificationPDDLAction.h"
#include "pddl_actions/PlannerInstance.h"

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>


/* The implementation of RPSquirrelRecursion.h */
namespace KCL_rosplan {

	/*-------------*/
	/* constructor */
	/*-------------*/

	RPSquirrelRecursion::RPSquirrelRecursion(ros::NodeHandle &nh)
		: node_handle(&nh), message_store(nh), initial_problem_generated(false), stop_when_enough_lumps_found(false), number_of_toys(0)
	{
		// knowledge interface
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		query_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>("/kcl_rosplan/query_knowledge_base");
		
		// create the action feedback publisher
		action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
		
		get_instance_client = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
		get_attribute_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
		
		std::string classifyTopic("/squirrel_perception_examine_waypoint");
		nh.param("squirrel_perception_classify_waypoint_service_topic", classifyTopic, classifyTopic);
		classify_object_waypoint_client = nh.serviceClient<squirrel_waypoint_msgs::ExamineWaypoint>(classifyTopic);
		
		pddl_generation_service = nh.advertiseService("/kcl_rosplan/generate_planning_problem", &KCL_rosplan::RPSquirrelRecursion::generatePDDLProblemFile, this);

		nh.getParam("/squirrel_planning_execution/stop_when_enough_lumps_found", stop_when_enough_lumps_found);
		nh.getParam("/number_of_toys", number_of_toys);
		
		ROS_INFO("KCL: (PS) Number of toys to find %d.", number_of_toys);
		
		std::string occupancyTopic("/map");
		nh.param("occupancy_topic", occupancyTopic, occupancyTopic);
		view_cone_generator = new ViewConeGenerator(nh, occupancyTopic);
	}
	
	/*---------------------------*/
	/* strategic action callback */
	/*---------------------------*/

	void RPSquirrelRecursion::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		rosplan_dispatch_msgs::ActionDispatch normalised_action_dispatch = *msg;
		std::string action_name = msg->name;
		std::transform(action_name.begin(), action_name.end(), action_name.begin(), tolower);
		normalised_action_dispatch.name = action_name;
		
		ROS_INFO("KCL: (RPSquirrelRecursion) Action received %s", msg->name.c_str());
		
		// ignore actions
		if("observe-classifiable_on_attempt" != action_name &&
		   "examine_area" != action_name &&
		   "explore_area" != action_name &&
		   "tidy_area" != action_name)
		{
			return;
		}

		bool actionAchieved = false;
		last_received_msg.push_back(normalised_action_dispatch);
		
		ROS_INFO("KCL: (RPSquirrelRecursion) action recieved %s", action_name.c_str());
		
		PlannerInstance& planner_instance = PlannerInstance::createInstance(*node_handle);
		
		// Lets start the planning process.
		std::string data_path;
		node_handle->getParam("/data_path", data_path);
		
		std::string planner_path;
		node_handle->getParam("/planner_path", planner_path);
		
		std::stringstream ss;
		ss << data_path << action_name << "_domain-nt.pddl";
		std::string domain_name = ss.str();
		
		ss.str(std::string());
		ss << data_path << action_name << "_problem.pddl";
		std::string problem_name = ss.str();
		
		ss.str(std::string());
		ss << "timeout 10 " << planner_path << "ff -o DOMAIN -f PROBLEM";
		std::string planner_command = ss.str();
		
		// Before calling the planner we create the domain so it can be parsed.
		if (!createDomain(action_name))
		{
			ROS_ERROR("KCL: (RPSquirrelRecursion) failed to produce a domain at %s for action name %s.", domain_name.c_str(), action_name.c_str());
			return;
		}
		
		planner_instance.startPlanner(domain_name, problem_name, data_path, planner_command);
		
		// publish feedback (enabled)
		rosplan_dispatch_msgs::ActionFeedback fb;
		fb.action_id = msg->action_id;
		fb.status = "action enabled";
		action_feedback_pub.publish(fb);

		// wait for action to finish
		ros::Rate loop_rate(1);
		while (ros::ok() && (planner_instance.getState() == actionlib::SimpleClientGoalState::ACTIVE || planner_instance.getState() == actionlib::SimpleClientGoalState::PENDING)) {
			ros::spinOnce();
			loop_rate.sleep();
			
			// Check if we have found enough objects.
			if (stop_when_enough_lumps_found && action_name == "explore_area")
			{
				// Count the number of objects we have found.
				rosplan_knowledge_msgs::GetInstanceService get_instances_msg;
				get_instances_msg.request.type_name = "object";
				if (!get_instance_client.call(get_instances_msg))
				{
					ROS_ERROR("KCL: (RPSquirrelRecursion) Could not request instances of type %s from the knowledge base.", get_instances_msg.request.type_name.c_str());
					exit(-1);
				}
				
				// If we have found enough objects, we skip to the examination phase.
				if (get_instances_msg.response.instances.size() == number_of_toys)
				{
					ROS_INFO("KCL: (RPSquirrelRecursion) We have enough objects in the domain.");
					planner_instance.stopPlanner();
					break;
				}
			}
			
			// Check if we have classified enough objects.
			rosplan_knowledge_msgs::GetInstanceService get_instances_msg;
			get_instances_msg.request.type_name = "object";
			if (!get_instance_client.call(get_instances_msg))
			{
				ROS_ERROR("KCL: (RPSquirrelRecursion) Could not request instances of type %s from the knowledge base.", get_instances_msg.request.type_name.c_str());
				exit(-1);
			}
			
			// Objects that are classified have a type that is not 'unknown'.
			unsigned int classified_objects = 0;
			
			// Find if this object has been classified at any location.
			for (std::vector<std::string>::const_iterator ci = get_instances_msg.response.instances.begin(); ci != get_instances_msg.response.instances.end(); ++ci)
			{
				const std::string& object = *ci;
				
				rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
				knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				knowledge_item.attribute_name = "is_of_type";
				
				diagnostic_msgs::KeyValue kv;
				kv.key = "t";
				kv.value = "unknown";
				knowledge_item.values.push_back(kv);
				
				kv.key = "o";
				kv.value = object;
				knowledge_item.values.push_back(kv);
				knowledge_item.is_negative = false;
				
				// Query the knowledge base.
				rosplan_knowledge_msgs::KnowledgeQueryService knowledge_query;
				knowledge_query.request.knowledge.push_back(knowledge_item);
				
				// Check if any of these facts are true.
				if (!query_knowledge_client.call(knowledge_query))
				{
					ROS_ERROR("KCL: (RPSquirrelRecursion) Could not call the query knowledge server.");
					exit(1);
				}
				
				if (knowledge_query.response.all_true)
				{
					++classified_objects;
					ROS_INFO("KCL: (RPSquirrelRecursion) The object: %s has been classified.", object.c_str());
				}
			}
			
			// Check if we are done.
			if (classified_objects == number_of_toys)
			{
				ROS_INFO("KCL: (RPSquirrelRecursion) All objects are classified.");
				exit(0);
			}
		}

		actionlib::SimpleClientGoalState state = planner_instance.getState();
		ROS_INFO("KCL: (RPSquirrelRecursion) action finished: %s, %s", action_name.c_str(), state.toString().c_str());

		if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
			
			// Update the knowledge base with what has been achieved.
			if ("explore_area" == action_name)
			{
				// Update the domain.
				const std::string& robot = msg->parameters[0].value;
				const std::string& area = msg->parameters[1].value;
				
				ROS_INFO("KCL: (RPSquirrelRecursion) Process the action: %s, Explore %s by %s", action_name.c_str(), area.c_str(), robot.c_str());
				
				// Remove the old knowledge.
				rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
				knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
				rosplan_knowledge_msgs::KnowledgeItem kenny_knowledge;
				kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				kenny_knowledge.attribute_name = "explored";
				kenny_knowledge.is_negative = false;
				
				diagnostic_msgs::KeyValue kv;
				kv.key = "a";
				kv.value = area;
				kenny_knowledge.values.push_back(kv);
				
				knowledge_update_service.request.knowledge = kenny_knowledge;
				if (!update_knowledge_client.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the (explored %s) predicate to the knowledge base.", area.c_str());
					exit(-1);
				}
				ROS_INFO("KCL: (RPSquirrelRecursion) Added the (explored %s) predicate to the knowledge base.", area.c_str());
				kenny_knowledge.values.clear();
			} else if ("examine_area" == action_name) {
				// Update the domain.
				const std::string& robot = msg->parameters[0].value;
				const std::string& area = msg->parameters[1].value;
				
				ROS_INFO("KCL: (RPSquirrelRecursion) Process the action: %s, Examine %s by %s", action_name.c_str(), area.c_str(), robot.c_str());
				
				// Remove the old knowledge.
				rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
				knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
				rosplan_knowledge_msgs::KnowledgeItem kenny_knowledge;
				kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				kenny_knowledge.attribute_name = "examined";
				kenny_knowledge.is_negative = false;
				
				diagnostic_msgs::KeyValue kv;
				kv.key = "a";
				kv.value = area;
				kenny_knowledge.values.push_back(kv);
				
				knowledge_update_service.request.knowledge = kenny_knowledge;
				if (!update_knowledge_client.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the (examined %s) predicate to the knowledge base.", area.c_str());
					exit(-1);
				}
				ROS_INFO("KCL: (RPSquirrelRecursion) Added the action (examined %s) predicate to the knowledge base.", area.c_str());
				kenny_knowledge.values.clear();
			} else if ("observe-classifiable_on_attempt" == action_name) {
				// Update the domain.
				const std::string& object = msg->parameters[0].value;
				const std::string& counter = msg->parameters[1].value;
				
				// Add the new knowledge.
				rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
				knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
				rosplan_knowledge_msgs::KnowledgeItem kenny_knowledge;
				kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				kenny_knowledge.attribute_name = "classifiable_on_attempt";
				
				// Check if this object has been classified or not.
				rosplan_knowledge_msgs::GetInstanceService get_instance;
				get_instance.request.type_name = "waypoint";
				
				if (!get_instance_client.call(get_instance))
				{
					ROS_ERROR("KCL: (RPSquirrelRecursion) Could not get the instances of type 'waypoint'.");
					exit(1);
				}
				
				rosplan_knowledge_msgs::KnowledgeQueryService knowledge_query;
				
				// Find if this object has been classified at any location.
				for (std::vector<std::string>::const_iterator ci = get_instance.response.instances.begin(); ci != get_instance.response.instances.end(); ++ci)
				{
					const std::string& wp1 = *ci;
					for (std::vector<std::string>::const_iterator ci = get_instance.response.instances.begin(); ci != get_instance.response.instances.end(); ++ci)
					{
						const std::string& wp2 = *ci;
						
						rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
						knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
						knowledge_item.attribute_name = "classifiable_from";
						
						diagnostic_msgs::KeyValue kv;
						kv.key = "from";
						kv.value = wp1;
						knowledge_item.values.push_back(kv);
						
						kv.key = "view";
						kv.value = wp2;
						knowledge_item.values.push_back(kv);
						
						kv.key = "o";
						kv.value = object;
						knowledge_item.values.push_back(kv);
						knowledge_item.is_negative = false;
						
						knowledge_query.request.knowledge.push_back(knowledge_item);
					}
				}
				
				// Check if any of these facts are true.
				if (!query_knowledge_client.call(knowledge_query))
				{
					ROS_ERROR("KCL: (RPSquirrelRecursion) Could not call the query knowledge server.");
					exit(1);
				}
				
				bool object_is_classified = false;
				for (std::vector<unsigned char>::const_iterator ci = knowledge_query.response.results.begin(); ci != knowledge_query.response.results.end(); ci++)
				{
					if (*ci == 1)
					{
						object_is_classified = true;
						break;
					}
				}
				
				kenny_knowledge.is_negative = !object_is_classified;
				if (kenny_knowledge.is_negative)
				{
					ROS_INFO("KCL: (RPSquirrelRecursion) %s was not classified on the %sth attempt.", object.c_str(), counter.c_str());
				}
				else
				{
					ROS_INFO("KCL: (RPSquirrelRecursion) %s was classified on the %sth attempt!", object.c_str(), counter.c_str());
				}
				
				diagnostic_msgs::KeyValue kv;
				kv.key = "o";
				kv.value = object;
				kenny_knowledge.values.push_back(kv);
				
				kv.key = "c";
				kv.value = counter;
				kenny_knowledge.values.push_back(kv);
				
				knowledge_update_service.request.knowledge = kenny_knowledge;
				if (!update_knowledge_client.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (ClassifyObjectPDDLAction) Could not add the classifiable_on_attempt predicate to the knowledge base.");
					exit(-1);
				}
				ROS_INFO("KCL: (ClassifyObjectPDDLAction) Added classifiable_on_attempt predicate to the knowledge base.");
				kenny_knowledge.values.clear();
			}
			
			// publish feedback (achieved)
			rosplan_dispatch_msgs::ActionFeedback fb;
			fb.action_id = msg->action_id;
			fb.status = "action achieved";
			action_feedback_pub.publish(fb);
		} else {
			// publish feedback (failed)
			rosplan_dispatch_msgs::ActionFeedback fb;
			fb.action_id = msg->action_id;
			fb.status = "action failed";
			action_feedback_pub.publish(fb);
		}

		last_received_msg.pop_back();
	}
	
	/*--------------------*/
	/* problem generation */
	/*--------------------*/

	/**
	 * Generate a contingent problem
	 */
	bool RPSquirrelRecursion::generatePDDLProblemFile(rosplan_knowledge_msgs::GenerateProblemService::Request &req, rosplan_knowledge_msgs::GenerateProblemService::Response &res) {
		
		ROS_INFO("KCL: (RPSquirrelRecursion) generatePDDLProblemFile: %s", req.problem_path.c_str());
		
		// Lets start the planning process.
		std::string data_path;
		node_handle->getParam("/data_path", data_path);
		
		/**
		 * If no message has been received yet we setup the initial condition.
		 */
		if (last_received_msg.empty() && !initial_problem_generated) {
			ROS_INFO("KCL: (RPSquirrelRecursion) Create the initial problem.");
			
			std::stringstream domain_ss;
			domain_ss << data_path << "tidy_room_domain-nt.pddl";
			std::string domain_name = domain_ss.str();
			
			generateInitialState();
			PlanningEnvironment planning_environment;
			planning_environment.parseDomain(domain_name);
			planning_environment.update(*node_handle);
			PDDLProblemGenerator pddl_problem_generator;
			
			pddl_problem_generator.generatePDDLProblemFile(planning_environment, req.problem_path);
			initial_problem_generated = true;
			return true;
		}
		
		else if (last_received_msg.empty())
		{
			ROS_INFO("KCL: (RPSquirrelRecursion) No messages received...");
			return false;
		}
		
		return true;
	}
	
	void RPSquirrelRecursion::generateInitialState()
	{
		// Add kenny
		rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
		knowledge_item.instance_type = "robot";
		knowledge_item.instance_name = "kenny";
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add kenny to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (RPSquirrelRecursion) Added kenny to the knowledge base.");
		
		// Add the single room.
		knowledge_item.instance_type = "area";
		knowledge_item.instance_name = "room";
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add area to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (RPSquirrelRecursion) Added area to the knowledge base.");
		
		// Set the location of the robot.
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge_item.attribute_name = "robot_in";
		knowledge_item.is_negative = false;
		diagnostic_msgs::KeyValue kv;
		kv.key = "v";
		kv.value = "kenny";
		knowledge_item.values.push_back(kv);
		kv.key = "a";
		kv.value = "room";
		knowledge_item.values.push_back(kv);
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the fact (robot_in kenny room) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (RPSquirrelRecursion) Added (robot_in kenny room) to the knowledge base.");
		knowledge_item.values.clear();
		
		// Setup the goal.
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge_item.attribute_name = "tidy";
		knowledge_item.is_negative = false;
		kv.key = "a";
		kv.value = "room";
		knowledge_item.values.push_back(kv);
		
		
		
		// Add the goal.
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the goal (tidy room) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (RPSquirrelRecursion) Added the goal (tidy room) to the knowledge base.");
	}
	
	bool RPSquirrelRecursion::createDomain(const std::string& action_name)
	{
		ROS_INFO("KCL: (RPSquirrelRecursion) Create domain for action %s.", action_name.c_str());
		// Lets start the planning process.
		std::string data_path;
		node_handle->getParam("/data_path", data_path);

		std::stringstream ss;

		ss << last_received_msg.back().name << "_domain-nt.pddl";
		std::string domain_name = ss.str();
		ss.str(std::string());

		ss << data_path << domain_name;
		std::string domain_path = ss.str();		
		ss.str(std::string());

		ss << last_received_msg.back().name << "_problem.pddl";
		std::string problem_name = ss.str();
		ss.str(std::string());

		ss << data_path << problem_name;
		std::string problem_path = ss.str();
		ss.str(std::string());
		
 		if (action_name == "explore_area") {
			
			//rviz things
			std::vector<geometry_msgs::Point> waypoints;
			std::vector<std_msgs::ColorRGBA> waypoint_colours;
			std::vector<geometry_msgs::Point> triangle_points;
			std::vector<std_msgs::ColorRGBA> triangle_colours;

			std::vector<geometry_msgs::Pose> view_poses;
			std::vector<tf::Vector3> bounding_box;
			bounding_box.push_back(tf::Vector3(-2.0, 0.9, 0.0));
			bounding_box.push_back(tf::Vector3(-2.0, -9.54, 0.0));
			bounding_box.push_back(tf::Vector3(6.05, -9.54, 0.0));
			bounding_box.push_back(tf::Vector3(6.05, 0.9, 0.0));
			view_cone_generator->createViewCones(view_poses, bounding_box, 10, 5, 0.94f, 4.0f, 100, 0.35f);
			
			// Add these poses to the knowledge base.
			rosplan_knowledge_msgs::KnowledgeUpdateService add_waypoints_service;
			add_waypoints_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			
			unsigned int waypoint_number = 0;
			std::stringstream ss;
			for (std::vector<geometry_msgs::Pose>::const_iterator ci = view_poses.begin(); ci != view_poses.end(); ++ci) {
				
				ss.str(std::string());
				ss << "explore_wp" << waypoint_number;
				rosplan_knowledge_msgs::KnowledgeItem waypoint_knowledge;
				add_waypoints_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
				waypoint_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
				waypoint_knowledge.instance_type = "waypoint";
				waypoint_knowledge.instance_name = ss.str();
				add_waypoints_service.request.knowledge = waypoint_knowledge;
				if (!update_knowledge_client.call(add_waypoints_service)) {
					ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add an explore wayoint to the knowledge base.");
					exit(-1);
				}

				// add waypoint to MongoDB
				geometry_msgs::PoseStamped pose;
				pose.header.frame_id = "/map";
				pose.pose = *ci;
				std::string id(message_store.insertNamed(ss.str(), pose));
				
				// Setup the goal.
				waypoint_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				waypoint_knowledge.attribute_name = "explored";
				waypoint_knowledge.is_negative = false;
				diagnostic_msgs::KeyValue kv;
				kv.key = "wp";
				kv.value = ss.str();
				waypoint_knowledge.values.push_back(kv);
				
				// Add the goal.
				add_waypoints_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
				add_waypoints_service.request.knowledge = waypoint_knowledge;
				if (!update_knowledge_client.call(add_waypoints_service)) {
					ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the goal (explored %s) to the knowledge base.", ss.str().c_str());
					exit(-1);
				}
				ROS_INFO("KCL: (RPSquirrelRecursion) Added the goal (explored %s) to the knowledge base.", ss.str().c_str());
				++waypoint_number;
			}

			// add initial state (robot_at)
			rosplan_knowledge_msgs::KnowledgeItem waypoint_knowledge;
			add_waypoints_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			waypoint_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			waypoint_knowledge.instance_type = "waypoint";
			waypoint_knowledge.instance_name = "kenny_waypoint";
			add_waypoints_service.request.knowledge = waypoint_knowledge;
			if (!update_knowledge_client.call(add_waypoints_service)) {
				ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add an explore wayoint to the knowledge base.");
				exit(-1);
			}
			
			// Set the location of the robot.
			waypoint_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			waypoint_knowledge.attribute_name = "robot_at";
			waypoint_knowledge.is_negative = false;
			diagnostic_msgs::KeyValue kv;
			kv.key = "v";
			kv.value = "kenny";
			waypoint_knowledge.values.push_back(kv);
			kv.key = "wp";
			kv.value = "kenny_waypoint";
			waypoint_knowledge.values.push_back(kv);
			add_waypoints_service.request.knowledge = waypoint_knowledge;
			if (!update_knowledge_client.call(add_waypoints_service)) {
				ROS_ERROR("KCL: (TidyRooms) Could not add the fact (robot_at kenny room) to the knowledge base.");
				exit(-1);
			}
			ROS_INFO("KCL: (TidyRooms) Added (robot_at kenny room) to the knowledge base.");
			waypoint_knowledge.values.clear();
			
			
			std_msgs::Int8 nr_waypoint_number_int8;
			nr_waypoint_number_int8.data = waypoint_number;
			ROS_INFO("KCL: (RPSquirrelRecursion) Added %d waypoints to the knowledge base.", nr_waypoint_number_int8.data);
			
			PlanningEnvironment planning_environment;
			planning_environment.parseDomain(domain_path);
			planning_environment.update(*node_handle);
			PDDLProblemGenerator pddl_problem_generator;
			
			pddl_problem_generator.generatePDDLProblemFile(planning_environment, problem_path);

		} else if (action_name == "examine_area") {
			
			// Fetch all the objects.
			rosplan_knowledge_msgs::GetAttributeService get_attribute;
			get_attribute.request.predicate_name = "object_at";
			if (!get_attribute_client.call(get_attribute)) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve the attributes of the predicate 'object_at'");
				return false;
			}
			
			std::map<std::string, std::string> object_to_location_mappings;
			std::map<std::string, std::vector<std::string> > near_waypoint_mappings;
			int max_objects = 0;
			for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = get_attribute.response.attributes.begin(); ci != get_attribute.response.attributes.end(); ++ci) {

				max_objects++;
				if(max_objects > 3) break;

				const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item = *ci;
				std::string object_predicate;
				std::string location_predicate;
				for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = knowledge_item.values.begin(); ci != knowledge_item.values.end(); ++ci) {
					const diagnostic_msgs::KeyValue& key_value = *ci;
					if ("o" == key_value.key) {
						object_predicate = key_value.value;
					}
					
					if ("wp" == key_value.key) {
						location_predicate = key_value.value;
					}
				}
				
				object_to_location_mappings[object_predicate] = location_predicate;
				
				// Find waypoints that are near this waypoint, these waypoints are used by the 
				// robot to pickup or push this object.
				rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
				knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
				
				std::vector<std::string> near_waypoints;
				for (unsigned int i = 0; i < 1; ++i)
				{
					rosplan_knowledge_msgs::KnowledgeItem kenny_knowledge;
					kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
					std::stringstream ss;
					ss << "near_" << location_predicate << "_" << i;
					
					near_waypoints.push_back(ss.str());
					
					kenny_knowledge.instance_type = "waypoint";
					kenny_knowledge.instance_name = ss.str();
					
					knowledge_update_service.request.knowledge = kenny_knowledge;
					if (!update_knowledge_client.call(knowledge_update_service)) {
						ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the waypoint %s to the knowledge base.", ss.str().c_str());
						exit(-1);
					}
					ROS_INFO("KCL: (RPSquirrelRecursion) Added %s to the knowledge base.", ss.str().c_str());
					
					kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
					kenny_knowledge.attribute_name = "near";
					kenny_knowledge.is_negative = false;
					diagnostic_msgs::KeyValue kv;
					kv.key = "wp1";
					kv.value = ss.str();
					kenny_knowledge.values.push_back(kv);
					kv.key = "wp2";
					kv.value = location_predicate;
					kenny_knowledge.values.push_back(kv);
					knowledge_update_service.request.knowledge = kenny_knowledge;
					if (!update_knowledge_client.call(knowledge_update_service)) {
						ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the fact (near %s %s) to the knowledge base.", ss.str().c_str(), location_predicate.c_str());
						exit(-1);
					}
					ROS_INFO("KCL: (RPSquirrelRecursion) Added (near %s %s) to the knowledge base.", ss.str().c_str(), location_predicate.c_str());
					kenny_knowledge.values.clear();
				}
				near_waypoint_mappings[location_predicate] = near_waypoints;
			}
			std_msgs::Int8 nr_objects;
			nr_objects.data = object_to_location_mappings.size();
			ROS_INFO("KCL: (RPSquirrelRecursion) Found %d objects to eximine.", nr_objects.data);
			
			// Get the location of kenny.
			get_attribute.request.predicate_name = "robot_at";
			if (!get_attribute_client.call(get_attribute)) {// || get_attribute.response.attributes.size() != 3) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve the attributes of the predicate 'robot_at'");
				return false;
			}
			
			std::string robot_location;
			for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = get_attribute.response.attributes[0].values.begin(); ci != get_attribute.response.attributes[0].values.end(); ++ci) {
				const diagnostic_msgs::KeyValue& knowledge_item = *ci;
				
				ROS_INFO("KCL: (RPSquirrelRecursion) Process robot_at attribute: %s %s", knowledge_item.key.c_str(), knowledge_item.value.c_str());
				
				if ("wp" == knowledge_item.key) {
					robot_location = knowledge_item.value;
				}
			}
			
			if ("" == robot_location) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve the location of Kenny");
				return false;
			}
			
			ROS_INFO("KCL: (RPSquirrelRecursion) Kenny is at waypoint: %s", robot_location.c_str());
			
			// Check which objects have already been classified.
			get_attribute.request.predicate_name = "is_of_type";
			if (!get_attribute_client.call(get_attribute)) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve the attributes of the predicate 'is_of_type'");
				return false;
			}
			
			for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = get_attribute.response.attributes.begin(); ci != get_attribute.response.attributes.end(); ++ci) {
				const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item = *ci;
				std::string object_predicate;
				std::string type_predicate;
				for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = knowledge_item.values.begin(); ci != knowledge_item.values.end(); ++ci) {
					const diagnostic_msgs::KeyValue& key_value = *ci;
					if ("o" == key_value.key) {
						object_predicate = key_value.value;
					}
					
					if ("t" == key_value.key) {
						type_predicate = key_value.value;
					}
				}
				
				if (type_predicate != "unknown")
				{
					object_to_location_mappings.erase(object_predicate);
					ROS_INFO("KCL: (RPSquirrelRecursion) No need to classify %s it is of type %s", object_predicate.c_str(), type_predicate.c_str());
				}
			}
			
			if (object_to_location_mappings.empty())
			{
				ROS_INFO("KCL: (RPSquirrelRecursion) All objects are all ready classified (or we found none!)");
			}
			else
			{
				ContingentStrategicClassifyPDDLGenerator::createPDDL(data_path, domain_name, problem_name, robot_location, object_to_location_mappings, near_waypoint_mappings, 3);
			}
			
		// Create the classify_object contingent domain and problem files.
		} else if (action_name == "observe-classifiable_on_attempt") {
		
			// Find the object that needs to be classified.
			std::string object_name;
			
			ROS_INFO("KCL: (RPSquirrelRecursion) %s.", action_name.c_str());
			
			object_name = last_received_msg.back().parameters[0].value;
			std::transform(object_name.begin(), object_name.end(), object_name.begin(), tolower);
			
			ROS_INFO("KCL: (RPSquirrelRecursion) Object name is: %s", object_name.c_str());
			
			// Get the location of the object.
			// (object_at ?o - object ?wp - location)
			rosplan_knowledge_msgs::GetAttributeService get_attribute;
			get_attribute.request.predicate_name = "object_at";
			if (!get_attribute_client.call(get_attribute)) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve the attributes of the predicate 'object_at'");
				return false;
			}
			
			std::string object_location;
			bool found_object_location = false;
			for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = get_attribute.response.attributes.begin(); ci != get_attribute.response.attributes.end(); ++ci) {
				const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item = *ci;
				ROS_INFO("KCL: (RPSquirrelRoadmap) %s", knowledge_item.attribute_name.c_str());
				for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = knowledge_item.values.begin(); ci != knowledge_item.values.end(); ++ci) {
					const diagnostic_msgs::KeyValue& key_value = *ci;
					if ("o" == key_value.key && object_name == key_value.value) {
						found_object_location = true;
					}
					
					if ("wp" == key_value.key) {
						object_location = key_value.value;
					}
					
					ROS_INFO("KCL: (RPSquirrelRoadmap) %s -> %s.", key_value.key.c_str(), key_value.value.c_str());
				}
				
				if (found_object_location) {
					break;
				}
			}
			
			if (!found_object_location) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve the location of the object %s", object_name.c_str());
				return false;
			}
			
			ROS_INFO("KCL: (RPSquirrelRecursion) Object location is: %s", object_location.c_str());
			
			squirrel_waypoint_msgs::ExamineWaypoint getTaskPose;

			// fetch position of object from message store
			std::vector< boost::shared_ptr<squirrel_object_perception_msgs::SceneObject> > results;
			if(message_store.queryNamed<squirrel_object_perception_msgs::SceneObject>(object_name, results)) {

				if(results.size()<1) {
					ROS_ERROR("KCL: (RPSquirrelRoadmap) aborting waypoint request; no matching obID %s", object_name.c_str());
					return false;
				}
			} else {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) could not query message store to fetch object pose");
				return false;
			}

			// request classification waypoints for object
			squirrel_object_perception_msgs::SceneObject &obj = *results[0];
			
			getTaskPose.request.object_pose.header = obj.header;
			getTaskPose.request.object_pose.pose = obj.pose;
			if (!classify_object_waypoint_client.call(getTaskPose)) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve classification waypoints for %s.", object_name.c_str());
				return false;
			}

			std_msgs::Int8 debug_pose_number;
			debug_pose_number.data = getTaskPose.response.poses.size();
			ROS_INFO("KCL: (RPSquirrelRecursion) Found %d observation poses", debug_pose_number.data);

			// Add all the waypoints to the knowledge base.
			std::stringstream ss;
			std::vector<std::string> observation_location_predicates;
			for(int i=0;i<getTaskPose.response.poses.size(); i++) {
				
				ss.str(std::string());
				ss << object_name << "_observation_wp" << i;
				
				ROS_INFO("KCL: (RPSquirrelRecursion) Process observation pose: %s", ss.str().c_str());
				
				rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
				updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
				updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
				updateSrv.request.knowledge.instance_type = "waypoint";
				updateSrv.request.knowledge.instance_name = ss.str();
				update_knowledge_client.call(updateSrv);
				
				observation_location_predicates.push_back(ss.str());
			}
			
			// Add a special observation waypoint.
			observation_location_predicates.push_back("nowhere");
			
			// Get the location of kenny.
			get_attribute.request.predicate_name = "robot_at";
			if (!get_attribute_client.call(get_attribute)) {// || get_attribute.response.attributes.size() != 3) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve the attributes of the predicate 'robot_at'");
				return false;
			}
			
			std::string robot_location;
			for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = get_attribute.response.attributes[0].values.begin(); ci != get_attribute.response.attributes[0].values.end(); ++ci) {
				const diagnostic_msgs::KeyValue& knowledge_item = *ci;
				
				ROS_INFO("KCL: (RPSquirrelRecursion) Process robot_at attribute: %s %s", knowledge_item.key.c_str(), knowledge_item.value.c_str());
				
				if ("wp" == knowledge_item.key) {
					robot_location = knowledge_item.value;
				}
			}
			
			if ("" == robot_location) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve the location of Kenny");
				return false;
			}
			
			ROS_INFO("KCL: (RPSquirrelRecursion) Kenny is at waypoint: %s", robot_location.c_str());
			
			ContingentTacticalClassifyPDDLGenerator::createPDDL(data_path, domain_name, problem_name, robot_location, observation_location_predicates, object_name, object_location);
		} else if (action_name == "tidy_area") {
			// Get all the objects in the knowledge base that are in this area. 
			// TODO For now we assume there is only one area, so all objects in the knowledge base are relevant (unless already tidied).
			
			
			rosplan_knowledge_msgs::GetAttributeService get_attribute;
			
			// Get the location of the boxes.
			// (box_at ?b - box ?wp - waypoint)
			get_attribute.request.predicate_name = "box_at";
			if (!get_attribute_client.call(get_attribute)) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve the attributes of the predicate 'box_at'");
				return false;
			}
			
			std::map<std::string, std::string> box_to_location_mapping;
			std::map<std::string, std::vector<std::string> > near_box_location_mapping;
			std::map<std::string, geometry_msgs::Pose> box_to_pose_mapping;
			std::map<std::string, geometry_msgs::Pose> type_to_box_pose_mapping;
			for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = get_attribute.response.attributes.begin(); ci != get_attribute.response.attributes.end(); ++ci) {
				const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item = *ci;
				std::string box_predicate;
				std::string box_location_predicate;
				for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = knowledge_item.values.begin(); ci != knowledge_item.values.end(); ++ci) {
					const diagnostic_msgs::KeyValue& key_value = *ci;
					if ("b" == key_value.key) {
						box_predicate = key_value.value;
					} else if ("wp" == key_value.key) {
						box_location_predicate = key_value.value;
					}
				}
				
				// Get the actual location of this box.
				std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results;
				
				std::stringstream ss;
				ss << "near_" << box_location_predicate;
				
				if(message_store.queryNamed<geometry_msgs::PoseStamped>(box_location_predicate, results))
				{
					if (results.size() < 1)
					{
						ROS_ERROR("KCL: (RPSquirrelRecursion) aborting waypoint request; no matching wpID %s", box_location_predicate.c_str());
						exit(-1);
					}
				} else {
					ROS_ERROR("KCL: (RPSquirrelRecursion) could not query message store to fetch waypoint pose");
					exit(-1);
				}
				const geometry_msgs::PoseStamped &box_wp = *results[0];
				box_to_pose_mapping[box_predicate] = box_wp.pose;
				
				// Create a waypoint 44 cm from this box at a random angle.
				float angle = ((float)rand() / (float)RAND_MAX) * 360.0f;
				tf::Vector3 v(0.44f, 0.0f, 0.0f);
				v.rotate(tf::Vector3(0, 0, 1), angle);
				v += tf::Vector3(box_wp.pose.position.x, box_wp.pose.position.y, 0.0f);
				
				tf::Quaternion v_rotation(tf::Vector3(0, 0, 1), angle + 180.0f);
				
				// Store this location in the knowledge base.
				geometry_msgs::PoseStamped near_pose;
				near_pose.header.seq = 0;
				near_pose.header.stamp = ros::Time::now();
				near_pose.header.frame_id = "/map";
				near_pose.pose.position.x = v.x();
				near_pose.pose.position.y = v.y();
				near_pose.pose.position.z = 0.0f;
				
				near_pose.pose.orientation.x = v_rotation.x();
				near_pose.pose.orientation.y = v_rotation.y();
				near_pose.pose.orientation.z = v_rotation.z();
				near_pose.pose.orientation.w = v_rotation.w();
				
				std::string near_waypoint_mongodb_id(message_store.insertNamed(ss.str(), near_pose));
				
				box_to_location_mapping[box_predicate] = box_location_predicate;
				
				std::vector<std::string> waypoints_near_box;
				
				rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
				knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
				
				rosplan_knowledge_msgs::KnowledgeItem kenny_knowledge;
				kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
				
				waypoints_near_box.push_back(ss.str());
				
				kenny_knowledge.instance_type = "waypoint";
				kenny_knowledge.instance_name = ss.str();
				
				knowledge_update_service.request.knowledge = kenny_knowledge;
				if (!update_knowledge_client.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the waypoint %s to the knowledge base.", ss.str().c_str());
					exit(-1);
				}
				ROS_INFO("KCL: (RPSquirrelRecursion) Added %s to the knowledge base.", ss.str().c_str());
				
				kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				kenny_knowledge.attribute_name = "near_for_dropping";
				kenny_knowledge.is_negative = false;
				diagnostic_msgs::KeyValue kv;
				kv.key = "wp1";
				kv.value = ss.str();
				kenny_knowledge.values.push_back(kv);
				kv.key = "wp2";
				kv.value = box_location_predicate;
				kenny_knowledge.values.push_back(kv);
				knowledge_update_service.request.knowledge = kenny_knowledge;
				if (!update_knowledge_client.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the fact (near %s %s) to the knowledge base.", ss.str().c_str(), box_location_predicate.c_str());
					exit(-1);
				}
				ROS_INFO("KCL: (RPSquirrelRecursion) Added (near %s %s) to the knowledge base.", ss.str().c_str(), box_location_predicate.c_str());
				kenny_knowledge.values.clear();

				near_box_location_mapping[box_location_predicate] = waypoints_near_box;
			}
			
			// Figure out which types of objects fit in each box.
			// (can_fit_inside ?t - type ?b - box)
			get_attribute.request.predicate_name = "can_fit_inside";
			if (!get_attribute_client.call(get_attribute)) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve the attributes of the predicate 'box_at'");
				return false;
			}
			
			std::map<std::string, std::string> box_to_type_mapping;
			for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = get_attribute.response.attributes.begin(); ci != get_attribute.response.attributes.end(); ++ci) {
				const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item = *ci;
				std::string box_predicate;
				std::string type_predicate;
				for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = knowledge_item.values.begin(); ci != knowledge_item.values.end(); ++ci) {
					const diagnostic_msgs::KeyValue& key_value = *ci;
					if ("b" == key_value.key) {
						box_predicate = key_value.value;
					} else if ("t" == key_value.key) {
						type_predicate = key_value.value;
					}
				}
				
				box_to_type_mapping[box_predicate] = type_predicate;
				type_to_box_pose_mapping[type_predicate] = box_to_pose_mapping[box_predicate];
			}
			
			// Get the location of kenny.
			// (robot_at ?v - robot ?wp - waypoint)
			get_attribute.request.predicate_name = "robot_at";
			if (!get_attribute_client.call(get_attribute)) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve the attributes of the predicate 'robot_at'");
				return false;
			}
			
			std::string robot_location;
			for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = get_attribute.response.attributes[0].values.begin(); ci != get_attribute.response.attributes[0].values.end(); ++ci) {
				const diagnostic_msgs::KeyValue& knowledge_item = *ci;
				
				ROS_INFO("KCL: (RPSquirrelRecursion) Process robot_at attribute: %s %s", knowledge_item.key.c_str(), knowledge_item.value.c_str());
				
				if ("wp" == knowledge_item.key) {
					robot_location = knowledge_item.value;
				}
			}
			
			if ("" == robot_location) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve the location of Kenny");
				return false;
			}
			
			ROS_INFO("KCL: (RPSquirrelRecursion) Kenny is at waypoint: %s", robot_location.c_str());
			
			// Get the location of the objects.
			// (object_at ?o - object ?wp - location)
			get_attribute.request.predicate_name = "object_at";
			if (!get_attribute_client.call(get_attribute)) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve the attributes of the predicate 'object_at'");
				return false;
			}
			
			// Create a mapping of each object to its location.
			std::map<std::string, std::string> object_to_location_mapping;
			std::map<std::string, std::vector<std::string> > grasping_waypoint_mappings;
			std::map<std::string, std::vector<std::string> > pushing_waypoint_mappings;
			std::map<std::string, geometry_msgs::Pose> object_to_pose_mapping;
			for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = get_attribute.response.attributes.begin(); ci != get_attribute.response.attributes.end(); ++ci) {
				const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item = *ci;
				std::string object_predicate;
				std::string object_location_predicate;
				for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = knowledge_item.values.begin(); ci != knowledge_item.values.end(); ++ci) {
					const diagnostic_msgs::KeyValue& key_value = *ci;
					if ("o" == key_value.key) {
						object_predicate = key_value.value;
					} else if ("wp" == key_value.key) {
						object_location_predicate = key_value.value;
					}
				}
				
				if (object_predicate != "" && object_location_predicate != "")
				{
					object_to_location_mapping[object_predicate] = object_location_predicate;
				}
				
				// Get the actual location of this object.
				std::vector< boost::shared_ptr<squirrel_object_perception_msgs::SceneObject> > results;
				if(message_store.queryNamed<squirrel_object_perception_msgs::SceneObject>(object_predicate, results))
				{
					if (results.size() < 1)
					{
						ROS_ERROR("KCL: (RPSquirrelRoadmap) aborting waypoint request; no matching obID %s", object_predicate.c_str());
						exit(-1);
					}
				} else {
					ROS_ERROR("KCL: (RPSquirrelRoadmap) could not query message store to fetch object pose");
					exit(-1);
				}
				const squirrel_object_perception_msgs::SceneObject &obj = *results[0];
				const geometry_msgs::Pose& obj_pose = obj.pose;
				object_to_pose_mapping[object_predicate] = obj_pose;
			}
			
			// Filter those objects that are already tidied.
			// (tidy ?o - object)
			get_attribute.request.predicate_name = "tidy";
			if (!get_attribute_client.call(get_attribute)) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve the attributes of the predicate 'tidy'");
				return false;
			}
			
			for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = get_attribute.response.attributes.begin(); ci != get_attribute.response.attributes.end(); ++ci) {
				const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item = *ci;
				for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = knowledge_item.values.begin(); ci != knowledge_item.values.end(); ++ci) {
					const diagnostic_msgs::KeyValue& key_value = *ci;
					if ("o" == key_value.key) {
						object_to_location_mapping.erase(key_value.value);
						break;
					}
				}
			}
			
			std_msgs::Int8 nr_untidied_objects;
			nr_untidied_objects.data = object_to_location_mapping.size();
			ROS_INFO("KCL: (RPSquirrelRecursion) Found %d untidied objects.", nr_untidied_objects.data);
			
			// Fetch the types of the untidied objects.
			// (is_of_type ?o - object ?t -type)
			get_attribute.request.predicate_name = "is_of_type";
			if (!get_attribute_client.call(get_attribute)) {
				ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve the attributes of the predicate 'is_of_type'");
				return false;
			}
			
			std::map<std::string, std::string> object_to_type_mapping;
			for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = get_attribute.response.attributes.begin(); ci != get_attribute.response.attributes.end(); ++ci) {
				const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item = *ci;
				std::string type_predicate;
				std::string object_predicate;
				for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = knowledge_item.values.begin(); ci != knowledge_item.values.end(); ++ci) {
					const diagnostic_msgs::KeyValue& key_value = *ci;
					if ("o" == key_value.key && object_to_location_mapping.count(key_value.value) == 1) {
						object_predicate = key_value.value;
					} else if ("t" == key_value.key) {
						type_predicate = key_value.value;
					}
				}
				
				if ("" != object_predicate)
				{
					object_to_type_mapping[object_predicate] = type_predicate;
					
					std::cout << " ************** MAP: " << object_predicate << " ->> " << type_predicate << std::endl;
					
					geometry_msgs::Pose obj_pose = object_to_pose_mapping[object_predicate];
					
					/**
					 * Create a waypoint for grasping.
					 */
					rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
					knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
					
					rosplan_knowledge_msgs::KnowledgeItem kenny_knowledge;
					kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
					std::stringstream ss;
					ss << "near_for_grasping_" << object_predicate;
					
					std::vector<std::string> grasping_waypoints;
					grasping_waypoints.push_back(ss.str());
					
					kenny_knowledge.instance_type = "waypoint";
					kenny_knowledge.instance_name = ss.str();
					
					knowledge_update_service.request.knowledge = kenny_knowledge;
					if (!update_knowledge_client.call(knowledge_update_service)) {
						ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the waypoint %s to the knowledge base.", ss.str().c_str());
						exit(-1);
					}
					ROS_INFO("KCL: (RPSquirrelRecursion) Added %s to the knowledge base.", ss.str().c_str());
					
					// Create a waypoint 43 cm from this box at a random angle.
					float angle = ((float)rand() / (float)RAND_MAX) * 360.0f;
					tf::Vector3 v(0.43f, 0.0f, 0.0f);
					v.rotate(tf::Vector3(0, 0, 1), angle);
					v += tf::Vector3(obj_pose.position.x, obj_pose.position.y, 0.0f);
					
					tf::Quaternion v_rotation(tf::Vector3(0, 0, 1), angle + 180.0f);
					
					// Store this location in the knowledge base.
					geometry_msgs::PoseStamped near_pose;
					near_pose.header.seq = 0;
					near_pose.header.stamp = ros::Time::now();
					near_pose.header.frame_id = "/map";
					near_pose.pose.position.x = v.x();
					near_pose.pose.position.y = v.y();
					near_pose.pose.position.z = 0.0f;
					
					near_pose.pose.orientation.x = v_rotation.x();
					near_pose.pose.orientation.y = v_rotation.y();
					near_pose.pose.orientation.z = v_rotation.z();
					near_pose.pose.orientation.w = v_rotation.w();
					
					std::string near_waypoint_mongodb_id(message_store.insertNamed(ss.str(), near_pose));
				
					kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
					kenny_knowledge.attribute_name = "near_for_grasping";
					kenny_knowledge.is_negative = false;
					diagnostic_msgs::KeyValue kv;
					kv.key = "wp1";
					kv.value = ss.str();
					kenny_knowledge.values.push_back(kv);
					kv.key = "wp2";
					kv.value = object_to_location_mapping[object_predicate];
					kenny_knowledge.values.push_back(kv);
					knowledge_update_service.request.knowledge = kenny_knowledge;
					if (!update_knowledge_client.call(knowledge_update_service)) {
						ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the fact (near %s %s) to the knowledge base.", ss.str().c_str(), object_predicate.c_str());
						exit(-1);
					}
					ROS_INFO("KCL: (RPSquirrelRecursion) Added (near %s %s) to the knowledge base.", ss.str().c_str(), object_predicate.c_str());
					kenny_knowledge.values.clear();

					grasping_waypoint_mappings[object_to_location_mapping[object_predicate]] = grasping_waypoints;
					
					/**
					 * Create a waypoing for pushing.
					 */
					ss.str(std::string());
					ss << "near_for_pushing_" << object_predicate;
					
					const geometry_msgs::Pose& box_pose = type_to_box_pose_mapping[type_predicate];
					
					tf::Vector3 object_pose_v3(obj_pose.position.x, obj_pose.position.y, obj_pose.position.z);
					tf::Vector3 box_pose_v3(box_pose.position.x, box_pose.position.y, box_pose.position.z); 
					tf::Vector3 behind_robot = object_pose_v3 - box_pose_v3;
					behind_robot.normalize();
					behind_robot = object_pose_v3 + behind_robot * 0.4f;
					
					// Make the angle such that is faces the object.
					angle = behind_robot.angle(object_pose_v3);
					
					// Create a waypoint 40 cm 'behind' the robot in relation to the box.
					tf::Quaternion behind_robot_rotation(tf::Vector3(0, 0, 1), angle);
					
					// Store this location in the knowledge base.
					//geometry_msgs::PoseStamped near_pose;
					near_pose.header.seq = 0;
					near_pose.header.stamp = ros::Time::now();
					near_pose.header.frame_id = "/map";
					near_pose.pose.position.x = behind_robot.x();
					near_pose.pose.position.y = behind_robot.y();
					near_pose.pose.position.z = 0.0f;
					
					near_pose.pose.orientation.x = behind_robot_rotation.x();
					near_pose.pose.orientation.y = behind_robot_rotation.y();
					near_pose.pose.orientation.z = behind_robot_rotation.z();
					near_pose.pose.orientation.w = behind_robot_rotation.w();
					
					std::string behind_robot_waypoint_mongodb_id(message_store.insertNamed(ss.str(), near_pose));
				
					std::vector<std::string> pushing_waypoints;
					pushing_waypoints.push_back(ss.str());
					
					kenny_knowledge.instance_type = "waypoint";
					kenny_knowledge.instance_name = ss.str();
					
					knowledge_update_service.request.knowledge = kenny_knowledge;
					if (!update_knowledge_client.call(knowledge_update_service)) {
						ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the waypoint %s to the knowledge base.", ss.str().c_str());
						exit(-1);
					}
					ROS_INFO("KCL: (RPSquirrelRecursion) Added %s to the knowledge base.", ss.str().c_str());
					
					kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
					kenny_knowledge.attribute_name = "near_for_grasping";
					kenny_knowledge.is_negative = false;

					kv.key = "wp1";
					kv.value = ss.str();
					kenny_knowledge.values.push_back(kv);
					kv.key = "wp2";
					kv.value = object_to_location_mapping[object_predicate];
					kenny_knowledge.values.push_back(kv);
					knowledge_update_service.request.knowledge = kenny_knowledge;
					if (!update_knowledge_client.call(knowledge_update_service)) {
						ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the fact (near %s %s) to the knowledge base.", ss.str().c_str(), object_predicate.c_str());
						exit(-1);
					}
					ROS_INFO("KCL: (RPSquirrelRecursion) Added (near %s %s) to the knowledge base.", ss.str().c_str(), object_predicate.c_str());
					kenny_knowledge.values.clear();

					pushing_waypoint_mappings[object_to_location_mapping[object_predicate]] = pushing_waypoints;
				}
			}
			
			// For all object that do not have a type (i.e. were not classified) we add the 'unknown' type.
			for (std::map<std::string, std::string>::const_iterator ci = object_to_location_mapping.begin(); ci != object_to_location_mapping.end(); ++ci)
			{
				const std::string& object_name = (*ci).first;
				
				std::cout << " ************** COULD NOT FIND TYPE OF: " << object_name << std::endl;
				if (object_to_type_mapping.find(object_name) == object_to_location_mapping.end())
				{
					object_to_type_mapping[object_name] = "unknown";
				}
			}
			
			ClassicalTidyPDDLGenerator::createPDDL(data_path, domain_name, problem_name, robot_location, object_to_location_mapping, grasping_waypoint_mappings, pushing_waypoint_mappings, object_to_type_mapping, box_to_location_mapping, box_to_type_mapping, near_box_location_mapping);
		} else {
			ROS_INFO("KCL: (RPSquirrelRecursion) Unable to create a domain for unknown action %s.", action_name.c_str());
			return false;
		}
		return true;
	}

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_RPSquirrelRecursion");
		ros::NodeHandle nh;

		// create PDDL action subscriber
		KCL_rosplan::RPSquirrelRecursion rpsr(nh);
		
		// Setup all the simulated actions.
		KCL_rosplan::ShedKnowledgePDDLAction shed_knowledge_action(nh);
		KCL_rosplan::FinaliseClassificationPDDLAction finalise_classify_action(nh);
		
		// listen for action dispatch
		ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPSquirrelRecursion::dispatchCallback, &rpsr);
		ROS_INFO("KCL: (RPSquirrelRecursion) Ready to receive");
		
		// Lets start the planning process.
		std::string data_path;
		nh.getParam("/data_path", data_path);
		
		std::string planner_path;
		nh.getParam("/planner_path", planner_path);
		
		std::stringstream ss;
		ss << data_path << "tidy_room_domain-nt.pddl";
		std::string domain_path = ss.str();
		
		ss.str(std::string());
		ss << data_path << "tidy_room_problem.pddl";
		std::string problem_path = ss.str();
		
		ss.str(std::string());
		ss << "timeout 10 " << planner_path << "ff -o DOMAIN -f PROBLEM";
		std::string planner_command = ss.str();
		
		rosplan_dispatch_msgs::PlanGoal psrv;
		psrv.domain_path = domain_path;
		psrv.problem_path = problem_path;
		psrv.data_path = data_path;
		psrv.planner_command = planner_command;
		psrv.start_action_id = 0;

		ROS_INFO("KCL: (RPSquirrelRecursion) Start plan action");
		actionlib::SimpleActionClient<rosplan_dispatch_msgs::PlanAction> plan_action_client("/kcl_rosplan/start_planning", true);

		plan_action_client.waitForServer();
		ROS_INFO("KCL: (RPSquirrelRecursion) Start planning server found");
		
		// send goal
		plan_action_client.sendGoal(psrv);
		ROS_INFO("KCL: (RPSquirrelRecursion) Goal sent");

		/*
		ros::ServiceClient run_planner_client = nh.serviceClient<rosplan_dispatch_msgs::PlanGoal>("/kcl_rosplan/planning_server");
		if (!run_planner_client.call(psrv))
		{
			ROS_ERROR("KCL: (TidyRoom) Failed to run the planning system.");
			exit(-1);
		}
		ROS_INFO("KCL: (TidyRoom) Planning system returned.");
		// Start the service ROSPlan will call when a domain and problem file needs to be generated.
		//ros::ServiceServer pddl_generation_service = nh.advertiseService("/kcl_rosplan/generate_planning_problem", &KCL_rosplan::RPSquirrelRecursion::generatePDDLProblemFile, &rpsr);
		*/

		ros::spin();
		return 0;
	}
	
