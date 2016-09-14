#include <std_msgs/Int8.h>

#include <map>
#include <algorithm>
#include <string>
#include <sstream>
#include <math.h>

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
#include <gazebo_msgs/SpawnModel.h>

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
		
		// Interface to spawn models in Gazebo.
		gazebo_spawn_model_client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
		
		std::string classifyTopic("/squirrel_perception_examine_waypoint");
		nh.param("squirrel_perception_classify_waypoint_service_topic", classifyTopic, classifyTopic);
		classify_object_waypoint_client = nh.serviceClient<squirrel_waypoint_msgs::ExamineWaypoint>(classifyTopic);
		
		pddl_generation_service = nh.advertiseService("/kcl_rosplan/generate_planning_problem", &KCL_rosplan::RPSquirrelRecursion::generatePDDLProblemFile, this);

		nh.getParam("/squirrel_interface_recursion/stop_when_enough_lumps_found", stop_when_enough_lumps_found);
		nh.getParam("/squirrel_interface_recursion/number_of_toys", number_of_toys);
		
		ROS_INFO("KCL: (RPSquirrelRecursion) Number of toys to find %d.", number_of_toys);
		if (stop_when_enough_lumps_found)
		{
			ROS_INFO("KCL: (RPSquirrelRecursion) Stop when enough toys are found!");
		}
		else
		{
			ROS_INFO("KCL: (RPSquirrelRecursion) Do NOT stop when enough toys are found!");
		}
		
		std::string occupancyTopic("/map");
		nh.param("occupancy_topic", occupancyTopic, occupancyTopic);
		view_cone_generator = new ViewConeGenerator(nh, occupancyTopic);
	}
	
	bool RPSquirrelRecursion::taskAchieved(const std::string& action_name) 
	{
		// Check if we have classified enough objects and/or identified enough 'lump' (depending).
		squirrel_object_perception_msgs::SceneObject lump;

		std::vector< boost::shared_ptr<squirrel_object_perception_msgs::SceneObject> > sceneObjects_results;
		message_store.query<squirrel_object_perception_msgs::SceneObject>(sceneObjects_results);

		//ROS_INFO("KCL: (RPSquirrelRecursion) Check if the task if achieved for the action %s.", action_name.c_str());
		
		unsigned int found_unexplored_lumps = 0;
		
		for (std::vector< boost::shared_ptr<squirrel_object_perception_msgs::SceneObject> >::const_iterator ci = sceneObjects_results.begin(); ci != sceneObjects_results.end(); ++ci)
		{
			const squirrel_object_perception_msgs::SceneObject& lump = **ci;
			const std::string& lump_name = lump.id;
			
			std::stringstream lump_wp_name_ss;
			lump_wp_name_ss << lump_name << "_observation_wp";
			
//			ROS_INFO("KCL: (RPSquirrelRecursion) Check if the lump waypoint %s has been explored.", lump_wp_name_ss.str().c_str());
			
			// Check if: 1) This waypoint has been observed; and 2) The lump is near an actual object.
			const geometry_msgs::Pose& pose = lump.pose;
			ToyState* matching_toy_state = NULL;
			
			for (std::vector<ToyState>::iterator ci = toy_locations.begin(); ci != toy_locations.end(); ++ci)
			{
				ToyState& toy_state = *ci;
				const geometry_msgs::Point& toy_location = toy_state.location_;
				geometry_msgs::Point delta;
				delta.x = pose.position.x - toy_location.x;
				delta.y = pose.position.y - toy_location.y;
				delta.z =  pose.position.z - toy_location.z;
				float distance = sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
				
//				ROS_INFO("KCL: (RPSquirrelRecursion) The lump is at (%f, %f, %f), existing toy is at (%f, %f, %f). Distance = %f.", pose.position.x, pose.position.y, pose.position.z, toy_location.x, toy_location.y, toy_location.z, distance);
				
				if (distance < 0.5f)
				{
					matching_toy_state = &toy_state;
					break;
				}
			}
			
//			if (matching_toy_state == NULL)
//			{
//				ROS_INFO("KCL: (RPSquirrelRecursion) Could not find an existing toy close enough.");
//			}
			
			// Check if this waypoint has been examined.
			rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			knowledge_item.attribute_name = "explored";
			
			diagnostic_msgs::KeyValue kv;
			kv.key = "wp";
			kv.value = lump_wp_name_ss.str();
			knowledge_item.values.push_back(kv);
			
			// Query the knowledge base.
			rosplan_knowledge_msgs::KnowledgeQueryService knowledge_query;
			knowledge_query.request.knowledge.push_back(knowledge_item);
			
			// Check if any of these facts are true.
			if (!query_knowledge_client.call(knowledge_query))
			{
//				ROS_ERROR("KCL: (RPSquirrelRecursion) Could not call the query knowledge server.");
				exit(1);
			}
			
			if (knowledge_query.response.all_true && matching_toy_state != NULL && !matching_toy_state->is_examined_)
			{
				matching_toy_state->is_examined_ = true;
				ROS_INFO("KCL: (RPSquirrelRecursion) The waypoint: %s has been explored and corresponds to the toy's location.", lump_wp_name_ss.str().c_str());
			}
			else if (knowledge_query.response.all_true)
			{
//				ROS_INFO("KCL: (RPSquirrelRecursion) The waypoint: %s has been explored, but does not correspond to a toy's location.", lump_wp_name_ss.str().c_str());
			}
			else
			{
//				ROS_INFO("KCL: (RPSquirrelRecursion) The waypoint: %s has not been explored, yet.", lump_wp_name_ss.str().c_str());
				++found_unexplored_lumps;
			}
		}
		
		// Objects that are classified have a type that is not 'unknown'.
		unsigned int classified_objects = 0;
		for (std::vector<ToyState>::const_iterator ci = toy_locations.begin(); ci != toy_locations.end(); ++ci)
		{
			const ToyState& toy_state = *ci;
			if (toy_state.is_examined_) ++classified_objects;
		}
		
		// Check if we are done.
		if (classified_objects == number_of_toys)
		{
			ROS_INFO("KCL: (RPSquirrelRecursion) All objects are classified.");
			return true;
		}
		else if ("explore_area" == action_name && found_unexplored_lumps == number_of_toys)
		{
			ROS_INFO("KCL: (RPSquirrelRecursion) We have found enough unidentified lumps to start the examination phase.");
			return true;
		}
		//ROS_INFO("KCL: (RPSquirrelRecursion) We have classified %d objects.", classified_objects);
		return false;
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
		if(//"observe-classifiable_on_attempt" != action_name &&
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
			//return;
		}
		
		planner_instance.startPlanner(domain_name, problem_name, data_path, planner_command);
		
		// publish feedback (enabled)
		rosplan_dispatch_msgs::ActionFeedback fb;
		fb.action_id = msg->action_id;
		fb.status = "action enabled";
		action_feedback_pub.publish(fb);

		// wait for action to finish
		ros::Rate loop_rate(1);
		bool completed_task = false;
		while (ros::ok() && (planner_instance.getState() == actionlib::SimpleClientGoalState::ACTIVE || planner_instance.getState() == actionlib::SimpleClientGoalState::PENDING)) {
			ros::spinOnce();
			loop_rate.sleep();
			
			if (taskAchieved(action_name))
			{
				completed_task = true;
				if ("explore_area" == action_name)
				{
					planner_instance.stopPlanner();
					break;
				}
				else if ("examine_area" == action_name)
				{
					planner_instance.stopPlanner();
					exit(0);
				}
			}
		}

		actionlib::SimpleClientGoalState state = planner_instance.getState();
		ROS_INFO("KCL: (RPSquirrelRecursion) action finished: %s, %s", action_name.c_str(), state.toString().c_str());

		if(state == actionlib::SimpleClientGoalState::SUCCEEDED || completed_task)
		{
			// First of all check if enough lumps were found, or if enough objects were classified.
			
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
		if (last_received_msg.empty())
		{
			ROS_INFO("KCL: (RPSquirrelRecursion) Create the initial problem.");
			
			std::stringstream domain_ss;
			domain_ss << data_path << "tidy_room_domain-nt.pddl";
			std::string domain_name = domain_ss.str();
			
			if (!initial_problem_generated)
			{
				generateInitialState();
			}
			
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
		/*
		// Remove all previous goals.
		rosplan_knowledge_msgs::GetAttributeService attribute_service;
		attribute_service.request.predicate_name.data = "explored";
		if (!get_attribute_client.call(attribute_service))
		{
			ROS_ERROR("KCL: (RPSquirrelRecursion) Unable to call the attribute service.");
			exit(-1);
		}
		
		for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = attribute_service.response.attributes.begin(); ci != attribute_service.reponse.attributes.end(); ++ci)
		{
			const rosplan_knowledge_msgs::KnowledgeItem& 
			rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
			rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::REMOVE_GOAL;
			knowledge_item.attribute_name = "explored";
			
			diagnostic_msgs::KeyValue kv;
			kv.key = "wp";
			kv.value = lump_wp_name_ss.str();
			knowledge_item.values.push_back(kv);
		}
		*/
		
		rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
		rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
		
		// Add kenny
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
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

		
		bool spawn_objects = false;
		std::string model_file_name;
		node_handle->getParam("/squirrel_interface_recursion/spawn_objects", spawn_objects);
		node_handle->getParam("/squirrel_interface_recursion/model_file_name", model_file_name);
		
		// Don't place objects in Gazebo for a sencond time.
		if (initial_problem_generated || !spawn_objects) return;
		
		
		// Place objects in the designated possible boxes.
		std::vector<std::pair<float, float> > object_placement_bounding_boxes;
		object_placement_bounding_boxes.push_back(std::make_pair(-1.82, -5.65));
		object_placement_bounding_boxes.push_back(std::make_pair(5.86, -9.47));
		object_placement_bounding_boxes.push_back(std::make_pair(-1.6, -2.23));
		object_placement_bounding_boxes.push_back(std::make_pair(0.617, -4.49));
		
		// Read in the model.
		//std::string model_file_name("/home/bram/.gazebo/models/cardboard_box/model.sdf");
		std::ifstream model_file(model_file_name.c_str());
		std::stringstream model_xml_ss;
		if (model_file.is_open())
		{
			std::string line;
			while (getline(model_file, line))
			{
				model_xml_ss << line << std::endl;
			}
		}
		else
		{
			ROS_ERROR("KCL: (RPSquirrelRecursion) Could not open %s.", model_file_name.c_str());
			exit(-1);
		}
		
		// Spawn a certain number of objects in Gazebo.
		for (int i = 0; i < number_of_toys; ++i)
		{
			// Find a pose.
			int object_placement_bounding_box_index = rand() % (object_placement_bounding_boxes.size() / 2);
			float min_x = std::min(object_placement_bounding_boxes[object_placement_bounding_box_index * 2].first, 
			                       object_placement_bounding_boxes[object_placement_bounding_box_index * 2 + 1].first);
			float max_x = std::max(object_placement_bounding_boxes[object_placement_bounding_box_index * 2].first, 
			                       object_placement_bounding_boxes[object_placement_bounding_box_index * 2 + 1].first);
			float min_y = std::min(object_placement_bounding_boxes[object_placement_bounding_box_index * 2].second, 
			                       object_placement_bounding_boxes[object_placement_bounding_box_index * 2 + 1].second);
			float max_y = std::max(object_placement_bounding_boxes[object_placement_bounding_box_index * 2].second, 
			                       object_placement_bounding_boxes[object_placement_bounding_box_index * 2 + 1].second);
			
			float delta_x = (float)rand() / (float)RAND_MAX;
			float delta_y = (float)rand() / (float)RAND_MAX;
			
			geometry_msgs::Pose model_pose;
			model_pose.orientation.x = 0;
			model_pose.orientation.y = 0;
			model_pose.orientation.z = 0;
			model_pose.orientation.w = 1;
			
			model_pose.position.x = (max_x - min_x) * delta_x + min_x;
			model_pose.position.y = (max_y - min_y) * delta_y + min_y;
			model_pose.position.z = 0.01f;
			
			std::stringstream ss_object_name;
			ss_object_name << "Box" << i;
			
			gazebo_msgs::SpawnModel spawn_model;
			spawn_model.request.model_name = ss_object_name.str().c_str();
			spawn_model.request.model_xml = model_xml_ss.str();
			spawn_model.request.robot_namespace = "";
			spawn_model.request.initial_pose = model_pose;
			spawn_model.request.reference_frame = "/map";
			if (!gazebo_spawn_model_client.call(spawn_model) || !spawn_model.response.success)
			{
				ROS_ERROR("KCL: (RPSquirrelRecursion) Unable to spawn object in Gazebo.");
				std::cout << model_xml_ss.str() << std::endl;
				exit(-1);
			}
			
			toy_locations.push_back(model_pose.position);
			
			ROS_INFO("KCL: (RPSquirrelRecursion) Object spawned!");
		}
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
			view_cone_generator->createViewCones(view_poses, bounding_box, 2, 5, 0.94f, 4.0f, 100, 1.0f);
			
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
			
			// Delete all the previous viewpoines.
			ROS_INFO("KCL: (RPSquirrelRecursion) Remove all previous waypoints.");
			rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
			
			// Fetch all the discovered objects.
			rosplan_knowledge_msgs::GetAttributeService get_attribute;
			get_attribute.request.predicate_name = "object_at";
			if (!get_attribute_client.call(get_attribute)) {
				ROS_ERROR("KCL: (RPSquirrelRecursion) Failed to recieve the attributes of the predicate 'object_at'");
				return false;
			}
			
			for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = get_attribute.response.attributes.begin(); ci != get_attribute.response.attributes.end(); ++ci) {
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
				
				ROS_INFO("KCL: (RPSquirrelRecursion) Object location is: %s", location_predicate.c_str());
				
				// Get the actual location of this object.
				std::vector<boost::shared_ptr<geometry_msgs::PoseStamped> > location_locations;
				if (message_store.queryNamed<geometry_msgs::PoseStamped>(location_predicate, location_locations))
				{
					for (std::vector<boost::shared_ptr<geometry_msgs::PoseStamped> >::const_iterator ci = location_locations.begin(); ci != location_locations.end(); ++ci)
					{
						//ROS_INFO("KCL: (RPSquirrelRecursion) Found the location of 
						std::cout << "******** Found the location of " << location_predicate << ": (" << (*ci)->pose.position.x << ", " << (*ci)->pose.position.y << ", " << (*ci)->pose.position.z << ")" << std::endl;
					}
				}
				else
				{
					ROS_ERROR("KCL: (RPSquirrelRoadmap) could not query message store to fetch object pose");
					return false;
				}
				
				// Find locations from where we can observe
				squirrel_waypoint_msgs::ExamineWaypoint getTaskPose;
				
				// fetch position of object from message store
				squirrel_object_perception_msgs::SceneObject lump;

				std::vector< boost::shared_ptr<squirrel_object_perception_msgs::SceneObject> > sceneObjects_results;
				message_store.query<squirrel_object_perception_msgs::SceneObject>(sceneObjects_results);

				ROS_INFO("KCL: (RPSquirrelRecursion) Number of objects found: %zu", sceneObjects_results.size());

				// Request a classification waypoint for each object.
				for (std::vector<boost::shared_ptr<squirrel_object_perception_msgs::SceneObject> >::const_iterator ci = sceneObjects_results.begin(); ci != sceneObjects_results.end(); ++ci)
				{
					const squirrel_object_perception_msgs::SceneObject& obj = **ci;
					getTaskPose.request.object_pose.header = obj.header;
					getTaskPose.request.object_pose.pose = obj.pose;
					if (!classify_object_waypoint_client.call(getTaskPose)) {
						ROS_ERROR("KCL: (RPSquirrelRoadmap) Failed to recieve classification waypoints for %s.", object_predicate.c_str());
						return false;
					}
	
					std_msgs::Int8 debug_pose_number;
					debug_pose_number.data = getTaskPose.response.poses.size();
//					ROS_INFO("KCL: (RPSquirrelRecursion) Found %d observation poses", debug_pose_number.data);

					// Add all the waypoints to the knowledge base.
					std::stringstream ss;
					std::vector<std::string> observation_location_predicates;
					for(int i=0;i<getTaskPose.response.poses.size(); i++) {
					
						ss.str(std::string());
						ss << object_predicate << "_observation_wp";

						geometry_msgs::PoseStamped pose;
						pose.header.seq = 0;
						pose.header.frame_id = "/map";
						pose.header.stamp = ros::Time::now();
						pose.pose = getTaskPose.response.poses[i].pose.pose;
						
						// Check if this location is suitable.
						if (view_cone_generator->isBlocked(pose.pose.position, 0.45f))
						{
//							ROS_INFO("KCL: (RPSquirrelRecursion) Ignore: (%f, %f, %f)", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
							continue;
						}
						
						std::string id(message_store.insertNamed(ss.str(), pose));
					
//						ROS_INFO("KCL: (RPSquirrelRecursion) Process observation pose: %s", ss.str().c_str());
					
//						std::cout << "KCL: (RPSquirrelRecursion) Lump location: (" << obj.pose.position.x << ", " << obj.pose.position.y << ", " << obj.pose.position.z << "); Observation location: (" << pose.pose.position.x << ", " << pose.pose.position.y << ", " << pose.pose.position.z << ")" << std::endl;
						
						rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
						updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
						updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
						updateSrv.request.knowledge.instance_type = "waypoint";
						updateSrv.request.knowledge.instance_name = ss.str();
						update_knowledge_client.call(updateSrv);
					
						observation_location_predicates.push_back(ss.str());
					
						// Setup the goal.
						updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
						updateSrv.request.knowledge.attribute_name = "explored";
						updateSrv.request.knowledge.is_negative = false;
						diagnostic_msgs::KeyValue kv;
						kv.key = "wp";
						kv.value = ss.str();
						updateSrv.request.knowledge.values.push_back(kv);
					
						// Add the goal.
						updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
						//updateSrv.request.knowledge = update_knowledge_client;
						if (!update_knowledge_client.call(updateSrv)) {
							ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the goal (explored %s) to the knowledge base.", ss.str().c_str());
							exit(-1);
						}
//						ROS_INFO("KCL: (RPSquirrelRecursion) Added the goal (explored %s) to the knowledge base.", ss.str().c_str());
						
						// only consider a single waypoint.
						break;
					}
				}
			}
			
			// add initial state (robot_at)
			rosplan_knowledge_msgs::KnowledgeItem waypoint_knowledge;
			updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			waypoint_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			waypoint_knowledge.instance_type = "waypoint";
			waypoint_knowledge.instance_name = "kenny_waypoint";
			updateSrv.request.knowledge = waypoint_knowledge;
			if (!update_knowledge_client.call(updateSrv)) {
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
			updateSrv.request.knowledge = waypoint_knowledge;
			if (!update_knowledge_client.call(updateSrv)) {
				ROS_ERROR("KCL: (TidyRooms) Could not add the fact (robot_at kenny room) to the knowledge base.");
				exit(-1);
			}
//			ROS_INFO("KCL: (TidyRooms) Added (robot_at kenny room) to the knowledge base.");
			waypoint_knowledge.values.clear();
			
			PlanningEnvironment planning_environment;
			planning_environment.parseDomain(domain_path);
			planning_environment.update(*node_handle);
			PDDLProblemGenerator pddl_problem_generator;
			
			pddl_problem_generator.generatePDDLProblemFile(planning_environment, problem_path);
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
	
