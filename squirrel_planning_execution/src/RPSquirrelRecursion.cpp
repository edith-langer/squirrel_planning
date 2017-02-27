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
#include "pddl_actions/GotoWaypointWrapper.h"

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <gazebo_msgs/SpawnModel.h>

/* The implementation of RPSquirrelRecursion.h */
namespace KCL_rosplan {

	
	TaskStateMonitor::TaskStateMonitor(ros::NodeHandle& nh, const BoundingBox& bb, const ViewConeGenerator& vcg, mongodb_store::MessageStoreProxy& ms)
		: node_handle(&nh), bounding_box(&bb), view_cone_generator(&vcg), message_store(&ms), number_of_toys_to_find(0), enough_unexplored_lumps_found(false), is_complete(false)
	{
		query_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>("/kcl_rosplan/query_knowledge_base");
		
		// Interface to spawn models in Gazebo.
		gazebo_spawn_model_client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
		
		// Read paramters that determine how to gauge overall state.
		nh.getParam("/squirrel_interface_recursion/number_of_toys", number_of_toys_to_find);
		ROS_INFO("KCL: (RPSquirrelRecursion) Number of toys to find %d.", number_of_toys_to_find);
	}
	
	void TaskStateMonitor::initialiseToys()
	{
		// Check wheter toys need to be spawned in Gazebo.
		bool spawn_objects = false;
		node_handle->getParam("/squirrel_interface_recursion/spawn_objects", spawn_objects);

		if (spawn_objects)
			std::cout << "----- Spawn objects ! -----" << std::endl;
		
		// Read the file containing the model (if we need to spawn them).
		std::string model_file_name;
		std::stringstream model_xml_ss;
		
		if (spawn_objects)
		{
			node_handle->getParam("/squirrel_interface_recursion/model_file_name", model_file_name);
			
			// Read in the model.
			std::ifstream model_file(model_file_name.c_str());
			
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
		}

		std::cout << "Model name: " << model_file_name << std::endl;
		std::cout << "Number of toys: " << number_of_toys_to_find << std::endl;
		
		// Read the poses of the toys from the launch file.
		std::vector<geometry_msgs::Pose> toy_poses;
		while (toy_poses.size() < number_of_toys_to_find)
		{
			std::stringstream bb_name;
			bb_name << "/squirrel_interface_recursion/toy_p";
			bb_name << toy_poses.size();
			
			if (!node_handle->hasParam(bb_name.str()))
			{
				ROS_INFO("KCL: (RPSquirrelRecursion) Could not find the parameter %s, if more toys need to be spawned they will be spawned at random locations.", bb_name.str().c_str());
				break;
			}
			
			std::string coordinate;
			node_handle->getParam(bb_name.str(), coordinate);
			std::vector<std::string> elements;
			split(coordinate, ',', elements);
			
			if (elements.size() != 3)
			{
				ROS_ERROR("KCL: (RPSquirrelRecursion) Misformatted coordinate for toy %s. Expected format (x,y,z)", coordinate.c_str());
				exit (-1);
			}
			
			geometry_msgs::Pose model_pose;
			model_pose.orientation.x = 0;
			model_pose.orientation.y = 0;
			model_pose.orientation.z = 0;
			model_pose.orientation.w = 1;
			
			model_pose.position.x = ::atof(elements[0].c_str());
			model_pose.position.y = ::atof(elements[1].c_str());
			model_pose.position.z = ::atof(elements[2].c_str());
			
			ROS_INFO("KCL: (RPSquirrelRecursion) Found toy coordinate (%f, %f, %f)", ::atof(elements[0].c_str()), ::atof(elements[1].c_str()), ::atof(elements[2].c_str()));
			toy_poses.push_back(model_pose);
		}
		
		// If the number of toys is more than the provided position we randomise the remaining locations.
		while (toy_poses.size() < number_of_toys_to_find)
		{
			// Find a pose.
			tf::Vector3 toy_location;
			
			bool valid_location = false;
			while (!valid_location)
			{
				toy_location = bounding_box->createPoint();
				geometry_msgs::Point p;
				p.x = toy_location.x();
				p.y = toy_location.y();
				p.z = 0;
				valid_location = !view_cone_generator->isBlocked(p, 1.0f);
				
				// Check if this object is too close to another object.
				for (unsigned int i = 0; i < toy_poses.size(); ++i)
				{
					const geometry_msgs::Pose& model_pose = toy_poses[i];
					float distance = sqrt((model_pose.position.x - toy_location.x()) * (model_pose.position.x - toy_location.x()) + (model_pose.position.y - toy_location.y()) * (model_pose.position.y - toy_location.y()));
					if (distance < 0.5f) valid_location = false;
				}
			}
			
			geometry_msgs::Pose model_pose;
			model_pose.orientation.x = 0;
			model_pose.orientation.y = 0;
			model_pose.orientation.z = 0;
			model_pose.orientation.w = 1;
			
			model_pose.position.x = toy_location.x();
			model_pose.position.y = toy_location.y();
			model_pose.position.z = 0.01f;
			
			ROS_INFO("KCL: (RPSquirrelRecursion) Random sampled toy coordinate (%f, %f, %f)", model_pose.position.x, model_pose.position.y, model_pose.position.z);
			
			toy_poses.push_back(model_pose);
		}
		
		// We initialise the state of each toy and spawn it in Gazebo if necessary.
		for (unsigned int i = 0; i < toy_poses.size(); ++i)
		{
			const geometry_msgs::Pose& model_pose = toy_poses[i];
			std::stringstream ss_object_name;
			ss_object_name << "Box" << i;
			
			if (spawn_objects)
			{
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
				ROS_INFO("KCL: (RPSquirrelRecursion) Object spawned at (%f, %f, %f)!", model_pose.position.x, model_pose.position.y, model_pose.position.z);
			}
			ToyState toy_state(model_pose.position, ss_object_name.str());
			toy_locations.push_back(toy_state);
		}
	}
	
	void TaskStateMonitor::updateState()
	{
		// Check if we have classified all toys and if we have identified enough unexamined 'lumps'.
		squirrel_object_perception_msgs::SceneObject lump;

		std::vector< boost::shared_ptr<squirrel_object_perception_msgs::SceneObject> > sceneObjects_results;
		message_store->query<squirrel_object_perception_msgs::SceneObject>(sceneObjects_results);

		unsigned int found_unexplored_lumps = 0;
		
		for (std::vector< boost::shared_ptr<squirrel_object_perception_msgs::SceneObject> >::const_iterator ci = sceneObjects_results.begin(); ci != sceneObjects_results.end(); ++ci)
		{
			const squirrel_object_perception_msgs::SceneObject& lump = **ci;
			const std::string& lump_name = lump.id;
			
			std::stringstream lump_wp_name_ss;
			lump_wp_name_ss << lump_name << "_observation_wp";
			
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
				
				if (distance < 0.5f)
				{
					matching_toy_state = &toy_state;
					break;
				}
			}
			
			// Check if this object has been examined.
			rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			knowledge_item.attribute_name = "examined";
			
			diagnostic_msgs::KeyValue kv;
			kv.key = "o";
			kv.value = lump_name;
			knowledge_item.values.push_back(kv);
			
			// Query the knowledge base.
			rosplan_knowledge_msgs::KnowledgeQueryService knowledge_query;
			knowledge_query.request.knowledge.push_back(knowledge_item);
			
			// Check if any of these facts are true.
			if (!query_knowledge_client.call(knowledge_query))
			{
				ROS_ERROR("KCL: (RPSquirrelRecursion) Could not call the query knowledge server.");
				exit(1);
			}
			
			// Negative option.
			knowledge_item.is_negative = true;
			
			// Query the knowledge base.
			rosplan_knowledge_msgs::KnowledgeQueryService negative_knowledge_query;
			negative_knowledge_query.request.knowledge.push_back(knowledge_item);
			
			// Check if any of these facts are false.
			if (!query_knowledge_client.call(negative_knowledge_query))
			{
				ROS_ERROR("KCL: (RPSquirrelRecursion) Could not call the query knowledge server.");
				exit(1);
			}
			
			
			if (knowledge_query.response.all_true && matching_toy_state != NULL && !matching_toy_state->is_examined_)
			{
				matching_toy_state->setExamined();
				ROS_INFO("KCL: (RPSquirrelRecursion) The waypoint: %s has been explored and corresponds to the toy's location.", lump_wp_name_ss.str().c_str());
			}
			else if (knowledge_query.response.all_true)
			{
//				ROS_INFO("KCL: (RPSquirrelRecursion) The waypoint: %s has been explored, but does not correspond to a toy's location.", lump_wp_name_ss.str().c_str());
			}
			else if (!negative_knowledge_query.response.all_true)
			{
//				ROS_INFO("KCL: (RPSquirrelRecursion) The waypoint: %s has not been explored, yet.", lump_wp_name_ss.str().c_str());
				++found_unexplored_lumps;
			}
			else
			{
				// False positive!
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
		if (classified_objects >= number_of_toys_to_find)
		{
			ROS_INFO("KCL: (RPSquirrelRecursion) All objects are classified.");
			is_complete = true;
		}
		else
		{
			is_complete = false;
		}
		
		if (found_unexplored_lumps >= (number_of_toys_to_find - classified_objects))
		{
			//ROS_INFO("KCL: (RPSquirrelRecursion) We have found enough unidentified lumps to start the examination phase.");
			enough_unexplored_lumps_found = true;
		}
		else
		{
			enough_unexplored_lumps_found = false;
		}
	}
	
	BoundingBox::BoundingBox(ros::NodeHandle& nh)
	{
		ROS_INFO("KCL: (RPSquirrelRecursion) Setup bounding box.");
		unsigned int bounding_box_param_index = 0;
		while (true)
		{
			std::stringstream bb_name;
			bb_name << "/squirrel_interface_recursion/viewcone_bounding_box_p";
			bb_name << bounding_box_param_index;
			
			if (!nh.hasParam(bb_name.str()))
			{
				ROS_INFO("KCL: (RPSquirrelRecursion) Could not find the parameter %s, bounding box complete.", bb_name.str().c_str());
				break;
			}
			
			std::string coordinate;
			nh.getParam(bb_name.str(), coordinate);
			std::vector<std::string> elements;
			split(coordinate, ',', elements);
			
			if (elements.size() != 3)
			{
				ROS_ERROR("KCL: (RPSquirrelRecursion) Misformatted coordinate for bounding box %s. Expected format (x,y,z)", coordinate.c_str());
				exit (-1);
			}
			
			bounding_box.push_back(tf::Vector3(::atof(elements[0].c_str()), ::atof(elements[1].c_str()), ::atof(elements[2].c_str())));
			
			ROS_INFO("KCL: (RPSquirrelRecursion) Found bounding box coordinate (%f, %f, %f)", ::atof(elements[0].c_str()), ::atof(elements[1].c_str()), ::atof(elements[2].c_str()));
			
			++bounding_box_param_index;
		}
	}
	
	bool BoundingBox::isInside(const tf::Vector3& v) const
	{
		char sign = 0;
		for (int i = 0; i < bounding_box.size(); ++i)
		{
			const tf::Vector3& v1 = bounding_box[i];
			const tf::Vector3& v2 = bounding_box[(i + 1) % bounding_box.size()];
			
			tf::Vector3 cross_product = (v - v1).cross(v2 - v1);
			
			if (sign == 0)
			{
				sign = cross_product.z() > 0 ? 1 : -1;
			}
			else
			{
				if (sign == -1 && cross_product.z() > 0 ||
				    sign == 1 && cross_product.z() < 0)
				{
					return false;
				}
			}
		}
		return true;
	}
	
	tf::Vector3 BoundingBox::createPoint() const
	{
		float min_x = std::numeric_limits<float>::max();
		float max_x = -std::numeric_limits<float>::max();
		float min_y = std::numeric_limits<float>::max();
		float max_y = -std::numeric_limits<float>::max();
		
		for (std::vector<tf::Vector3>::const_iterator ci = bounding_box.begin(); ci != bounding_box.end(); ++ci)
		{
			const tf::Vector3 v = *ci;
			if (v.x() < min_x) min_x = v.x();
			if (v.x() > max_x) max_x = v.x();
			if (v.y() < min_y) min_y = v.y();
			if (v.y() > max_y) max_y = v.y();
		}
		
		while (true)
		{
			// Find a pose.
			float x = (float)rand() / (float)RAND_MAX * (max_x - min_x) + min_x;
			float y = (float)rand() / (float)RAND_MAX * (max_y - min_y) + min_y;
			
			tf::Vector3 v(x, y, 0);
			if (isInside(v)) return v;
		}
		return tf::Vector3(0, 0, 0);
	}

	/*-------------*/
	/* constructor */
	/*-------------*/

	RPSquirrelRecursion::RPSquirrelRecursion(ros::NodeHandle &nh)
		: node_handle(&nh), message_store(nh), initial_problem_generated(false), waypoint_number(0), number_of_segmentation_actions(0)
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

		
		
		std::string occupancyTopic("/map");
		nh.param("occupancy_topic", occupancyTopic, occupancyTopic);
		view_cone_generator = new ViewConeGenerator(nh, occupancyTopic);
		
		bounding_box = new BoundingBox(*node_handle);
		
		task_state_monitor = new TaskStateMonitor(nh, *bounding_box, *view_cone_generator, message_store);
		
		start_time = ros::Time::now();
	}
	
	/*---------------------------*/
	/* strategic action callback */
	/*---------------------------*/

	void RPSquirrelRecursion::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		if (task_state_monitor->isComplete()) {
			std::cout << " ================== PLANNING COMPLETE! ==================" << std::endl;
				
			for (std::vector<ToyState>::const_iterator ci = task_state_monitor->getToyLocations().begin(); ci != task_state_monitor->getToyLocations().end(); ++ci)
			{
				const ToyState& toy_state = *ci;
				std::cout << "Time to find: " << toy_state.name_ << " " << toy_state.time_stamp_.toSec() - start_time.toSec() << std::endl;
			}
			std::cout << "Total time: " << ros::Time::now().toSec() - start_time.toSec() << std::endl;
			std::cout << "Number of segmentation actions: " << number_of_segmentation_actions << "; Success: " << task_state_monitor->getNumberOfToysToFind() << "; Fails: " << number_of_segmentation_actions - task_state_monitor->getNumberOfToysToFind() << "; " << (number_of_segmentation_actions == 0 ? 0 : ((float)task_state_monitor->getNumberOfToysToFind() / (float)number_of_segmentation_actions) * 100.0f) << "%" << std::endl;
			ros::shutdown();
			exit(0);
		}
	
		rosplan_dispatch_msgs::ActionDispatch normalised_action_dispatch = *msg;
		std::string action_name = msg->name;
		std::transform(action_name.begin(), action_name.end(), action_name.begin(), tolower);
		normalised_action_dispatch.name = action_name;
		
		ROS_INFO("KCL: (RPSquirrelRecursion) Action received %s", msg->name.c_str());
		
		// Dirty trick to trigger a replan.
		if ("tidy_area" == action_name)
		{
			last_received_msg.clear();
			
			// publish feedback (enabled)
			rosplan_dispatch_msgs::ActionFeedback fb;
			fb.action_id = msg->action_id;
			fb.status = "action enabled";
			action_feedback_pub.publish(fb);
			
			// publish feedback (failed)
			fb.action_id = msg->action_id;
			fb.status = "action failed";
			action_feedback_pub.publish(fb);
			return;
		}
		
		if ("examine_object" == action_name)// || "explore_waypoint" == action_name)
		{
			++number_of_segmentation_actions;
		}
		
		
		// ignore actions
		if("examine_area" != action_name &&
		   "explore_area" != action_name)
		{
			return;
		}

		bool actionAchieved = false;
		last_received_msg.push_back(normalised_action_dispatch);
		
		ROS_INFO("KCL: (RPSquirrelRecursion) action recieved %s", action_name.c_str());
		
		PlannerInstance& planner_instance = PlannerInstance::createInstance(*node_handle);
		
		// Start the planning process.
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
		
		// Before calling the planner we create the domain so it can be parsed. We also insert all the relevant facts in the knowledge base 
		// to construct the planning problem.
		if (!createDomain(action_name))
		{
			ROS_ERROR("KCL: (RPSquirrelRecursion) failed to produce a domain at %s for action name %s.", domain_name.c_str(), action_name.c_str());
			exit(-1);
		}
		
		planner_instance.startPlanner(domain_name, problem_name, data_path, planner_command);
		
		// publish feedback (enabled)
		rosplan_dispatch_msgs::ActionFeedback fb;
		fb.action_id = msg->action_id;
		fb.status = "action enabled";
		action_feedback_pub.publish(fb);

		// wait for plan to finish.
		ros::Rate loop_rate(1);
		while (ros::ok() && (planner_instance.getState() == actionlib::SimpleClientGoalState::ACTIVE || planner_instance.getState() == actionlib::SimpleClientGoalState::PENDING)) {
			ros::spinOnce();
			loop_rate.sleep();
			
			// Check if the task (or part thereof) has been achieved.
			task_state_monitor->updateState();
			
			// We have completed the task.
			if (task_state_monitor->isComplete()) {
				planner_instance.stopPlanner();
				
				std::cout << " ================== PLANNING COMPLETE! ==================" << std::endl;
				
				for (std::vector<ToyState>::const_iterator ci = task_state_monitor->getToyLocations().begin(); ci != task_state_monitor->getToyLocations().end(); ++ci)
				{
					const ToyState& toy_state = *ci;
					std::cout << "Time to find: " << toy_state.name_ << " " << toy_state.time_stamp_.toSec() - start_time.toSec() << std::endl;
				}
				std::cout << "Total time: " << ros::Time::now().toSec() - start_time.toSec() << std::endl;
				std::cout << "Number of segmentation actions: " << number_of_segmentation_actions << "; Success: " << task_state_monitor->getNumberOfToysToFind() << "; Fails: " << number_of_segmentation_actions - task_state_monitor->getNumberOfToysToFind() << "; " << (number_of_segmentation_actions == 0 ? 0 : ((float)task_state_monitor->getNumberOfToysToFind() / (float)number_of_segmentation_actions) * 100.0f) << "%" << std::endl;
				ros::shutdown();
				exit(0);
			}
			
			// We found enough lumps, we can now move to the next phase which is to examine the found lumps.
			if (task_state_monitor->enoughLumpsFound() && "explore_area" == action_name)
			{
				std::cout << " ================== ENOUGH LUMPS FOUND! ==================" << std::endl;
				std::cout << "Found enough lumps, move to segmenting them." << std::endl;
				planner_instance.stopPlanner();
				break;
			}
		}

		actionlib::SimpleClientGoalState state = planner_instance.getState();
		ROS_INFO("KCL: (RPSquirrelRecursion) action finished: %s, %s", action_name.c_str(), state.toString().c_str());

		// Check if we achieved the (sub-)task at the end of execution. If not, we trigger a replan. We alter 
		// the knoweldge base such that we start exploring the room given new viewcones.
		task_state_monitor->updateState();
		if ((!task_state_monitor->enoughLumpsFound() && "explore_area" == action_name) ||
		    (!task_state_monitor->enoughLumpsFound() && !task_state_monitor->isComplete()))	
		{
			last_received_msg.clear();
			
			// Remove the predicate (explored area), otherwise the replanning would fail.
			if (msg->parameters.size() < 2)
			{
				std::stringstream ss;
				ss << "(" << action_name;
				for (unsigned int i = 0; i < msg->parameters.size(); ++i)
				{
					ss << " " << msg->parameters[i];
				}
				ss << ")";
				ROS_ERROR("KCL: (RPSquirrelRecursion) The action does not have a 2nd parameter (area).");
				ROS_ERROR("KCL: (RPSquirrelRecursion) %s.", ss.str().c_str());
				exit(-1);
			}
			const std::string& area = msg->parameters[1].value;
			
			// Remove the facts that the area is explored.
			rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
			knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
			rosplan_knowledge_msgs::KnowledgeItem kenny_knowledge;
			kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			kenny_knowledge.attribute_name = "explored";
			kenny_knowledge.is_negative = false;
			
			diagnostic_msgs::KeyValue kv;
			kv.key = "a";
			kv.value = msg->parameters[1].value;
			kenny_knowledge.values.push_back(kv);
			
			knowledge_update_service.request.knowledge = kenny_knowledge;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (RPSquirrelRecursion) Could not remove the (explored %s) predicate from the knowledge base.", area.c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (RPSquirrelRecursion) Removed the (explored %s) predicate from the knowledge base.", area.c_str());
			kenny_knowledge.values.clear();
			
			// Remove all previous explored waypoints.
			rosplan_knowledge_msgs::GetAttributeService attribute_service;
			
			// Reset kenny's waypoint.
			attribute_service.request.predicate_name = "robot_at";
			if (!get_attribute_client.call(attribute_service))
			{
				ROS_ERROR("KCL: (RPSquirrelRecursion) Unable to call the attribute service.");
				exit(-1);
			}
			
			for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = attribute_service.response.attributes.begin(); ci != attribute_service.response.attributes.end(); ++ci)
			{
				const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item = *ci;
				rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
				knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
				knowledge_update_service.request.knowledge = knowledge_item;
				if (!update_knowledge_client.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (RPSquirrelRecursion) Could not remove the previous goals from the knowledge base.");
					exit(-1);
				}
				
				std::stringstream ss;
				ss << "(" << knowledge_item.attribute_name;
				for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = knowledge_item.values.begin(); ci != knowledge_item.values.end(); ++ci)
				{
					ss << " " << (*ci).value;
				}
				ss << ")";
				ROS_INFO("KCL: (RPSquirrelRecursion) Removed the fact %s.", ss.str().c_str());
			}
			
			// add initial state (robot_at)
			rosplan_knowledge_msgs::KnowledgeItem waypoint_knowledge;
			rosplan_knowledge_msgs::KnowledgeUpdateService add_waypoints_service;
			add_waypoints_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;

			waypoint_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			waypoint_knowledge.attribute_name = "robot_at";
			waypoint_knowledge.is_negative = false;
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
			
			// Trigger a replan.
			fb.action_id = msg->action_id;
			fb.status = "action failed";
			action_feedback_pub.publish(fb);
			return;
		}
		
		// If an action was successful we update the knowledge base with the next predicates that are now true.
		if(state == actionlib::SimpleClientGoalState::SUCCEEDED ||
		   ("explore_area" == action_name && task_state_monitor->enoughLumpsFound())
		)
		{
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
		
		// Remove all previous goals.
		rosplan_knowledge_msgs::GetAttributeService attribute_service;
		attribute_service.request.predicate_name = "explored";
		if (!get_attribute_client.call(attribute_service))
		{
			ROS_ERROR("KCL: (RPSquirrelRecursion) Unable to call the attribute service.");
			exit(-1);
		}
		
		for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = attribute_service.response.attributes.begin(); ci != attribute_service.response.attributes.end(); ++ci)
		{
			const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item = *ci;
			rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
			knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_GOAL;
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (RPSquirrelRecursion) Could not remove the previous goals from the knowledge base.");
				exit(-1);
			}
			
			std::stringstream ss;
			ss << "(" << knowledge_item.attribute_name;
			for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = knowledge_item.values.begin(); ci != knowledge_item.values.end(); ++ci)
			{
				ss << " " << (*ci).value;
			}
			ss << ")";
			ROS_INFO("KCL: (RPSquirrelRecursion) Removed the goal %s.", ss.str().c_str());
		}
		
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
				
				// Setup the toys.
				task_state_monitor->initialiseToys();
			}
			
			PlanningEnvironment planning_environment;
			planning_environment.parseDomain(domain_name);
			planning_environment.update(*node_handle);
			PDDLProblemGenerator pddl_problem_generator;
			
			pddl_problem_generator.generatePDDLProblemFile(planning_environment, req.problem_path);
			initial_problem_generated = true;
			return true;
		}
		return true;
	}
	
	void RPSquirrelRecursion::generateInitialState()
	{
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

		// add initial state (robot_at)
		rosplan_knowledge_msgs::KnowledgeItem waypoint_knowledge;
		rosplan_knowledge_msgs::KnowledgeUpdateService add_waypoints_service;
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
		//diagnostic_msgs::KeyValue kv;
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
		
		// Setup the camera.
		waypoint_knowledge.attribute_name = "camera_neutral";
		waypoint_knowledge.is_negative = false;
		kv.key = "v";
		kv.value = "kenny";
		waypoint_knowledge.values.push_back(kv);
		add_waypoints_service.request.knowledge = waypoint_knowledge;
		if (!update_knowledge_client.call(add_waypoints_service)) {
			ROS_ERROR("KCL: (TidyRooms) Could not add the fact (camera_neutral kenny) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (TidyRooms) Added (camera_neutral kenny) to the knowledge base.");
		waypoint_knowledge.values.clear();
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

			int max_viewcones = 2;
			int occupancy_threshold = 5;
			float fov = 0.8f;
			float view_distance = 3.0f;
			int sample_size = 1000;
			float safe_distance = 1.0f;
			
			node_handle->getParam("/squirrel_interface_recursion/viewcone_max_viewcones", max_viewcones);
			node_handle->getParam("/squirrel_interface_recursion/viewcone_occupancy_threshold", occupancy_threshold);
			node_handle->getParam("/squirrel_interface_recursion/viewcone_field_of_view", fov);
			node_handle->getParam("/squirrel_interface_recursion/viewcone_view_distance", view_distance);
			node_handle->getParam("/squirrel_interface_recursion/viewcone_sample_size", sample_size);
			node_handle->getParam("/squirrel_interface_recursion/viewcone_safe_distance", safe_distance);
			
			ROS_INFO("KCL: (RPSquirrelRecursion) Generate view cones:");
			ROS_INFO("KCL: (RPSquirrelRecursion) Max view cones: %d", max_viewcones);
			ROS_INFO("KCL: (RPSquirrelRecursion) Occupancy threshold: %d", occupancy_threshold);
			ROS_INFO("KCL: (RPSquirrelRecursion) Field of View (in Radians): %f", fov);
			ROS_INFO("KCL: (RPSquirrelRecursion) View distance: %f", view_distance);
			ROS_INFO("KCL: (RPSquirrelRecursion) Sample size: %d", sample_size);
			ROS_INFO("KCL: (RPSquirrelRecursion) Safe distance: %f", safe_distance);
			
			view_cone_generator->createViewCones(view_poses, bounding_box->getBoundingBox(), max_viewcones, occupancy_threshold, fov, view_distance, sample_size, safe_distance);
			
			// Add these poses to the knowledge base.
			rosplan_knowledge_msgs::KnowledgeUpdateService add_waypoints_service;
			add_waypoints_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			
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
				
				// Add the known types.
				waypoint_knowledge.instance_type = "type";
				waypoint_knowledge.instance_name = "type1";
				add_waypoints_service.request.knowledge = waypoint_knowledge;
				if (!update_knowledge_client.call(add_waypoints_service)) {
					ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add a type to the knowledge base.");
					exit(-1);
				}
				
				waypoint_knowledge.instance_name = "type2";
				add_waypoints_service.request.knowledge = waypoint_knowledge;
				if (!update_knowledge_client.call(add_waypoints_service)) {
					ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add a type to the knowledge base.");
					exit(-1);
				}
				
				rosplan_knowledge_msgs::GetInstanceService getInstances;
				getInstances.request.type_name = "type";
				if (!get_instance_client.call(getInstances)) {
					ROS_ERROR("KCL: (PerceptionAction) Failed to get all the type instances.");
					return false;
				}
				ROS_INFO("KCL: (PerceptionAction) Received %zd type instances.", getInstances.response.instances.size());
				
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
			
			std::map<std::string, geometry_msgs::Pose> object_to_location_map;
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
				if (message_store.queryNamed<geometry_msgs::PoseStamped>(location_predicate, location_locations) && location_locations.size() == 1)
				{
					for (std::vector<boost::shared_ptr<geometry_msgs::PoseStamped> >::const_iterator ci = location_locations.begin(); ci != location_locations.end(); ++ci)
					{
						//std::cout << "KCL: (RPSquirrelRoadmap) Found the location of " << location_predicate << ": (" << (*ci)->pose.position.x << ", " << (*ci)->pose.position.y << ", " << (*ci)->pose.position.z << ")" << std::endl;
						ROS_ERROR("KCL: (RPSquirrelRoadmap) Found the location of %s: (%f, %f, %f)", location_predicate.c_str(), (*ci)->pose.position.x, (*ci)->pose.position.y, (*ci)->pose.position.z);
						object_to_location_map[object_predicate] = (*ci)->pose;
					}
				}
				else
				{
					ROS_ERROR("KCL: (RPSquirrelRoadmap) could not query message store to fetch object pose. Found %zd poses.", location_locations.size());
					return false;
				}
				
				// Find locations from where we can observe
				squirrel_waypoint_msgs::ExamineWaypoint getTaskPose;

				getTaskPose.request.object_pose.header = location_locations[0]->header;
				getTaskPose.request.object_pose.pose = location_locations[0]->pose;
				
				ROS_INFO("KCL: (RPSquirrelRecursion) Find observation poses.");
				
				if (!classify_object_waypoint_client.call(getTaskPose))
				{
					ROS_ERROR("KCL: (RPSquirrelRecursion) Failed to recieve classification waypoints for %s.", object_predicate.c_str());
					return false;
				}

				std_msgs::Int8 debug_pose_number;
				debug_pose_number.data = getTaskPose.response.poses.size();
				ROS_INFO("KCL: (RPSquirrelRecursion) Found %d observation poses", debug_pose_number.data);

				// Add all the waypoints to the knowledge base.
				std::stringstream ss;
				geometry_msgs::PoseStamped best_pose;
				best_pose.header.seq = 0;
				best_pose.header.frame_id = "/map";
				best_pose.header.stamp = ros::Time::now();
				std_msgs::Float64 best_angle;
				
				float max_distance = -std::numeric_limits<float>::max();
				
				for(int i=0;i<getTaskPose.response.poses.size(); i++) {
				
					ss.str(std::string());
					ss << object_predicate << "_observation_wp";
					
					// Check if this location is suitable.
					float distance = view_cone_generator->minDistanceToBlocked(getTaskPose.response.poses[i].pose.pose.position, 0.4f);
					if (distance > max_distance)
					{
						max_distance = distance;
						best_pose.pose = getTaskPose.response.poses[i].pose.pose;
						best_angle.data = getTaskPose.response.tiltAngle[i];
					}
				}
				
				// Store the best pose in the message_store.
				std::string id(message_store.insertNamed(ss.str(), best_pose));
				{
				std::stringstream angle_ss;
				angle_ss << ss.str() << "_angle";
				std::string id(message_store.insertNamed(angle_ss.str(), best_angle));
				}
				ROS_INFO("KCL: (RPSquirrelRecursion) Best pose: (%f, %f, %f) Q=[%f, %f, %f, %f], distance to an obstacle: %f. Desired tilt angle: %f", best_pose.pose.position.x, best_pose.pose.position.y, best_pose.pose.position.z, best_pose.pose.orientation.x, best_pose.pose.orientation.y, best_pose.pose.orientation.z, best_pose.pose.orientation.w, max_distance, best_angle.data);
				
				tf::Quaternion q(best_pose.pose.orientation.x, best_pose.pose.orientation.y, best_pose.pose.orientation.z, best_pose.pose.orientation.w);
				tf::Matrix3x3 m(q);
				double roll, pitch, yaw;
				m.getRPY(roll, pitch, yaw);
				
				ROS_INFO("KCL: (RPSquirrelRecursion) Roll: %f, Pitch: %f, Yaw: %f\n", roll, pitch, yaw);
				
				rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
				updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
				updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
				updateSrv.request.knowledge.instance_type = "waypoint";
				updateSrv.request.knowledge.instance_name = ss.str();
				if (!update_knowledge_client.call(updateSrv)) {
					ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the instance %s to the knowledge base.", ss.str().c_str());
					exit(-1);
				}
				
				// Add the fact that the object is observable from there.
				updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				updateSrv.request.knowledge.attribute_name = "observable_from";
				updateSrv.request.knowledge.is_negative = false;
				diagnostic_msgs::KeyValue kv;
				kv.key = "o";
				kv.value = object_predicate;
				updateSrv.request.knowledge.values.push_back(kv);
				
				kv.key = "wp";
				kv.value = ss.str();
				updateSrv.request.knowledge.values.push_back(kv);
				if (!update_knowledge_client.call(updateSrv)) {
					ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the fact (observable_from %s %s) to the knowledge base.", object_predicate.c_str(), ss.str().c_str());
					exit(-1);
				}
				
				updateSrv.request.knowledge.values.clear();
				
				// Setup the goal.
				updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
				updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				updateSrv.request.knowledge.attribute_name = "examined";
				updateSrv.request.knowledge.is_negative = false;
				//diagnostic_msgs::KeyValue kv;
				kv.key = "o";
				kv.value = object_predicate;
				updateSrv.request.knowledge.values.push_back(kv);
				
				//updateSrv.request.knowledge = update_knowledge_client;
				if (!update_knowledge_client.call(updateSrv)) {
					ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the goal (examined %s) to the knowledge base.", object_predicate.c_str());
					exit(-1);
				}
				ROS_INFO("KCL: (RPSquirrelRecursion) Added the goal (examined %s) to the knowledge base.", object_predicate.c_str());
			}
			
			/**
			 * Setup the order in which objects need to be observed.
			 */
			std::string robot_location;
			get_attribute.request.predicate_name = "robot_at";
			if (!get_attribute_client.call(get_attribute)) {
				ROS_ERROR("KCL: (RPSquirrelRecursion) Failed to recieve the attributes of the predicate 'robot_at'");
				return false;
			}
			
			for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = get_attribute.response.attributes.begin(); ci != get_attribute.response.attributes.end(); ++ci) {
				const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item = *ci;
				for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = knowledge_item.values.begin(); ci != knowledge_item.values.end(); ++ci) {
					const diagnostic_msgs::KeyValue& key_value = *ci;
					if ("v" == key_value.key) {
						robot_location = key_value.value;
						break;
					}
				}
				if (robot_location != "")
					break;
			}
			
			// Get the actual location of this robot.
			std::vector<boost::shared_ptr<geometry_msgs::PoseStamped> > robot_locations;
			if (message_store.queryNamed<geometry_msgs::PoseStamped>(robot_location, robot_locations) && robot_locations.size() > 1)
			{
				ROS_INFO("KCL: (RPSquirrelRecursion) Found the location of the robot.");
			}
			else
			{
				ROS_ERROR("KCL: (RPSquirrelRecursion) could not query message store to fetch the robot pose. Found %zd poses.", robot_locations.size());
				return false;
			}
			
			std::set<std::string> processed_objects;
			geometry_msgs::Pose current_pose = robot_locations[0]->pose;
			std::string previous_object;
			while (processed_objects.size() != object_to_location_map.size())
			{
				float smallest_distance = std::numeric_limits<float>::max();
				std::string closest_object;
				for (std::map<std::string, geometry_msgs::Pose>::const_iterator ci = object_to_location_map.begin(); ci != object_to_location_map.end(); ++ci)
				{
					const std::string& name = ci->first;
					const geometry_msgs::Pose& object_pose = ci->second;
					float distance = (current_pose.position.x - object_pose.position.x) * (current_pose.position.x - object_pose.position.x) +
					                 (current_pose.position.y - object_pose.position.y) * (current_pose.position.y - object_pose.position.y) +
					                 (current_pose.position.z - object_pose.position.z) * (current_pose.position.z - object_pose.position.z);
					
					if (distance < smallest_distance)
					{
						smallest_distance = distance;
						closest_object = name;
					}
				}
				
				// Update the current pose to the closest object's location.
				current_pose = object_to_location_map[closest_object];
				
				// Make this object to be observed next.
				if (previous_object != "")
				{
					updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
					updateSrv.request.knowledge.attribute_name = "next";
					updateSrv.request.knowledge.is_negative = false;
					diagnostic_msgs::KeyValue kv;
					kv.key = "o";
					kv.value = previous_object;
					updateSrv.request.knowledge.values.push_back(kv);
					
					kv.key = "o2";
					kv.value = closest_object;
					updateSrv.request.knowledge.values.push_back(kv);
					if (!update_knowledge_client.call(updateSrv)) {
						ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the fact (next %s %s) to the knowledge base.", closest_object.c_str(), previous_object.c_str());
						exit(-1);
					}
				}
				// This is the first object to observe.
				else
				{
					updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
					updateSrv.request.knowledge.attribute_name = "observe";
					updateSrv.request.knowledge.is_negative = false;
					diagnostic_msgs::KeyValue kv;
					kv.key = "o";
					kv.value = closest_object;
					updateSrv.request.knowledge.values.push_back(kv);
					
					if (!update_knowledge_client.call(updateSrv)) {
						ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the fact (observe %s) to the knowledge base.", closest_object.c_str());
						exit(-1);
					}
				}
				
				previous_object = closest_object;
			}
			
			// For the last object we need to insert an additional fact, otherwise the planning problem is unsolvable.
			updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			updateSrv.request.knowledge.attribute_name = "next";
			updateSrv.request.knowledge.is_negative = false;
			diagnostic_msgs::KeyValue kv;
			kv.key = "o";
			kv.value = previous_object;
			updateSrv.request.knowledge.values.push_back(kv);
			
			kv.key = "o2";
			kv.value = previous_object;
			updateSrv.request.knowledge.values.push_back(kv);
			if (!update_knowledge_client.call(updateSrv)) {
				ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the fact (next %s %s) to the knowledge base.", previous_object.c_str(), previous_object.c_str());
				exit(-1);
			}
			
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
		
		// Start the goto waypoint wrapper.
		float fov, view_distance;
		nh.getParam("/squirrel_interface_recursion/viewcone_field_of_view", fov);
		nh.getParam("/squirrel_interface_recursion/viewcone_view_distance", view_distance);
		KCL_rosplan::GotoWaypointWrapper goto_wrapper(nh, "/move_base", fov, view_distance);
		
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

		while(ros::ok() && ros::master::check()){ros::spinOnce();}
		return 0;
	}
	
