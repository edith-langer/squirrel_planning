#include <std_msgs/Int8.h>

#include <map>
#include <algorithm>
#include <string>
#include <sstream>

#include "squirrel_planning_execution/SortingGame.h"
#include "pddl_actions/ShedKnowledgePDDLAction.h"
#include "pddl_actions/FinaliseClassificationPDDLAction.h"
#include "pddl_actions/NextTurnPDDLAction.h"
#include "pddl_actions/PlannerInstance.h"
#include "pddl_actions/SimulatedObservePDDLAction.h"

/* The implementation of RPSquirrelRecursion.h */
namespace KCL_rosplan {

	/*-------------*/
	/* constructor */
	/*-------------*/

	SortingGame::SortingGame(ros::NodeHandle &nh)
		: node_handle(&nh), message_store(nh), initial_problem_generated(false)
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
		
		generateInitialState();
	}
	
	/*--------------------*/
	/* problem generation */
	/*--------------------*/
	void SortingGame::generateInitialState()
	{
		/** INSTANCES **/
		
		// Robots.
		rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
		knowledge_item.instance_type = "robot";
		knowledge_item.instance_name = "kenny";
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SortingGame) Could not add kenny to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SortingGame) Added kenny to the knowledge base.");
		
		// Waypoints.
		knowledge_item.instance_type = "waypoint";
		knowledge_item.instance_name = "start_wp";
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SortingGame) Could not add start_wp to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SortingGame) Added start_wp to the knowledge base.");
		
		knowledge_item.instance_name = "idle_wp";
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SortingGame) Could not add idle_wp to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SortingGame) Added idle_wp to the knowledge base.");
		
		knowledge_item.instance_name = "object_wp";
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SortingGame) Could not add object_wp to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SortingGame) Added object_wp to the knowledge base.");
		
		knowledge_item.instance_name = "pickup_wp";
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SortingGame) Could not add pickup_wp to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SortingGame) Added pickup_wp to the knowledge base.");
		
		knowledge_item.instance_name = "drop_wp";
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SortingGame) Could not add drop_wp to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SortingGame) Added drop_wp to the knowledge base.");
		
		// Sounds.
		knowledge_item.instance_type = "sound";
		knowledge_item.instance_name = "sound_ok";
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SortingGame) Could not add sound_ok to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SortingGame) Added sound_ok to the knowledge base.");
		
		knowledge_item.instance_name = "sound_no";
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SortingGame) Could not add sound_no to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SortingGame) Added sound_no to the knowledge base.");
		
		knowledge_item.instance_name = "sound_hi";
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SortingGame) Could not add sound_hi to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SortingGame) Added sound_hi to the knowledge base.");
		
		// Wiggles.
		knowledge_item.instance_type = "wiggle";
		knowledge_item.instance_name = "wiggle_error";
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SortingGame) Could not add wiggle_error to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SortingGame) Added wiggle_error to the knowledge base.");
		
		knowledge_item.instance_name = "wiggle_no";
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SortingGame) Could not add wiggle_no to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SortingGame) Added wiggle_no to the knowledge base.");
		
		// Knowledge bases.
		knowledge_item.instance_type = "knowledgebase";
		knowledge_item.instance_name = "basis_kb";
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SortingGame) Could not add basis_kb to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SortingGame) Added basis_kb to the knowledge base.");
		
		knowledge_item.instance_name = "kid_0_kb";
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SortingGame) Could not add kid_0_kb to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SortingGame) Added kid_0_kb to the knowledge base.");
		
		// Kids.
		knowledge_item.instance_type = "kid";
		knowledge_item.instance_name = "kid_0";
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SortingGame) Could not add kid_0 to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SortingGame) Added kid_0 to the knowledge base.");
		
		knowledge_item.instance_name = "kid_1";
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SortingGame) Could not add kid_1 to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SortingGame) Added kid_1 to the knowledge base.");
			
		// Commands.
		knowledge_item.instance_type = "command";
		knowledge_item.instance_name = "gehe";
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SortingGame) Could not add gehe to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SortingGame) Added gehe to the knowledge base.");
		
		knowledge_item.instance_name = "links";
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SortingGame) Could not add links to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SortingGame) Added links to the knowledge base.");
		
		// Objects.
		knowledge_item.instance_type = "object";
		knowledge_item.instance_name = "object0";
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SortingGame) Could not add object0 to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SortingGame) Added object0 to the knowledge base.");
		
		// Types.
		knowledge_item.instance_type = "type";
		knowledge_item.instance_name = "dinosaur";
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SortingGame) Could not add dinosaur to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SortingGame) Added dinosaur to the knowledge base.");
		
		// Levels.
		knowledge_item.instance_type = "level";
		for (unsigned int i = 0; i < 6; ++i)
		{
			std::stringstream ss;
			ss << "l" << i;
			knowledge_item.instance_name = ss.str();
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (SortingGame) Could not add %s to the knowledge base.", ss.str().c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (SortingGame) Added %s to the knowledge base.", ss.str().c_str());
		}
		
		/** FACTS **/
		
		// robot_at.
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge_item.attribute_name = "robot_at";
		knowledge_item.is_negative = false;
		diagnostic_msgs::KeyValue kv;
		kv.key = "v";
		kv.value = "kenny";
		knowledge_item.values.push_back(kv);
		kv.key = "wp";
		kv.value = "start_wp";
		knowledge_item.values.push_back(kv);
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SortingGame) Could not add the fact (robot_at kenny start_wp) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SortingGame) Added (robot_at kenny start_wp) to the knowledge base.");
		knowledge_item.values.clear();
		
		// current_kb.
		knowledge_item.attribute_name = "current_kb";
		knowledge_item.is_negative = false;
		kv.key = "kb";
		kv.value = "basis_kb";
		knowledge_item.values.push_back(kv);
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SortingGame) Could not add the fact (current_kb basis_kb) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SortingGame) Added (current_kb basis_kb) to the knowledge base.");
		knowledge_item.values.clear();
		
		// parent.
		knowledge_item.attribute_name = "parent";
		knowledge_item.is_negative = false;
		kv.key = "kb";
		kv.value = "basis_kb";
		knowledge_item.values.push_back(kv);
		
		kv.key = "kb2";
		kv.value = "kid_0_kb";
		knowledge_item.values.push_back(kv);
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SortingGame) Could not add the fact (parent basis_kb kid_0_kb) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SortingGame) Added (parent basis_kb kid_0_kb) to the knowledge base.");
		knowledge_item.values.clear();
		
		// gripper_empty.
		knowledge_item.attribute_name = "gripper_empty";
		knowledge_item.is_negative = false;
		kv.key = "v";
		kv.value = "kenny";
		knowledge_item.values.push_back(kv);
		
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SortingGame) Could not add the fact (gripper_empty kenny) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SortingGame) Added (gripper_empty kenny) to the knowledge base.");
		knowledge_item.values.clear();
		
		// near.
		knowledge_item.attribute_name = "near";
		knowledge_item.is_negative = false;
		kv.key = "wp1";
		kv.value = "pickup_wp";
		knowledge_item.values.push_back(kv);
		
		kv.key = "wp2";
		kv.value = "object_wp";
		knowledge_item.values.push_back(kv);
		 
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SortingGame) Could not add the fact (near pickup_wp object_wp) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SortingGame) Added (near pickup_wp object_wp) to the knowledge base.");
		knowledge_item.values.clear();
		
		kv.key = "wp1";
		kv.value = "idle_wp";
		knowledge_item.values.push_back(kv);
		
		kv.key = "wp2";
		kv.value = "drop_wp";
		knowledge_item.values.push_back(kv);
		 
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SortingGame) Could not add the fact (near idle_wp drop_wp) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SortingGame) Added (near idle_wp drop_wp) to the knowledge base.");
		knowledge_item.values.clear();
		
		// Levels.
		knowledge_item.attribute_name = "next";
		knowledge_item.is_negative = false;
		
		for (unsigned int i = 0; i < 5; ++i)
		{
			std::stringstream ss;
			ss << "l" << (i + 1);
			std::string l_next = ss.str();
			kv.key = "l";
			kv.value = l_next;
			knowledge_item.values.push_back(kv);
			
			ss.str(std::string());
			ss << "l" << i;
			std::string l_prev = ss.str();
			kv.key = "l2";
			kv.value = l_prev;
			knowledge_item.values.push_back(kv);
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (SortingGame) Could not add the fact (next %s %s) to the knowledge base.", l_next.c_str(), l_prev.c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (SortingGame) Added (next %s %s) to the knowledge base.", l_next.c_str(), l_prev.c_str());
			knowledge_item.values.clear();
		}
		
		// lev.
		knowledge_item.attribute_name = "lev";
		knowledge_item.is_negative = false;
		kv.key = "l";
		kv.value = "l0";
		knowledge_item.values.push_back(kv);
		
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SortingGame) Could not add the fact (lev l0) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SortingGame) Added (lev l0) to the knowledge base.");
		knowledge_item.values.clear();
		
		// resolve-axioms
		knowledge_item.attribute_name = "resolve-axioms";
		knowledge_item.is_negative = false;
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SortingGame) Could not add the fact (resolve-axioms) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SortingGame) Added (resolve-axioms) to the knowledge base.");
		knowledge_item.values.clear();
		
		// Set all commands to false.
		knowledge_item.attribute_name = "has_commanded";
		knowledge_item.is_negative = true;
		
		kv.key = "k";
		kv.value = "kid_0";
		knowledge_item.values.push_back(kv);
		
		kv.key ="c";
		kv.value = "gehe";
		knowledge_item.values.push_back(kv);
		
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SortingGame) Could not add the fact (not (has_commanded kid_0 gehe)) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SortingGame) Added (not (has_commanded kid_0 gehe)) to the knowledge base.");
		knowledge_item.values.clear();
		
		kv.key = "k";
		kv.value = "kid_0";
		knowledge_item.values.push_back(kv);
		
		kv.key ="c";
		kv.value = "links";
		knowledge_item.values.push_back(kv);
		
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SortingGame) Could not add the fact (not (has_commanded kid_0 links)) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SortingGame) Added (not (has_commanded kid_0 links)) to the knowledge base.");
		knowledge_item.values.clear();
		
		kv.key = "k";
		kv.value = "kid_0";
		knowledge_item.values.push_back(kv);
		
		kv.key ="c";
		kv.value = "gone";
		knowledge_item.values.push_back(kv);
		
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SortingGame) Could not add the fact (not (has_commanded kid_0 gone)) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SortingGame) Added (not (has_commanded kid_0 gone)) to the knowledge base.");
		knowledge_item.values.clear();
	}
	
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_SortingGame");
		ros::NodeHandle nh;

		// create PDDL action subscriber
		KCL_rosplan::SortingGame sorting_game(nh);
		
		// Setup all the simulated actions.
		KCL_rosplan::ShedKnowledgePDDLAction shed_knowledge_action(nh);
		KCL_rosplan::FinaliseClassificationPDDLAction finalise_classify_action(nh);
		KCL_rosplan::NextTurnPDDLAction next_turn_action(nh);
		KCL_rosplan::SimulatedObservePDDLAction simulated_observe_action(nh);
/*		
		// Lets start the planning process.
		std::string data_path;
		nh.getParam("/data_path", data_path);
		
		std::string planner_path;
		nh.getParam("/planner_path", planner_path);
		
		std::stringstream ss;
		ss << data_path << "sorting_game_domain.pddl";
		std::string domain_path = ss.str();
		
		std::string problem_path;
		
		std::string planner_command;
		ss.str(std::string());
		ss << "cat " << data_path << "/sorting_game_plan.pddl";
		planner_command = ss.str();
		
		rosplan_dispatch_msgs::PlanGoal psrv;
		psrv.domain_path = domain_path;
		psrv.problem_path = problem_path;
		psrv.data_path = data_path;
		psrv.planner_command = planner_command;
		psrv.start_action_id = 0;

		ROS_INFO("KCL: (SortingGame) Start plan action");
		actionlib::SimpleActionClient<rosplan_dispatch_msgs::PlanAction> plan_action_client("/kcl_rosplan/start_planning", true);

		plan_action_client.waitForServer();
		ROS_INFO("KCL: (SortingGame) Start planning server found");
		
		// send goal
		plan_action_client.sendGoal(psrv);
		ROS_INFO("KCL: (SortingGame) Goal sent");
*/
		while(ros::ok() && ros::master::check()){ros::spinOnce();}
		return 0;
	}
	
