#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>
#include "mongodb_store/message_store.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_srvs/Empty.h"
#include "diagnostic_msgs/KeyValue.h"
#include <actionlib/client/simple_action_client.h>

#include "rosplan_dispatch_msgs/PlanAction.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"

#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/GetInstanceService.h"
#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "rosplan_knowledge_msgs/GenerateProblemService.h"
#include "rosplan_knowledge_msgs/KnowledgeQueryService.h"
#include "rosplan_dispatch_msgs/PlanningService.h"

#include "rosplan_planning_system/PlanningEnvironment.h"
#include "rosplan_planning_system/PDDLProblemGenerator.h"

#include "squirrel_waypoint_msgs/ExamineWaypoint.h"
#include "squirrel_object_perception_msgs/SceneObject.h"

#ifndef KCL_recursion
#define KCL_recursion

/**
 * This file defines the RPSquirrelRecursion class.
 * RPSquirrelRecursion is used to execute strategic PDDL actions
 * that correspond to a tactical problem. This is done through
 * instantiating a new planning system node.
 */
namespace KCL_rosplan {

	class ViewConeGenerator;
	
	class RPSquirrelRecursion
	{

	private:
		ros::NodeHandle* node_handle;
		mongodb_store::MessageStoreProxy message_store;
		ros::Publisher action_feedback_pub;
		
		/* PDDL problem generation */
		

		/* knowledge service clients */
		ros::ServiceClient update_knowledge_client;
		ros::ServiceClient get_instance_client;
		ros::ServiceClient get_attribute_client;
		ros::ServiceClient query_knowledge_client;
		
		// waypoint request services
		ros::ServiceClient classify_object_waypoint_client;
		
		// server that generates the PDDL domain and problem files.
		ros::ServiceServer pddl_generation_service;
		
		// Cache the last message sent.
		std::vector<rosplan_dispatch_msgs::ActionDispatch> last_received_msg;
		
		// View point generator.
		ViewConeGenerator* view_cone_generator;
		
		// Generate the initial state for the highest level of abstraction.
		void generateInitialState();
		
		/**
		 * Create a PDDL domainfile that is needed to execute @ref{action_name}.
		 * @param action_name The name of the PDDL action that has been dispatched.
		 * @return True if the domain was successfully created, false otherwise.
		 */
		bool createDomain(const std::string& action_name);
		
		bool initial_problem_generated;
		
		// When true, the exploration stops when enough 'lumps' have been found.
		bool stop_when_enough_lumps_found;
		
		// For this experiment we know the number of toys we need to find.
		unsigned int number_of_toys;

	public:

		/* constructor */
		RPSquirrelRecursion(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
		
		/* callback function from the ROSPlan planning system to generate the PDDL problem file (and domain in our case) */
		bool generatePDDLProblemFile(rosplan_knowledge_msgs::GenerateProblemService::Request &req, rosplan_knowledge_msgs::GenerateProblemService::Response &res);
		bool generateContingentProblem(rosplan_knowledge_msgs::GenerateProblemService::Request &req, rosplan_knowledge_msgs::GenerateProblemService::Response &res);
		bool generateRegularProblem(rosplan_knowledge_msgs::GenerateProblemService::Request &req, rosplan_knowledge_msgs::GenerateProblemService::Response &res);
	};
}
#endif
