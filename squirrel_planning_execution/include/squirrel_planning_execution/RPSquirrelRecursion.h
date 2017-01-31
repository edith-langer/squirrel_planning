#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/foreach.hpp>
#include "mongodb_store/message_store.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/Point.h>
#include "std_srvs/Empty.h"
#include <std_msgs/Float64.h>
#include "diagnostic_msgs/KeyValue.h"
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

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
	
	/**
	 * Utility class to aid placing toys and viewcones within a defined bounding box.
	 */
	class BoundingBox
	{
	public:
		/**
		 * Setup the bounding box.
		 * @param nh ROS Node Handle.
		 */
		BoundingBox(ros::NodeHandle& nh);
		
		/**
		 * Check whether @ref{v} is inside this bounding box.
		 * @param v The point to be checked.
		 * @return True if @ref{v} falls within the bounding box, false otherwise.
		 */
		bool isInside(const tf::Vector3& v) const;
		
		/**
		 * Generate a point that falls within this bounding box.
		 * @return A point that falls within this bounding box.
		 */
		tf::Vector3 createPoint() const;
		
		/**
		 * Get the bounding box points.
		 */
		inline const std::vector<tf::Vector3>& getBoundingBox() const { return bounding_box; }
		
	private:
		// Bounding box where all the objects and view cones should be placed.
		std::vector<tf::Vector3> bounding_box;
	};
	
	struct ToyState
	{
		ToyState(const geometry_msgs::Point& location, const std::string& toy_name)
			: location_(location), name_(toy_name), is_examined_(false)
		{
			
		}
		
		void setExamined()
		{
			is_examined_ = true;
			time_stamp_ = ros::Time::now();
		}
		
		geometry_msgs::Point location_;
		std::string name_;
		bool is_examined_;
		ros::Time time_stamp_;
	};
	
	/**
	 * Class to manage the toys in the domain and manage to state of the overall system.
	 */
	class TaskStateMonitor
	{
	public:
		
		/**
		 * Constructor.
		 * @param nh The ROS node handle.
		 * @param bb The bounding box where all toys should be placed in.
		 * @param vvg A utility class that performs functions on an occupancy grid.
		 * @param ms The message_store (link to MongoDB).
		 */
		TaskStateMonitor(ros::NodeHandle& nh, const BoundingBox& bb, const ViewConeGenerator& vcg, mongodb_store::MessageStoreProxy& ms);
		
		/**
		 * Initialise the toys that need to be found. The paramters that needs to be set in the launchfile are:
		 * - number_of_toys (int), sets the number of objects that exists.
		 * - spawn_objects (bool), if true we call the spawn service for Gazebo.
		 * - model_file_name (string), the path to a model that is spawned in Gazebo.
		 * - toy_p{#number} (string), multiple of these parameters can exist. We look for @ref{number_of_toys) 
		 *   params named: toy_p0, toy_p1, ... toy_p{number_of_toys} - 1. The value is three floats separated by 
		 *   commas: "x-coordinate, y-coordinate, z-coordinate".
		 * - viewcone_bounding_box_p{#number}, multiple of these parameters can exist. The parameters need to 
		 *   be named: viewcone_bounding_box_p0, ..., viewcone_bounding_box_p{M}. The value is three floats separated 
		 *   by commas: "x-coordinate, y-coordinate, z-coordinate". These coordinates much be ordered such that they 
		 *   form a convex polygon.
		 */
		void initialiseToys();
		
		/**
		 * Check if we have achieved our objectives and update the state of the toys.
		 */
		void updateState();
		
		/**
		 * @return Whether the task is complete.
		 */
		bool isComplete() const { return is_complete; }
		
		/**
		 * @return Whether enough unexplored lumps are found.
		 */
		bool enoughLumpsFound() const { return enough_unexplored_lumps_found; }
		
		/**
		 * @return The number of objects we are looking for.
		 */
		int getNumberOfToysToFind() const { return number_of_toys_to_find; }
		
		/**
		 * @return The state of the toys.
		 */
		const std::vector<ToyState>& getToyLocations() const { return toy_locations; }
		
	private:
		
		// ROS Nodehandle.
		ros::NodeHandle* node_handle;
		
		// Bounding box all toys should be inside of.
		const BoundingBox* bounding_box;
		
		// A class that receives an occupancy grid and provides utility functions.
		const ViewConeGenerator* view_cone_generator;
		
		// MongoDB interface.
		mongodb_store::MessageStoreProxy* message_store;
		
		// ROSPlan Knowledge base interfaces.
		ros::ServiceClient query_knowledge_client;
		
		// Gazebo interfaces.
		ros::ServiceClient gazebo_spawn_model_client;
		
		// The locations of the toys toy_locationsin this domain.
		std::vector<ToyState> toy_locations;
		
		int number_of_toys_to_find;         // The number of toys that we need to find.
		bool enough_unexplored_lumps_found; // Have enough lumps been found to start examination?
		bool is_complete;                   // Have enough objects been examined? I.e. is the task complete?
	};
	
	/**
	 * Utility function to split strings.
	 */
	void split(const string& s, char delim, std::vector<std::string>& elements)
	{
		std::stringstream ss;
		ss.str(s);
		string item;
		while (std::getline(ss, item, delim))
		{
			elements.push_back(item);
		}
	}
	
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
		
		// Flag that is set True when the first PDDL problem has been generated. 
		bool initial_problem_generated;
		
		// Bounding box where all the objects and view cones should be placed.
		BoundingBox* bounding_box;
		
		// Make sure waypoints are uniquely named.
		int waypoint_number;
		
		// Class that manages the state of the task and toys.
		TaskStateMonitor* task_state_monitor;
		
		// Start time of the experiments.
		ros::Time start_time;
		
		// The number of segmentation actions performed.
		int number_of_segmentation_actions;

	public:

		/* constructor */
		RPSquirrelRecursion(ros::NodeHandle &nh);
		
		/* listen to and process action_dispatch topic */
		void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
		
		/* callback function from the ROSPlan planning system to generate the PDDL problem file (and domain in our case) */
		bool generatePDDLProblemFile(rosplan_knowledge_msgs::GenerateProblemService::Request &req, rosplan_knowledge_msgs::GenerateProblemService::Response &res);
	};
}
#endif
