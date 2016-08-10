#ifndef SQUIRRELPLANNINGEXECUTION_PDDLACTIONS_PLANNERINSTANCE_H
#define SQUIRRELPLANNINGEXECUTION_PDDLACTIONS_PLANNERINSTANCE_H

#include <ros/ros.h>
#include <rosplan_dispatch_msgs/ActionDispatch.h>
#include <rosplan_dispatch_msgs/PlanAction.h>
#include <actionlib/client/simple_action_client.h>


namespace KCL_rosplan
{

/**
 * This factory creates a new ROSPlan instance so we can have multiple instances running at the same time.
 */
class PlannerInstance
{
public:
	
	/**
	 * Create an instance of the ROS Planner.
	 * @param node_handle A ROS node handle.
	 */
	static PlannerInstance& createInstance(ros::NodeHandle& node_handle);
	
	/**
	 * Destructor
	 */
	~PlannerInstance();
	
	/**
	 * @return The state of the planning system.
	 */
	actionlib::SimpleClientGoalState getState() const;
	
	/**
	 * Start the planner.
	 * @param domain_path The PDDL domain path.
	 * @param problem_path The PDDL problem path.
	 * @param data_path The data path.
	 * @param planner_command The planner command that gets executed.
	 */
	void startPlanner(const std::string& domain_path, const std::string& problem_path, const std::string& data_path, const std::string& planner_command);
	
	/**
	 * Stops the planner. It only sends the command to the planner, it might take longer before the planner has actually stopped.
	 */
	void stopPlanner();
private:
	
	/**
	 * Constructor.
	 * @param node_handle An existing and initialised ros node handle.
	 */
	PlannerInstance(ros::NodeHandle& node_handle, const std::string& planning_instance_name, unsigned int planning_instance_id);
	
	ros::NodeHandle* node_handle_;       // ROS Node handle.
	std::string planning_instance_name_; // The name of the planning instance, it is used to make sure the names of the topics / services are unique.
	unsigned int planner_instance_id_;   // The planner instance ID, it is used to make sure the action IDs are unique.
	
	// The action client that communicates with the ROS Planner.
	actionlib::SimpleActionClient<rosplan_dispatch_msgs::PlanAction>* plan_action_client_;
	
	// Topic to give commands to the planner (stop, pause, etc).
	ros::Publisher plan_command_pub_;
	
	static unsigned int total_planner_instances_;
};

};

#endif
