#include "PlannerInstance.h"

#include <std_msgs/String.h>

namespace KCL_rosplan
{

unsigned int PlannerInstance::total_planner_instances_ = 0;
	
PlannerInstance& PlannerInstance::createInstance(ros::NodeHandle& node_handle)
{
	++total_planner_instances_;
	
	// Create a new planning system.
	std::stringstream nspace;
	nspace << "instance" << total_planner_instances_;
	
	std::stringstream commandLine;
	commandLine << "rosrun rosplan_planning_system planner ";
	commandLine << "/rosplan_planning_system:=/" << nspace.str() << "/rosplan_planning_system ";
	commandLine << "/kcl_rosplan/plan:=/kcl_rosplan/" << nspace.str() << "/plan ";
	commandLine << "/kcl_rosplan/system_state:=/kcl_rosplan/" << nspace.str() << "/system_state ";
	commandLine << "/kcl_rosplan/planning_commands:=/kcl_rosplan/" << nspace.str() << "/planning_commands ";
	commandLine << "/kcl_rosplan/planning_server:=/kcl_rosplan/" << nspace.str() << "/planning_server ";
	commandLine << "/kcl_rosplan/planning_server_params:=/kcl_rosplan/" << nspace.str() << "/planning_server_params ";
	commandLine << "/kcl_rosplan/start_planning:=/kcl_rosplan/" << nspace.str() << "/start_planning ";
	commandLine << "&";
	int return_value = system(commandLine.str().c_str());

	PlannerInstance* planning_instance = new PlannerInstance(node_handle, nspace.str(), total_planner_instances_);
	return *planning_instance;
}

PlannerInstance::PlannerInstance(ros::NodeHandle& node_handle, const std::string& planning_instance_name, unsigned int planner_instance_id)
	: node_handle_(&node_handle), planning_instance_name_(planning_instance_name), planner_instance_id_(planner_instance_id)
{
	// Create action client
	std::stringstream commandPub;
	commandPub << "/kcl_rosplan/" << planning_instance_name << "/start_planning";
	plan_action_client_ = new actionlib::SimpleActionClient<rosplan_dispatch_msgs::PlanAction>(commandPub.str(), true);
	ROS_INFO("KCL: (PlannerInstance) Waiting for action server to start.");
	plan_action_client_->waitForServer();
	ROS_INFO("KCL: (PlannerInstance) Action server started.");
	
	std::stringstream ss;
	ss << "/kcl_rosplan/" << planning_instance_name << "/planning_commands";
	plan_command_pub_ = node_handle.advertise<std_msgs::String>(ss.str(), 1);
}

PlannerInstance::~PlannerInstance()
{
	delete plan_action_client_;
}

void PlannerInstance::startPlanner(const std::string& domain_path, const std::string& problem_path, const std::string& data_path, const std::string& planner_command)
{
	rosplan_dispatch_msgs::PlanGoal psrv;
	psrv.domain_path = domain_path;
	psrv.problem_path = problem_path;
	psrv.data_path = data_path;
	psrv.planner_command = planner_command;
	psrv.start_action_id = total_planner_instances_ * 1000;
	
	plan_action_client_->sendGoal(psrv);
}

void PlannerInstance::stopPlanner()
{
	std_msgs::String command;
	command.data = "cancel";
	plan_command_pub_.publish(command);
}

actionlib::SimpleClientGoalState PlannerInstance::getState() const
{
	return plan_action_client_->getState();
}

};
