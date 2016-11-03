#include "ros/ros.h"
#include "std_msgs/String.h"
#include "squirrel_interface_perception/RPObjectPerception.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <stdlib.h> 
#include <algorithm> 
#include "mongodb_store/message_store.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/AddWaypoint.h"

namespace KCL_rosplan {

	/* constructor */
	RPObjectPerception::RPObjectPerception(ros::NodeHandle &nh, std::string &dp)
	 : message_store(nh), dataPath(dp) {

		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		add_waypoint_client = nh.serviceClient<rosplan_knowledge_msgs::AddWaypoint>("/kcl_rosplan/roadmap_server/add_waypoint");
		
		add_object_service = nh.advertiseService("/kcl_rosplan/add_object", &RPObjectPerception::addObjects, this);
		update_object_service = nh.advertiseService("/kcl_rosplan/update_object", &RPObjectPerception::updateObjects, this);
		remove_object_service = nh.advertiseService("/kcl_rosplan/remove_object", &RPObjectPerception::removeObjects, this);
	}

	/* add new object to knowledge base and scene database */
	bool RPObjectPerception::addObjects(
		squirrel_planning_knowledge_msgs::AddObjectService::Request &req,
		squirrel_planning_knowledge_msgs::AddObjectService::Response &res) {

		ROS_INFO("KCL: (RPObjectPerception::addObjects) %s", req.object.id.c_str());
		
		// store object in mongodb
		std::string mongo_id = message_store.insertNamed(req.object.id, req.object);
		mongo_id_mapping.insert(std::make_pair(req.object.id, mongo_id));
		
		// Add INSTANCE object
		rosplan_knowledge_msgs::KnowledgeUpdateService obSrv;
		obSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		obSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
		obSrv.request.knowledge.instance_type = "object";
		obSrv.request.knowledge.instance_name = req.object.id;
		if (!update_knowledge_client.call(obSrv)) {
			ROS_ERROR("KCL: (ObjectPerception) error adding knowledge");
			res.result = squirrel_planning_knowledge_msgs::AddObjectService::Response::FAILURE;
			return false;
		}

		// create new waypoint
		rosplan_knowledge_msgs::AddWaypoint addWPSrv;
		std::stringstream ss;
		ss << "wp_" << req.object.id;
		addWPSrv.request.id = ss.str();
		addWPSrv.request.waypoint.header = req.object.header;
                addWPSrv.request.waypoint.pose = req.object.pose;

		addWPSrv.request.waypoint.pose.position.z = 0;

		tf::Quaternion quat(tf::Vector3(0., 0., 1.), M_PI);
		addWPSrv.request.waypoint.pose.orientation.x = quat.x();
		addWPSrv.request.waypoint.pose.orientation.y = quat.y();
		addWPSrv.request.waypoint.pose.orientation.z = quat.z();
		addWPSrv.request.waypoint.pose.orientation.w = quat.w();

		addWPSrv.request.connecting_distance = 5;
		addWPSrv.request.occupancy_threshold = 20;
		if (!add_waypoint_client.call(addWPSrv)) {
			ROS_ERROR("KCL: (ObjectPerception) Failed to add a new waypoint for the object");
		}
		ROS_INFO("KCL: (ObjectPerception) Road map service returned");
		
		// Add PREDICATE at_object
		rosplan_knowledge_msgs::KnowledgeUpdateService atObjectSrv;
		atObjectSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		atObjectSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		atObjectSrv.request.knowledge.attribute_name = "object_at";
		diagnostic_msgs::KeyValue oPair;
		oPair.key = "o";
		oPair.value = req.object.id;
		atObjectSrv.request.knowledge.values.push_back(oPair);
		oPair.key = "wp";
		oPair.value = ss.str();
		atObjectSrv.request.knowledge.values.push_back(oPair);
		if (!update_knowledge_client.call(atObjectSrv)) {
			ROS_ERROR("KCL: (ObjectPerception) error adding object_at predicate");
			res.result = squirrel_planning_knowledge_msgs::AddObjectService::Response::FAILURE;
			return false;
		}
		
		// Add PREDICATE tidy_location_unknown
		rosplan_knowledge_msgs::KnowledgeUpdateService tidyLocationUnknownSrv;
		tidyLocationUnknownSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		tidyLocationUnknownSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		tidyLocationUnknownSrv.request.knowledge.attribute_name = "tidy_location_unknown";
		oPair.key = "o";
		oPair.value = req.object.id;
		tidyLocationUnknownSrv.request.knowledge.values.push_back(oPair);
		if (!update_knowledge_client.call(tidyLocationUnknownSrv)) {
			ROS_ERROR("KCL: (ObjectPerception) error adding tidy_location_unknown predicate");
			res.result = squirrel_planning_knowledge_msgs::AddObjectService::Response::FAILURE;
			return false;
		}
		
		// Add GOAL tidy_object
		rosplan_knowledge_msgs::KnowledgeUpdateService toSrv;
		toSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
		toSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		toSrv.request.knowledge.attribute_name = "tidy";
		oPair.key = "o";
		oPair.value = req.object.id;
		toSrv.request.knowledge.values.push_back(oPair);
		if (!update_knowledge_client.call(toSrv)) {
			ROS_ERROR("KCL: (ObjectPerception) error adding tidy goal");
			res.result = squirrel_planning_knowledge_msgs::AddObjectService::Response::FAILURE;
			return false;
		}

		res.result = squirrel_planning_knowledge_msgs::AddObjectService::Response::SUCCESS;
		return true;
	}

	/* remove object from knowledge base and scene database */
	bool RPObjectPerception::removeObjects(
		squirrel_planning_knowledge_msgs::RemoveObjectService::Request &req,
		squirrel_planning_knowledge_msgs::RemoveObjectService::Response &res) {
		
		// Remove all data associated with the given id.
		std::pair<std::multimap<std::string, std::string>::iterator, std::multimap<std::string, std::string>::iterator> mm_ci;
		mm_ci = mongo_id_mapping.equal_range(req.id);
		for (std::multimap<std::string, std::string>::const_iterator ci = mm_ci.first; ci != mm_ci.second; ++ci) {
			message_store.deleteID((*ci).second);
		}
		
		// Remove the mappings from knowledge base.
		rosplan_knowledge_msgs::KnowledgeUpdateService wpSrv;
		wpSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
		wpSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
		wpSrv.request.knowledge.instance_type = "object";
		wpSrv.request.knowledge.instance_name = req.id;
		if (!update_knowledge_client.call(wpSrv)) {
			ROS_ERROR("KCL: (ObjectPerception) error removing knowledge");
			res.result = squirrel_planning_knowledge_msgs::RemoveObjectService::Response::FAILURE;
			return false;
		}
		
		res.result = squirrel_planning_knowledge_msgs::RemoveObjectService::Response::SUCCESS;
		return true;
	}

	/* update object in scene database and recalculate for knowledge base */
	bool RPObjectPerception::updateObjects(
		squirrel_planning_knowledge_msgs::UpdateObjectService::Request &req,
		squirrel_planning_knowledge_msgs::UpdateObjectService::Response &res) {
#if 0
		// Update all data associated with the given id -- the types might be swapped, but this should only affect performance.
		std::pair<std::multimap<std::string, std::string>::iterator, std::multimap<std::string, std::string>::iterator> mm_ci;
		mm_ci = mongo_id_mapping.equal_range(req.object.id);
		
		// We expect exactly one instance in the DB, anything else indicates a failure.
		int found_instances = std::distance(mm_ci.first, mm_ci.second);
		if (found_instances != 1) {
			ROS_ERROR("KCL: (ObjectPerception) error updating, we expected mongodb to contain 1 instance for id %s, instead we found %d.", req.id.c_str(), found_instances);
			res.result = squirrel_planning_knowledge_msgs::UpdateObjectService::Response::FAILURE;
			return false;
		}
		
		std::multimap<std::string, std::string>::const_iterator update_ci = mm_ci.first;
		message_store.updateNamed((*update_ci).second, req.object); ++update_ci;

		std_msgs::String ros_string_category;
		ros_string_category.data = req.category;
		message_store.updateNamed((*update_ci).second, ros_string_category); ++update_ci;
		message_store.updateNamed((*update_ci).second, req.pose);
		
		// update waypoint in mongodb
		std::stringstream ss;
		ss << req.id << "_wp";
		message_store.updateNamed(ss.str(), req.pose);
#endif
		// TODO ensure that updating waypoint position is OK
		res.result = squirrel_planning_knowledge_msgs::UpdateObjectService::Response::SUCCESS;
		return true;
	}

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		// setup ros
		ros::init(argc, argv, "rosplan_object_perception");
		ros::NodeHandle nh("~");

		// default config
		std::string dataPath = "common/";
		nh.param("data_path", dataPath, dataPath);

		// init services
		KCL_rosplan::RPObjectPerception rms(nh, dataPath);

		ROS_INFO("KCL: (RPObjectPerception) Ready to receive");
		while(ros::ok() && ros::master::check()){ros::spinOnce();}
		return 0;
	}
