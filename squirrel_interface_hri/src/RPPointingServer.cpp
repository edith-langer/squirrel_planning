#include "ros/ros.h"
#include "squirrel_hri_knowledge/RPPointingServer.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <stdlib.h> 
#include <algorithm>
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h" 
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "mongodb_store/message_store.h"

#include <tf/transform_listener.h>
#include <tf/tf.h>
	
namespace KCL_rosplan {

	/* constructor */
	RPPointingServer::RPPointingServer(ros::NodeHandle &nh)
	 : message_store(nh), has_received_point_(false) {
		knowledgeInterface = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		add_waypoint_client = nh.serviceClient<rosplan_knowledge_msgs::AddWaypoint>("/kcl_rosplan/roadmap_server/add_waypoint");
		action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
		head_tilt_pub = nh.advertise<std_msgs::Float64>("/tilt_controller/command", 10, true);
		head_nod_pub = nh.advertise<std_msgs::String>("/expression", 10, true);
		head_down_angle = 0.6;
		head_up_angle = -0.3;
		count = 0;
	}
	
	void RPPointingServer::receivePointLocation(const geometry_msgs::PointStamped::ConstPtr& ptr) {
		count++;
		if(count>20) {
			received_point_ = *ptr;
			has_received_point_ = true;
			count = 0;
		}
	}
	
	/* action dispatch callback; parameters (?ob - object) */
	void RPPointingServer::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// ignore non-goto-waypoint actions
		if(0!=msg->name.compare("request_tidy")) return;

		ROS_INFO("KCL: (PointingServer) action recieved");

		// publish feedback (enabled)
		rosplan_dispatch_msgs::ActionFeedback fb;
		fb.action_id = msg->action_id;
		fb.status = "action enabled";
		action_feedback_pub.publish(fb);
		
		// get waypoint ID from action dispatch
		std::string obID;
		bool found = false;
		for(size_t i=0; i<msg->parameters.size(); i++) {
			if(0==msg->parameters[i].key.compare("ob")) {
				obID = msg->parameters[i].value;
				found = true;
			}
		}
		if(!found) {
			ROS_INFO("KCL: (PointingServer) aborting action dispatch; malformed parameters");
			return;
		}

		// tilt the head kinect up
		std_msgs::Float64 ht;
		ht.data = head_up_angle;
		head_tilt_pub.publish(ht);

		// Wait for a point to be published.
		ros::Rate r(10);
		has_received_point_ = false;
		int counter = 0;
		while (!has_received_point_ && ros::ok() && counter < 600) {
			ros::spinOnce();
			r.sleep();
			counter++;
		}
		
		if(!has_received_point_) {
			
			// tilt the head kinect down
			ht.data = head_down_angle;
			head_tilt_pub.publish(ht);
			
			// publish feedback (failed)
			fb.action_id = msg->action_id;
			fb.status = "action failed";
			action_feedback_pub.publish(fb);
			
			return;
		}

		has_received_point_ = false;
		ROS_INFO("KCL: (PointingServer) Received point");
		
		// nod the head
		std_msgs::String exp;
		exp.data = "ok";
		head_nod_pub.publish(exp);
		ros::Rate nodRate(1);
		nodRate.sleep();
		
		// tilt the head kinect down
		ht.data = head_down_angle;
		head_tilt_pub.publish(ht);
		
		// convert point to pose
		geometry_msgs::PoseStamped pose_bl, pose;
		pose_bl.header.frame_id = "/kinect_depth_optical_frame";
		pose_bl.pose.position.x = received_point_.point.x;
		pose_bl.pose.position.y = received_point_.point.y;
		pose_bl.pose.position.z = received_point_.point.z;
		pose_bl.pose.orientation.x = 0;
		pose_bl.pose.orientation.y = 0;
		pose_bl.pose.orientation.z = 0;
		pose_bl.pose.orientation.w = 1;
		
		tf::TransformListener tfl;
		try {
			tfl.waitForTransform("/map","/kinect_depth_optical_frame", ros::Time::now(), ros::Duration(1.0));
			tfl.transformPose("/map", pose_bl, pose);
		} catch ( tf::TransformException& ex ) {
			ROS_ERROR("%s: error while transforming point", ros::this_node::getName().c_str(), ex.what());
			return;
		}	

		// create new waypoint
		rosplan_knowledge_msgs::AddWaypoint addWPSrv;
		std::stringstream ss;
		ss << "point_location_" << obID;
		addWPSrv.request.id = ss.str();
		addWPSrv.request.waypoint = pose;
		addWPSrv.request.connecting_distance = 5;
		addWPSrv.request.occupancy_threshold = 20;
		if (!add_waypoint_client.call(addWPSrv)) {
			ROS_ERROR("KCL: (ObjectPerception) Failed to add a new waypoint for the object");
		}
		ROS_INFO("KCL: (ObjectPerception) Road map service returned");

		// fetch position of object from message store
		std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results;
		if(message_store.queryNamed<geometry_msgs::PoseStamped>(obID, results)) {
			if(results.size()<1) {
				ROS_INFO("KCL: (PointingServer) aborting pushing location; no matching obID %s", obID.c_str());
				// publish feedback (achieved)
				fb.action_id = msg->action_id;
				fb.status = "action achieved";
				action_feedback_pub.publish(fb);
				return;
			}
			if(results.size()>1)
				ROS_ERROR("KCL: (PointingServer) multiple waypoints share the same wpID");
		} else {
			ROS_ERROR("KCL: (PointingServer) could not query message store to fetch object pose");
			// publish feedback (achieved)
			fb.action_id = msg->action_id;
			fb.status = "action achieved";
			action_feedback_pub.publish(fb);
			return;
		}

		// add PREDICATE tidy_location
		rosplan_knowledge_msgs::KnowledgeUpdateService tlSrv;
		tlSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		tlSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		tlSrv.request.knowledge.attribute_name = "tidy_location";
		diagnostic_msgs::KeyValue object;
		object.key = "o";
		object.value = obID;
		tlSrv.request.knowledge.values.push_back(object);
		diagnostic_msgs::KeyValue location;
		location.key = "wp";
		location.value = ss.str();
		tlSrv.request.knowledge.values.push_back(location);
		if (!knowledgeInterface.call(tlSrv))
			ROS_ERROR("KCL: (PointingServer) error adding knowledge");
		
		// Remove tidy_location_unknown for this object.
		rosplan_knowledge_msgs::KnowledgeUpdateService tidyLocationUnknownSrv;
		tidyLocationUnknownSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
		tidyLocationUnknownSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		tidyLocationUnknownSrv.request.knowledge.attribute_name = "tidy_location_unknown";
		object.key = "o";
		object.value = obID;
		tidyLocationUnknownSrv.request.knowledge.values.push_back(object);
		if (!knowledgeInterface.call(tidyLocationUnknownSrv)) 
			ROS_ERROR("KCL: (PointingServer) error removing tidy_location_unknown predicate");

		// calculate pushing pose for object and new point
		geometry_msgs::PoseStamped &objPose = *results[0];
		float d = sqrt(
			(objPose.pose.position.x - pose.pose.position.x)*(objPose.pose.position.x - pose.pose.position.x) +
			(objPose.pose.position.y - pose.pose.position.y)*(objPose.pose.position.y - pose.pose.position.y));
		geometry_msgs::PoseStamped pushingPose;
		pushingPose.header.frame_id = "/map";
		float startDistance = 0.30;
		pushingPose.pose.position.x = objPose.pose.position.x + startDistance*(objPose.pose.position.x - pose.pose.position.x)/d;
		pushingPose.pose.position.y = objPose.pose.position.y + startDistance*(objPose.pose.position.y - pose.pose.position.y)/d;

		float angle = atan2(pose.pose.position.y - objPose.pose.position.y, pose.pose.position.x - objPose.pose.position.x);
		if(isnan(angle)) angle = 0;

		pushingPose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);

		// add pushing location for this object
		std::stringstream ss_pp;
		ss_pp << "push_start_location_" << obID;
		addWPSrv.request.id = ss_pp.str();
		addWPSrv.request.waypoint = pushingPose;
		addWPSrv.request.connecting_distance = 5;
		addWPSrv.request.occupancy_threshold = 20;
		if (!add_waypoint_client.call(addWPSrv)) {
			ROS_ERROR("KCL: (PointingServer) Failed to add a new waypoint for the object");
		}
		ROS_INFO("KCL: (PointingServer) Road map service returned");

		// add PREDICATE tidy_location
		rosplan_knowledge_msgs::KnowledgeUpdateService ppSrv;
		ppSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		ppSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		ppSrv.request.knowledge.attribute_name = "push_location";
		ppSrv.request.knowledge.values.push_back(object);
		location.value = ss_pp.str();
		ppSrv.request.knowledge.values.push_back(location);
		if (!knowledgeInterface.call(ppSrv))
			ROS_ERROR("KCL: (PointingServer) error adding knowledge");

		// publish feedback (achieved)
		fb.action_id = msg->action_id;
		fb.status = "action achieved";
		action_feedback_pub.publish(fb);
	}

} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

	ros::init(argc, argv, "rosplan_pointing_server");
	ros::NodeHandle nh;

	// create PDDL action subscriber
	KCL_rosplan::RPPointingServer rpps(nh);

	// listen for pointing
	ros::Subscriber pointing_pose_sub = nh.subscribe("/squirrel_person_tracker/pointing_pose", 1, &KCL_rosplan::RPPointingServer::receivePointLocation, &rpps);

	// listen for action dispatch
	ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPPointingServer::dispatchCallback, &rpps);
	ROS_INFO("KCL: (PointingServer) Ready to receive");

	while(ros::ok() && ros::master::check()){ros::spinOnce();}
	return 0;
}
