#include "squirrel_hri_knowledge/PerformSocialBehaviour.h"
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <squirrel_vad_msgs/vad.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float64.h>
#include <squirrel_hri_msgs/Expression.h>
#include <squirrel_interaction/BaseLights.h>
#include <squirrel_view_controller_msgs/LookAtPosition.h>

namespace KCL_rosplan {

	/* constructor */
	PerformSocialBehaviour::PerformSocialBehaviour(ros::NodeHandle &nh, const std::string& move_base_action_name)
		 : message_store(nh), arousal_threshold(0.25f), action_client(move_base_action_name), has_received_pointing_location_(false), head_down_angle_(-0.3), head_up_angle_(0.3), current_arousal(-1)
	{
		knowledgeInterface = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
		
		lights_pub_ = nh.advertise<std_msgs::ColorRGBA>("/light/command", 1, true);
		lights_complex_pub_ = nh.advertise<squirrel_interaction::BaseLights>("/light_complex/command", 1, true);
		expression_pub_ = nh.advertise<std_msgs::String>("/expression", 1, true);

		arousal_sub_ = nh.subscribe("/cobotnity_arousal", 1, &PerformSocialBehaviour::getArousal, this);
		point_pose_sub_ = nh.subscribe("/squirrel_person_tracker/pointing_pose", 1, &PerformSocialBehaviour::getPointingPose, this);
		
		// Set controls for the neck and head.
		neck_tilt_pub_ = nh.advertise<std_msgs::Float64>("/neck_tilt_controller/command", 10, true);
		//head_nod_pub_ = nh.advertise<std_msgs::String>("/expression", 10, true);
		//head_pan_pub_ = nh.advertise<std_msgs::Float64>("/head_controller/command", 10, true);
		//neck_pan_pub_ = nh.advertise<std_msgs::Float64>("/neck_pan_controller/command", 10, true);
		view_controller_client_ = nh.serviceClient<squirrel_view_controller_msgs::LookAtPosition>("/squirrel_view_controller/look_at_position");

		nh.getParam("arousal_threshold", arousal_threshold);
		ROS_INFO("KCL: (PerformSocialBehaviour) Arousal threshold set at: %f", arousal_threshold);
	}

void PerformSocialBehaviour::getArousal(const std_msgs::Float32::ConstPtr& msg)
{
//	ROS_INFO("KCL: (PerformSocialBehaviour) New arousal value read. It is currently: %f", arousal_threshold);
	current_arousal = msg->data;
}

void PerformSocialBehaviour::getPointingPose(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	child_pointing_location_ = *msg;
	has_received_pointing_location_ = true;
}

void PerformSocialBehaviour::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
{
		// ignore other actions
		if (msg->name != "accomodate-distress" &&
		    msg->name != "improve-distress" &&
		    msg->name != "accomodate-sadness" &&
		    msg->name != "improve-sadness" &&
		    msg->name != "improve-boredom" &&
		    msg->name != "maintain-happyness" &&
		    msg->name != "improve-introvert" &&
		    msg->name != "reciprocal-behaviour")
		{
			return;
		}

		ROS_INFO("KCL: (PerformSocialBehaviour) action recieved");
		
		// publish feedback (enabled)
		rosplan_dispatch_msgs::ActionFeedback fb;
		fb.action_id = msg->action_id;
		fb.status = "action enabled";
		action_feedback_pub.publish(fb);
		
/*
		// Get the latest arausal level.
		std::vector< boost::shared_ptr<squirrel_vad_msgs::vad> > results;
		message_store.queryNamed<squirrel_vad_msgs::vad>("vad", results);
		
		if (results.empty())
		{
			ROS_ERROR("KCL: (perform_social_behaviour) Could not receive the arausal level from the message store.");
			exit(1);
		}
		
		// Depending on the level we perform different social behaviour.
		float arousal = results[0]->energy;
*/


/*
		int current_state = -1;
		for (unsigned int i = 0; i < 100; ++i)
		{
			ros::spinOnce();
			ROS_INFO("KCL: (PerformSocialBehaviour) arousal is: %f, threshold is set at %f", current_arousal, arousal_threshold);
			
			std_msgs::ColorRGBA color_command;
			
			// No arousal.
			if (current_arousal == 0.0)
			{
				// Then look at the children.
				// @todo Talk to Bajo.
				
				float red = 25;
				float green = 240;
				float blue = 50;
				ROS_INFO("KCL: (ShowLightsAction) Parsed the lights, showing (%f, %f, %f)", red, green, blue);
				
				color_command.r = red;
				color_command.g = green;
				color_command.b = blue;
				color_command.a = 255;
				lights_pub_.publish(color_command);
				
				if (i == 0 || current_state != 0)
				{
					std_msgs::String sound_command;
					sound_command.data = squirrel_hri_msgs::Expression::HELLO;
					expression_pub_.publish(sound_command);
					current_state = 0;
				}
			}
			// Low arousal.
			else if (current_arousal < arousal_threshold)
			{
				// Go to some random location.
				float red = 145 * (current_arousal / 0.25);
				float green = 174 * (current_arousal / 0.25);
				float blue = 182;
				ROS_INFO("KCL: (ShowLightsAction) Parsed the lights, showing (%f, %f, %f)", red, green, blue);
				
				color_command.r = red;
				color_command.g = green;
				color_command.b = blue;
				color_command.a = 255;
				lights_pub_.publish(color_command);
				
				if (i == 0 || current_state != 1)
				{
					std_msgs::String sound_command;
					sound_command.data = squirrel_hri_msgs::Expression::GOODBYE;
					expression_pub_.publish(sound_command);
					current_state = 1;
				}
			}
			// High arousal.
			else
			{
				// Make some noise!
				float red = 255;
				float green = 124 * current_arousal;
				float blue = 150 * current_arousal;
				ROS_INFO("KCL: (ShowLightsAction) Parsed the lights, showing (%f, %f, %f)", red, green, blue);
				
				color_command.r = red;
				color_command.g = green;
				color_command.b = blue;
				color_command.a = 255;
				lights_pub_.publish(color_command);
				
				if (i == 0 || current_state != 2)
				{
					std_msgs::String sound_command;
					sound_command.data = squirrel_hri_msgs::Expression::CHEERING;
					expression_pub_.publish(sound_command);
					current_state = 2;
				}
			}
			
			// get waypoint ID from action dispatch
			std::string childID;
			bool found = false;
			for(size_t i=0; i<msg->parameters.size(); i++) {
				if(0==msg->parameters[i].key.compare("c")) {
					childID = msg->parameters[i].value;
					found = true;
				}
			}
			if(!found) {
				ROS_INFO("KCL: (PerformSocialBehaviour) aborting action dispatch; malformed parameters");
				return;
			}

			ros::Duration(0.1).sleep();
		}
*/
		
		// publish feedback (achieved)
		fb.action_id = msg->action_id;
		fb.status = "action achieved";
		action_feedback_pub.publish(fb);
	}


	void PerformSocialBehaviour::displayRainbow(float delay, float duration)
	{
		// Fill in the lights.
		const int total_lights = 42;

		float time = 0;
		unsigned int i = 0;
		/** Display some dazzeling lights! **/
		while (time < duration)
		{
			ros::Duration(delay).sleep();
			time += delay;


			squirrel_interaction::BaseLights bs;
			float delta = 255.0 / (total_lights / 3.0);
//			bs.colors.reserve(42 * 3);
			for (unsigned int j = 0; j < total_lights / 3; ++j)
			{
				bs.colors[(j + i) % (total_lights * 3)] = j * delta;
				bs.colors[(j + 1 + i) % (total_lights * 3)] = 255 - j * delta;
				bs.colors[(j + 2 + i) % (total_lights * 3)] = 0;
			}
			for (unsigned int j = 0; j < total_lights / 3; ++j)
			{
				bs.colors[(j + total_lights + i) % (total_lights * 3)] = 255 - j * delta;
				bs.colors[(j + 1 + total_lights + i) % (total_lights * 3)] = 0;
				bs.colors[(j + 2 + total_lights + i) % (total_lights * 3)] = j * delta;
			}
			for (unsigned int j = 0; j < total_lights / 3; ++j)
			{
				bs.colors[(j + total_lights * 2 + i) % (total_lights * 3)] = 0;
				bs.colors[(j + 2 + total_lights * 2 + i) % (total_lights * 3)] = j * delta;
				bs.colors[(j + 1 + total_lights * 2 + i) % (total_lights * 3)] = 255 - j * delta;
			}
			lights_complex_pub_.publish(bs);
			ros::spinOnce();
			i += 3;
		}
	}

	void PerformSocialBehaviour::displayLights(unsigned int r, unsigned int g, unsigned int b)
	{
		std_msgs::ColorRGBA color_command;
		color_command.r = r;
		color_command.g = g;
		color_command.b = b;
		color_command.a = 255;
		lights_pub_.publish(color_command);
	}

	void PerformSocialBehaviour::performDeicticGaze(const geometry_msgs::PoseStamped& p)
	{
		ROS_INFO("KCL: (PerformSocialBehaviour) Turn neck and head to where the object of interest is.");

		float x = p.pose.position.x;
		float y = p.pose.position.y;
		squirrel_view_controller_msgs::LookAtPosition lap;

		lap.request.target = p;
		lap.request.reason = "Look at the object of interest.";
		if (!view_controller_client_.call(lap))
		{
			ROS_ERROR("KCL: (PerformSocialBehaviour) Could not call the Look At Position service.");
			exit(-1);
		}
		displayLights(0, 1, 0);
		ros::Duration(0.5).sleep();

		unsigned int i = 0;
		do
		{
			// Wait for a bit and look away.
			float x = lap.request.target.pose.position.x;
			lap.request.target.pose.position.x = -y;
			lap.request.target.pose.position.y = x;
			if (!view_controller_client_.call(lap))
			{
				ROS_ERROR("KCL: (PerformSocialBehaviour) Could not call the Look At Position service.");
				exit(-1);
			}

			// Wait for a bit and look at the object again.
			displayLights(1, 1, 0);
			ros::Duration(0.1).sleep();
			lap.request.target = p;
			lap.request.reason = "Look at the object of interest.";
			if (!view_controller_client_.call(lap))
			{
				ROS_ERROR("KCL: (PerformSocialBehaviour) Could not call the Look At Position service.");
				exit(-1);
			}
			displayLights(0, 1, 0);
			ros::Duration(0.2).sleep();
			++i;
		}
		while(i < 2);
	}

	void PerformSocialBehaviour::performSocialGaze()
	{
		ROS_INFO("KCL: (PerformSocialBehaviour) Performing social gaze, waiting for point... Neck tilted up %f", head_up_angle_);
		// Fetch the orientaiton of the head, relative to the position the child is pointing at.
		// tilt the head kinect up
		std_msgs::Float64 ht;
		ht.data = head_up_angle_;
		neck_tilt_pub_.publish(ht);

		// Wait for a point to be published.
		ros::Rate r(10);
		has_received_pointing_location_ = false;
		while (!has_received_pointing_location_ && ros::ok()) {
			ros::spinOnce();
			r.sleep();
		}
		has_received_pointing_location_ = false;
		ROS_INFO("KCL: (PerformSocialBehaviour) Received point");

		// nod the head
		ROS_INFO("KCL: (PerformSocialBehaviour) Nodding the head");
		std_msgs::String exp;
		exp.data = "ok";
		expression_pub_.publish(exp);
		ros::Rate nodRate(1);
		nodRate.sleep();

		// tilt the head kinect down
		ROS_INFO("KCL: (PerformSocialBehaviour) Tilting neck down %f", head_down_angle_);
		ht.data = head_down_angle_;
		neck_tilt_pub_.publish(ht);

		// convert point to pose
		geometry_msgs::PoseStamped pose_bl;
		pose_bl.header.frame_id = "/kinect_depth_optical_frame";
		pose_bl.pose.position.x = child_pointing_location_.point.x;
		pose_bl.pose.position.y = child_pointing_location_.point.y;
		pose_bl.pose.position.z = child_pointing_location_.point.z;
		pose_bl.pose.orientation.x = 0;
		pose_bl.pose.orientation.y = 0;
		pose_bl.pose.orientation.z = 0;
		pose_bl.pose.orientation.w = 1;

		ROS_INFO("KCL: (PerformSocialBehaviour) Turn neck and head to where the child is pointing.");
		squirrel_view_controller_msgs::LookAtPosition lap;
		lap.request.target = pose_bl;
		lap.request.reason = "Look at where the child is pointing.";
		if (!view_controller_client_.call(lap))
		{
			ROS_ERROR("KCL: (PerformSocialBehaviour) Could not call the Look At Position service.");
			exit(-1);
		}


		// tilt the kinect back to its neutral position.
		ROS_INFO("KCL: (PerformSocialBehaviour) Tilt camera to neutral position: 0.0");
		ht.data = 0.0;
		neck_tilt_pub_.publish(ht);

		// Turn the head and camera to face where the child is pointing.
		ROS_INFO("KCL: (PerformSocialBehaviour) Turn neck and head to where the child is pointing.");
		
		/** Display some dazzeling lights! **/
		displayRainbow(0.01f, 10.0f);
	}
};


/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

	ros::init(argc, argv, "rosplan_perform_social_behaviour");
	ros::NodeHandle nh;
	
	// listen for action dispatch
	KCL_rosplan::PerformSocialBehaviour psb(nh, "/move");
	ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::PerformSocialBehaviour::dispatchCallback, &psb);
	ROS_INFO("KCL: (PerformSocialBehaviour) Ready to receive");

	while (true)
	{
		ROS_INFO("KCL: (PerformSocialBehaviour) Perform Social gaze...");
		ros::spinOnce();
		psb.performSocialGaze();
		geometry_msgs::PoseStamped p;
		p.header.frame_id = "/map";
		p.pose.position.x = ((float)rand() / (float)RAND_MAX - 0.5f) * 5.0f;
		p.pose.position.y = ((float)rand() / (float)RAND_MAX - 0.5f) * 5.0f;
		p.pose.position.z = ((float)rand() / (float)RAND_MAX - 0.5f) * 5.0f;
		p.pose.orientation.x = 0;
		p.pose.orientation.y = 0;
		p.pose.orientation.z = 0;
		p.pose.orientation.w = 1;

		ROS_INFO("KCL: (PerformSocialBehaviour) Perform Deictic gaze to: (%f, %f, %f...", p.pose.position.x, p.pose.position.y, p.pose.position.z);
		psb.performDeicticGaze(p);
	}
	return 0;
}

