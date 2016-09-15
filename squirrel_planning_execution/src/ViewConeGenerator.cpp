#include <iostream>

#include <squirrel_planning_execution/ViewConeGenerator.h>
#include <occupancy_grid_utils/ray_tracer.h>
#include <occupancy_grid_utils/coordinate_conversions.h>
#include <visualization_msgs/MarkerArray.h>
//#include <tf/Quaternion.h>
//#include <tf/Vector3.h>

#include <stdlib.h>
#include <time.h> 

namespace KCL_rosplan {

ViewConeGenerator::ViewConeGenerator(ros::NodeHandle& node_handle, const std::string& topic_name)
	: has_received_occupancy_grid_(false)
{
	navigation_grid_sub_ = node_handle.subscribe(topic_name, 1, &ViewConeGenerator::storeNavigationGrid, this);
	rivz_pub_ = node_handle.advertise<visualization_msgs::MarkerArray>("/vis/view_cones", 1000);
	srand(time(NULL));
}

void ViewConeGenerator::storeNavigationGrid(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	last_received_occupancy_grid_msgs_ = *msg;
	has_received_occupancy_grid_ = true;
}

void ViewConeGenerator::createViewCones(std::vector<geometry_msgs::Pose>& poses, const std::vector<tf::Vector3>& bounding_box, unsigned int max_view_cones, int occupancy_threshold, float fov, float view_distance, unsigned int sample_size, float safe_distance)
{
	if (!has_received_occupancy_grid_) {
		ROS_WARN("(ViewConeGenerator) The occupancy grid was not published yet, no poses returned.");
		return;
	}
	
	ROS_INFO("(ViewConeGenerator) View code generation started %d. Occupancy grid size: (%d, %d), cell size: %f", max_view_cones, last_received_occupancy_grid_msgs_.info.width, last_received_occupancy_grid_msgs_.info.height, last_received_occupancy_grid_msgs_.info.resolution);
	// Initialise the processed cells list.
	std::vector<bool> processed_cells(last_received_occupancy_grid_msgs_.info.width * last_received_occupancy_grid_msgs_.info.height, false);
	for (int y = 0; y < last_received_occupancy_grid_msgs_.info.height; ++y) {
		for (int x = 0; x < last_received_occupancy_grid_msgs_.info.width; ++x) {
			if (last_received_occupancy_grid_msgs_.data[x + y * last_received_occupancy_grid_msgs_.info.width] > occupancy_threshold ||
			    last_received_occupancy_grid_msgs_.data[x + y * last_received_occupancy_grid_msgs_.info.width] == -1) {
				processed_cells[x + y * last_received_occupancy_grid_msgs_.info.width] = true;
			} else {
				processed_cells[x + y * last_received_occupancy_grid_msgs_.info.width] = false;
			}
		}
	}
	
	ROS_INFO("(ViewConeGenerator) Initialised the processed cells.");
	
	for (unsigned int i = 0; i < max_view_cones; ++i) {
		ROS_INFO("(ViewConeGenerator) Process view cone: %d.", i);
		// First we generate a bunch of random view cones and rate them.
		geometry_msgs::Pose best_pose;
		std::vector<occupancy_grid_utils::Cell> best_visible_cells;
		for (unsigned int j = 0; j < sample_size; ++j) {
			int grid_x = ((float)rand() / (float)RAND_MAX) * last_received_occupancy_grid_msgs_.info.width;
			int grid_y = ((float)rand() / (float)RAND_MAX) * last_received_occupancy_grid_msgs_.info.height;
			
			occupancy_grid_utils::Cell c(grid_x, grid_y);
			
			geometry_msgs::Point p = occupancy_grid_utils::cellCenter(last_received_occupancy_grid_msgs_.info, c);
			
			// Check if this point falls within the bounding box.
			{
				bool falls_within_bounded_box = true;
				char sign = 0;
				tf::Vector3 cell_point(p.x, p.y, p.z);
				for (int i = 0; i < bounding_box.size(); ++i)
				{
					const tf::Vector3& v1 = bounding_box[i];
					const tf::Vector3& v2 = bounding_box[(i + 1) % bounding_box.size()];
					
					tf::Vector3 cross_product = (cell_point - v1).cross(v2 - v1);
					
					if (sign == 0)
					{
						sign = cross_product.z() > 0 ? 1 : -1;
					}
					else
					{
						if (sign == -1 && cross_product.z() > 0 ||
							 sign == 1 && cross_product.z() < 0)
						{
							falls_within_bounded_box = false;
							break;
						}
					}
				}
				
				if (!falls_within_bounded_box)
				{
					//std::cout << "b(" << p.x << "," << p.y <<")";
					continue;
				}
			}
			
			// Check if this cell point is not too close to any obstacles.
			if (isBlocked(p, safe_distance)) {
				std::cout << "#";
				continue;
			}
			
			float yaw = ((float)rand() / (float)RAND_MAX) * M_PI;
			if ((float)rand() / (float)RAND_MAX < 0.5f) yaw = -yaw;
			
			//ROS_INFO("(ViewConeGenerator) Sample cone: (%d, %d) %f.", grid_x, grid_y, yaw);
			//ROS_INFO("(ViewConeGenerator) Sample cone: (%f, %f) %f.", p.x, p.y, yaw);
			
			geometry_msgs::Pose pose;
			pose.position = p;
			
			tf::Quaternion q;
			q.setEulerZYX(yaw, 0.0f, 0.0f);

			pose.orientation.x = q.getX();
			pose.orientation.y = q.getY();
			pose.orientation.z = q.getZ();
			pose.orientation.w = q.getW();
			
			// Calculate the triangle points encompasses the area that is viewed.
			tf::Vector3 view_point(p.x, p.y, p.z);
			
			tf::Vector3 v0(view_distance, 0.0f, 0.0f);
			v0 = v0.rotate(tf::Vector3(0.0f, 0.0f, 1.0f), yaw);
			
			//ROS_INFO("(ViewConeGenerator) Viewing direction: (%f, %f, %f).", v0.x(), v0.y(), v0.z());
			
			tf::Vector3 v0_normalised = v0.normalized();
			
			//ROS_INFO("(ViewConeGenerator) Viewing direction (normalised): (%f, %f, %f).", v0_normalised.x(), v0_normalised.y(), v0_normalised.z());
			
			tf::Vector3 v1 = v0;
			v1 = v1.rotate(tf::Vector3(0.0f, 0.0f, 1.0f), fov / 2.0f);
			v1 = v1.normalize();
			
			//ROS_INFO("(ViewConeGenerator) V1 (normalised): (%f, %f, %f).", v1.x(), v1.y(), v1.z());
			
			float length = v0.length() / v0_normalised.dot(v1);
			v1 *= length;
			v1 += view_point;
			
			//ROS_INFO("(ViewConeGenerator) V1 (actual): (%f, %f, %f); length = %f.", v1.x(), v1.y(), v1.z(), length);
			
			tf::Vector3 v2 = v0;
			v2 = v2.rotate(tf::Vector3(0.0f, 0.0f, 1.0f), -fov / 2.0f);
			v2 = v2.normalize();
			//ROS_INFO("(ViewConeGenerator) V2 (normalised): (%f, %f, %f).", v2.x(), v2.y(), v2.z());
			
			length = v0.length() / v0_normalised.dot(v2);
			v2 *= length;
			v2 += view_point;
			//ROS_INFO("(ViewConeGenerator) V2 (actual): (%f, %f, %f); length = %f.", v2.x(), v2.y(), v2.z(), length);
			
			//ROS_INFO("(ViewConeGenerator) Triangle: (%f, %f, %f), (%f, %f, %f), (%f, %f, %f).", p.x, p.y, p.z, v1.x(), v1.y(), v1.z(), v2.x(), v2.y(), v2.z());
			
			// The triangle now is view_point, v1, v2, we use a flood algorithm to determine which cells
			// are inside the viewing cone.
			//occupancy_grid_utils::Cell cell = occupancy_grid_utils::pointCell(last_received_occupancy_grid_msgs_.info, );
			
			std::vector<occupancy_grid_utils::Cell> open_list;
			open_list.push_back(c);
			
			std::vector<occupancy_grid_utils::Cell> complete_list;
			
			while (open_list.size() > 0 && ros::ok()) {
				
				occupancy_grid_utils::Cell cell = open_list[0];
				open_list.erase(open_list.begin());
				
				// Make sure this cell was not already added.
				bool has_been_processed = false;
				for (std::vector<occupancy_grid_utils::Cell>::const_iterator ci = complete_list.begin(); ci != complete_list.end(); ++ci) {
					const occupancy_grid_utils::Cell& completed_cell = *ci;
					if (completed_cell.x == cell.x && completed_cell.y == cell.y) {
						has_been_processed = true;
						break;
					}
				}
				
				if (has_been_processed) {
					//std::cout << "\tHas already been processed, move on!" << std::endl;
					continue;
				}
				
				//std::cout << "[" << open_list.size() << ", " << complete_list.size() << "; max=" << last_received_occupancy_grid_msgs_.info.width * last_received_occupancy_grid_msgs_.info.height << "] Process: (" << cell.x << ", " << cell.y << ")" << std::endl;
				
				// Check if the cell is inside the triangle.
				geometry_msgs::Point cell_centre_point = occupancy_grid_utils::cellCenter(last_received_occupancy_grid_msgs_.info, cell);
				tf::Vector3 cell_point(cell_centre_point.x, cell_centre_point.y, cell_centre_point.z);
				
				tf::Vector3 cross_v1 = (cell_point - v1).cross(v2 - v1);
				tf::Vector3 cross_v2 = (cell_point - v2).cross(view_point - v2);
				tf::Vector3 cross_v3 = (cell_point - view_point).cross(v1 - view_point);
				
				// If both cross produces have the same sign we are good.
				bool is_in_triangle = false;
				if ((cross_v1.z() > 0 && cross_v2.z() > 0 && cross_v3.z() > 0) ||
				    (cross_v1.z() < 0 && cross_v2.z() < 0 && cross_v3.z() < 0) ||
				    (cell.x == c.x && cell.y == c.y)) {
					is_in_triangle = true;
				}
				
				if (!is_in_triangle) {
					//std::cout << "\tIs not inside the triangle!" << std::endl;
					continue;
				}
				
				complete_list.push_back(cell);
				
				// Add its children to the list.
				occupancy_grid_utils::Cell new_cell;
				for (int x = cell.x - 1; x < cell.x + 2; ++x) {
					for (int y = cell.y - 1; y < cell.y + 2; ++y) {
						if (x > -1 && x < last_received_occupancy_grid_msgs_.info.width &&
						    y > -1 && y < last_received_occupancy_grid_msgs_.info.height)
						{
							new_cell.x = x;
							new_cell.y = y;
							open_list.push_back(new_cell);
						}
					}
				}
			}
			
			//ROS_INFO("(ViewConeGenerator) Finished flood algorithm, %d cells in view.", complete_list.size());
			
			// Next we determine which of these cell points are visible from 'view_point'.
			std::vector<occupancy_grid_utils::Cell> visible_cells;
			for (std::vector<occupancy_grid_utils::Cell>::const_iterator ci = complete_list.begin(); ci != complete_list.end(); ++ci) {
				
				const occupancy_grid_utils::Cell& cell = *ci;
				// Don't count cells that have already been processed.
				if (processed_cells[cell.x + cell.y * last_received_occupancy_grid_msgs_.info.width]) {
					continue;
				}
				
				geometry_msgs::Point point = occupancy_grid_utils::cellCenter(last_received_occupancy_grid_msgs_.info, cell);
				
				if (canConnect(point, p, occupancy_threshold)) {
					visible_cells.push_back(cell);
				}
			}
			
			//ROS_INFO("(ViewConeGenerator) Finished checking visibility, %d cells actually visible.", visible_cells.size());
			
			// Check if this is the best view cone thus far.
			if (visible_cells.size() > best_visible_cells.size()) {
				best_pose = pose;
				best_visible_cells = visible_cells;
				
				tf::Quaternion q(best_pose.orientation.x, best_pose.orientation.y, best_pose.orientation.z, best_pose.orientation.w);
				float best_yaw = tf::getYaw(q);
				
				//ROS_INFO("(ViewConeGenerator) Best new pose(%f, %f, %f), yaw=%f (actual=%f) with %d cells.", best_pose.position.x, best_pose.position.y, best_pose.position.z, best_yaw, yaw, visible_cells.size());
			}
			else {
				std::cout << "-";
			}
		}
		
		std::cout << std::endl;
		
		if (best_visible_cells.empty()) {
			ROS_INFO("(ViewConeGenerator) No good poses found!");
			continue;
		}
		
		// Update the state of which cells have been observed.
		for (std::vector<occupancy_grid_utils::Cell>::const_iterator ci = best_visible_cells.begin(); ci != best_visible_cells.end(); ++ci) {
			const occupancy_grid_utils::Cell& cell = *ci;
			processed_cells[cell.x + cell.y * last_received_occupancy_grid_msgs_.info.width] = true;
		}
		
		tf::Quaternion q(best_pose.orientation.x, best_pose.orientation.y, best_pose.orientation.z, best_pose.orientation.w);
		float yaw = tf::getYaw(q);
		
		ROS_INFO("(ViewConeGenerator) Add the pose(%f, %f, %f), yaw=%f with %d cells to the return list.", best_pose.position.x, best_pose.position.y, best_pose.position.z, yaw, best_visible_cells.size());
		poses.push_back(best_pose);
	}
	
	for (std::vector<geometry_msgs::Pose>::const_iterator ci = poses.begin(); ci != poses.end(); ++ci) {
		ROS_INFO("(ViewConeGenerator) Found the pose(%f, %f, %f).", (*ci).position.x, (*ci).position.y, (*ci).position.z);
	}
	
	// Visualise the view cones.
	visualiseViewCones(poses, view_distance, fov);
}

void ViewConeGenerator::visualiseViewCones(const std::vector<geometry_msgs::Pose>& poses, float view_distance, float fov) const
{
	std::vector<geometry_msgs::Point> waypoints;
	std::vector<std_msgs::ColorRGBA> waypoint_colours;
	std::vector<geometry_msgs::Point> triangle_points;
	std::vector<std_msgs::ColorRGBA> triangle_colours;
	
	for (std::vector<geometry_msgs::Pose>::const_iterator ci = poses.begin(); ci != poses.end(); ++ci) {
		const geometry_msgs::Pose& pose = *ci;
		
		// Create a triangle out of this pose.
		tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
		float yaw = tf::getYaw(q);
		
		//ROS_INFO("(ViewConeGenerator) Yaw: %f.", yaw);
		
		// Calculate the triangle points encompasses the area that is viewed.
		tf::Vector3 view_point(pose.position.x, pose.position.y, pose.position.z);
		
		tf::Vector3 v0(view_distance, 0.0f, 0.0f);
		v0 = v0.rotate(tf::Vector3(0.0f, 0.0f, 1.0f), yaw);
		
		//ROS_INFO("(ViewConeGenerator) Viewing direction: (%f, %f, %f).", v0.x(), v0.y(), v0.z());
		
		tf::Vector3 v0_normalised = v0.normalized();
		
		//ROS_INFO("(ViewConeGenerator) Viewing direction (normalised): (%f, %f, %f).", v0_normalised.x(), v0_normalised.y(), v0_normalised.z());
		
		tf::Vector3 v1 = v0;
		v1 = v1.rotate(tf::Vector3(0.0f, 0.0f, 1.0f), fov / 2.0f);
		v1 = v1.normalize();
		
		//ROS_INFO("(ViewConeGenerator) V1 (normalised): (%f, %f, %f).", v1.x(), v1.y(), v1.z());
		
		float length = v0.length() / v0_normalised.dot(v1);
		v1 *= length;
		v1 += view_point;
		
		//ROS_INFO("(ViewConeGenerator) V1 (actual): (%f, %f, %f); length = %f.", v1.x(), v1.y(), v1.z(), length);
		
		tf::Vector3 v2 = v0;
		v2 = v2.rotate(tf::Vector3(0.0f, 0.0f, 1.0f), -fov / 2.0f);
		v2 = v2.normalize();
		//ROS_INFO("(ViewConeGenerator) V2 (normalised): (%f, %f, %f).", v2.x(), v2.y(), v2.z());
		
		length = v0.length() / v0_normalised.dot(v2);
		v2 *= length;
		v2 += view_point;
		//ROS_INFO("(ViewConeGenerator) V2 (actual): (%f, %f, %f); length = %f.", v2.x(), v2.y(), v2.z(), length);
		
		//ROS_INFO("(ViewConeGenerator) Triangle: (%f, %f, %f), (%f, %f, %f), (%f, %f, %f).", view_point.x(), view_point.y(), view_point.z(), v1.x(), v1.y(), v1.z(), v2.x(), v2.y(), v2.z());

		geometry_msgs::Point pose_view_point;
		pose_view_point.x = view_point.x();
		pose_view_point.y = view_point.y();
		pose_view_point.z = view_point.z();
		triangle_points.push_back(pose_view_point);
		waypoints.push_back(pose_view_point);
		
		geometry_msgs::Point pose_v1;
		pose_v1.x = v1.x();
		pose_v1.y = v1.y();
		pose_v1.z = v1.z();
		triangle_points.push_back(pose_v1);
		
		geometry_msgs::Point pose_v2;
		pose_v2.x = v2.x();
		pose_v2.y = v2.y();
		pose_v2.z = v2.z();
		triangle_points.push_back(pose_v2);
		
		std_msgs::ColorRGBA colour;
		colour.r = (float)rand() / (float)RAND_MAX;
		colour.b = (float)rand() / (float)RAND_MAX;
		colour.g = (float)rand() / (float)RAND_MAX;
		colour.a = 1.0f;
		triangle_colours.push_back(colour);
		triangle_colours.push_back(colour);
		triangle_colours.push_back(colour);
		
		colour.r = 1;
		colour.b = 1;
		colour.g = 1;
		colour.a = 1.0f;
		waypoint_colours.push_back(colour);
	}
	
	visualization_msgs::MarkerArray marker_array;
	
	// Create a marker to be displayed.
	visualization_msgs::Marker marker;
		
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();
	marker.id = 0;
	marker.ns = "Waypoints";
	marker.type = visualization_msgs::Marker::SPHERE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.points = waypoints;
	marker.colors = waypoint_colours;
	marker.pose.position.x = 0.0;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;
	marker.color.g = 1.0;
	marker.color.a = 1.0;

	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker_array.markers.push_back(marker);
	
	// Reuse the marker for the actual view cones.2
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();
	marker.id = 1;
	marker.ns = "ViewCone";
	marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.points = triangle_points;
	marker.colors = triangle_colours;
	marker.pose.position.x = 0.0;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;
	marker.color.g = 1.0;
	marker.color.a = 1.0;

	marker.scale.x = 1.;
	marker.scale.y = 1.;
	marker.scale.z = 1.;
	marker_array.markers.push_back(marker);
	
	rivz_pub_.publish(marker_array);
}

bool ViewConeGenerator::canConnect(const geometry_msgs::Point& w1, const geometry_msgs::Point& w2, int occupancy_threshold)
{
	occupancy_grid_utils::RayTraceIterRange ray_range = occupancy_grid_utils::rayTrace(last_received_occupancy_grid_msgs_.info, w1, w2, true, true);
	for (occupancy_grid_utils::RayTraceIterator i = ray_range.first; i != ray_range.second; ++i)
	{
		const occupancy_grid_utils::Cell& cell = *i;

		// Check if this cell is occupied.
		if (cell.x + cell.y * last_received_occupancy_grid_msgs_.info.width >= last_received_occupancy_grid_msgs_.data.size() ||
		    cell.x + cell.y * last_received_occupancy_grid_msgs_.info.width < 0 ||
		    last_received_occupancy_grid_msgs_.data[cell.x + cell.y * last_received_occupancy_grid_msgs_.info.width] > occupancy_threshold ||
		    last_received_occupancy_grid_msgs_.data[cell.x + cell.y * last_received_occupancy_grid_msgs_.info.width] == -1)
		{
			return false;
		}
	}
	
	return true;
}

float ViewConeGenerator::minDistanceToBlocked(const geometry_msgs::Point& point, float max_distance) const
{
	float min_distance = std::numeric_limits<float>::max();
	for (float x = -max_distance - last_received_occupancy_grid_msgs_.info.resolution; x < max_distance + last_received_occupancy_grid_msgs_.info.resolution; x += last_received_occupancy_grid_msgs_.info.resolution)
	{
		for (float y = -max_distance - last_received_occupancy_grid_msgs_.info.resolution; y < max_distance + last_received_occupancy_grid_msgs_.info.resolution; y += last_received_occupancy_grid_msgs_.info.resolution)
		{
			float distance = sqrt(x*x + y*y);
			if (distance > max_distance)
			{
				continue;
			}
			
			geometry_msgs::Point p;
			p.x = x + point.x;
			p.y = y + point.y;
			
			occupancy_grid_utils::Cell cell = occupancy_grid_utils::pointCell(last_received_occupancy_grid_msgs_.info, p);
			
			if (cell.x < 0 || cell.y < 0 || cell.x >= last_received_occupancy_grid_msgs_.info.width || cell.y >= last_received_occupancy_grid_msgs_.info.height) {
				continue;
			}
			
			if (last_received_occupancy_grid_msgs_.data[cell.x + cell.y * last_received_occupancy_grid_msgs_.info.width] > 0)
			{
				if (min_distance > distance) min_distance = distance;
			}
		}
	}
	return min_distance;
}


bool ViewConeGenerator::isBlocked(const geometry_msgs::Point& point, float min_distance) const
{
	for (float x = -min_distance - last_received_occupancy_grid_msgs_.info.resolution; x < min_distance + last_received_occupancy_grid_msgs_.info.resolution; x += last_received_occupancy_grid_msgs_.info.resolution)
	{
		for (float y = -min_distance - last_received_occupancy_grid_msgs_.info.resolution; y < min_distance + last_received_occupancy_grid_msgs_.info.resolution; y += last_received_occupancy_grid_msgs_.info.resolution)
		{
			//if (sqrt((x - point.x) * (x - point.x) + (y - point.y) * (y - point.y)) > min_distance)
			if (sqrt(x*x + y*y) > min_distance)
			{
				continue;
			}
			
			geometry_msgs::Point p;
			p.x = x + point.x;
			p.y = y + point.y;
			
			occupancy_grid_utils::Cell cell = occupancy_grid_utils::pointCell(last_received_occupancy_grid_msgs_.info, p);
			
			if (cell.x < 0 || cell.y < 0 || cell.x >= last_received_occupancy_grid_msgs_.info.width || cell.y >= last_received_occupancy_grid_msgs_.info.height) {
				continue;
			}
			
			if (last_received_occupancy_grid_msgs_.data[cell.x + cell.y * last_received_occupancy_grid_msgs_.info.width] > 0)
			{
				return true;
			}
		}
	}
	return false;
}

};
