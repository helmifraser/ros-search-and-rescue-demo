#pragma once

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <fstream>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <boost/thread.hpp>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <math.h> 


class ObDet
{
private:
	geometry_msgs::Twist cmd_stored;
	ros::Publisher pub_vel;
	ros::Publisher pub_pcl;
	ros::Publisher pub_mark;
	ros::Subscriber laser_sub;	
	ros::Subscriber odom_sub;
	laser_geometry::LaserProjection projector;
	tf::TransformListener listener;

	visualization_msgs::Marker marker, points, line;

	image_transport::Subscriber image_sub_;
	std::string OPENCV_WINDOW1, OPENCV_WINDOW2, OPENCV_WINDOW3;


	int count, count_substage, check, body_count, object_count, size_of_array, center, posX_body, posY_body, 
			posX_object, posY_object, iLowH_body,iHighH_body,iLowS_body,iHighS_body,iLowV_body,iHighV_body, 
			iLowH_object,iHighH_object,iLowS_object,iHighS_object,iLowV_object,iHighV_object, vector_body, 
			morphsize, Colour, stage, substage, sub_sub_stage,amount_objects_hit;

	bool clear_path, clear_turn, turn_check, travel_check, sub_clear_path, axis, stop,
			past,object_detected, body_detected, close_enough;

	float metre, metreback, angular_speed_360, angular_speed_90, linear_speed, turn_angle, degree, 
			target, ratio, desired, error, lastx, lasty,
			nowx, nowy, start_angle, current_angle, x, y, z,
			object_distance, target_angle,
			bodies_angles[100], bodies_coords[100], bodies_xcoords[100],bodies_ycoords[100], objects_angles[100],
			objects_coords[100], objects_xcoords[100], objects_ycoords[100], dist_saved_bodies[100], dist_saved_objects[100];
	
  double dArea_body, dArea_object;
  	
public:
	ObDet(ros::NodeHandle nh_, image_transport::ImageTransport it_);
	~ObDet();
	void odomcallback(const nav_msgs::Odometry::ConstPtr& msg);
	void lasercallback(const sensor_msgs::LaserScan::ConstPtr& scan);
	void imagecallback(const sensor_msgs::ImageConstPtr& msg);
	void traveldecision(float distance_in);
	void search();
	void createline();
	void createmodel();
	void turn90(bool right);
	void turn360();
	void straight(bool direction);
	void travel(float distance);
	void process();
};