/*!
*****************************************************************
* \file
*
* \note
* Copyright (c) 2012 \n
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA) \n\n
*
*****************************************************************
*
* \note
* Project name: Care-O-bot
* \note
* ROS stack name: cob_people_perception
* \note
* ROS package name: cob_people_detection
*
* \author
* Author: Richard Bormann
* \author
* Supervised by:
*
* \date Date of creation: 07.08.2012
*
* \brief
* functions for detecting a head within a point cloud/depth image
* current approach: haar detector on depth image
*
*****************************************************************
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer. \n
* - Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution. \n
* - Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission. \n
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 3 of the
* License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*
****************************************************************/



#ifdef __LINUX__
	#include "cob_people_detection/head_detector_node.h"
	#include "cob_vision_utils/GlobalDefines.h"
#else
#endif

// OpenCV
//#include "opencv/cv.h"
//#include<opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

// point cloud
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

// timer
#include <cob_people_detection/timer.h>

#include <fstream>

using namespace ipa_PeopleDetector;
using namespace std;


HeadDetectorNode::HeadDetectorNode(ros::NodeHandle nh)
: node_handle_(nh)
{
	data_directory_ = ros::package::getPath("cob_people_detection") + "/common/files/";

	// Parameters
	double depth_increase_search_scale;		// The factor by which the search window is scaled between the subsequent scans
	int depth_drop_groups;					// Minimum number (minus 1) of neighbor rectangles that makes up an object.
	int depth_min_search_scale_x;				// Minimum search scale x
	int depth_min_search_scale_y;				// Minimum search scale y
	std::cout << "\n--------------------------\nHead Detector Parameters:\n--------------------------\n";
	node_handle_.param("data_directory", data_directory_, data_directory_);
	std::cout << "data_directory = " << data_directory_ << "\n";
	node_handle_.param("fill_unassigned_depth_values", fill_unassigned_depth_values_, true);
	std::cout << "fill_unassigned_depth_values = " << fill_unassigned_depth_values_ << "\n";
	node_handle_.param("depth_increase_search_scale", depth_increase_search_scale, 1.1);
	std::cout << "depth_increase_search_scale = " << depth_increase_search_scale << "\n";
	node_handle_.param("depth_drop_groups", depth_drop_groups, 68);
	std::cout << "depth_drop_groups = " << depth_drop_groups << "\n";
	node_handle_.param("depth_min_search_scale_x", depth_min_search_scale_x, 20);
	std::cout << "depth_min_search_scale_x = " << depth_min_search_scale_x << "\n";
	node_handle_.param("depth_min_search_scale_y", depth_min_search_scale_y, 20);
	std::cout << "depth_min_search_scale_y = " << depth_min_search_scale_y << "\n";
	node_handle_.param("display_timing", display_timing_, false);
	std::cout << "display_timing = " << display_timing_ << "\n";

	// initialize head detector
	head_detector_.init(data_directory_, depth_increase_search_scale, depth_drop_groups, depth_min_search_scale_x, depth_min_search_scale_y);

	// advertise topics
	head_position_publisher_ = node_handle_.advertise<cob_people_detection_msgs::ColorDepthImageArray>("head_positions", 1);

	// subscribe to sensor topic
	pointcloud_sub_ = nh.subscribe("pointcloud_rgb", 1, &HeadDetectorNode::pointcloud_callback, this);

	std::cout << "HeadDetectorNode initialized." << std::endl;
}

HeadDetectorNode::~HeadDetectorNode(void)
{
}

void HeadDetectorNode::pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud)
{

//	Timer tim;
//	tim.start();

	// convert incoming colored point cloud to cv::Mat images
	cv::Mat depth_image;
	cv::Mat color_image;
	// Now we have both depth and color images, which are cv::Mat
	convertPclMessageToMat(pointcloud, depth_image, color_image);

//	cv::Mat gray_depth(depth_image.rows, depth_image.cols, CV_32FC1);
//	for (int v=0; v<depth_image.rows; ++v)
//		for (int u=0; u<depth_image.cols; ++u)
//			gray_depth.at<float>(v,u) = depth_image.at<cv::Vec3f>(v,u).val[2];
//	cv::normalize(gray_depth, gray_depth, 0.f, 1.f, cv::NORM_MINMAX);
//	cv::imshow("depth image", gray_depth);
//	char key = cv::waitKey(1000);
//	if (key == 's')
//	{
//		cv::normalize(gray_depth, gray_depth, 0.f, 255.f, cv::NORM_MINMAX);
//		cv::imwrite("depth_image.png", gray_depth);
//	}

	// detect heads in the depth image
	// Detection results are cv Rectangles
	std::vector<cv::Rect> head_bounding_boxes;
	head_detector_.detectRangeFace(depth_image, head_bounding_boxes, fill_unassigned_depth_values_);
	//	cv::Mat image;
	//	cv::cvtColor(color_image, image, cv::COLOR_RGB2BGR);

	// First test 1 person head tracking
	std::ofstream mylog;
	mylog.open ("mylog");

	
	static int track_object = 0;
	static cv::Mat hist;
	static cv::Rect track_box;
	static cv::Rect selection_box;
	int channels[] = {0, 1};
	int hbins = 30, sbins = 32;
	int histSize[] = {hbins, sbins};
	float hranges[] = {0, 180};
	float sranges[] = {0, 256};
	const float* ranges[] = {hranges, sranges};
	int track_min_box_area = 1000;
	cv::Size image_size = color_image.size();
	
	cv::Rect head_box;

	float scale = 0.4;
	cv::Mat backproj;
	cv::RotatedRect track_rot_box;
	cv::Mat hsv, mask;
	//	cv::Mat image;
	cv::cvtColor(color_image, hsv, cv::COLOR_RGB2BGR);
	cv::cvtColor(hsv, hsv, cv::COLOR_BGR2HSV);	
	//	cv::imshow("HSV Test", hsv);
        char c = (char)cv::waitKey(10);	
        cv::inRange(hsv, cv::Scalar(0, 15, 10), cv::Scalar(180, 256, 256), mask);
	
	if (head_bounding_boxes.size() == 1 || head_bounding_boxes.size() == 0) {
	  if (head_bounding_boxes.size() == 1) {
	    
	    // Shrink head box so that face is in it
	    head_box = head_bounding_boxes[0];
	    // Save height and width, keep the head area the same although the tracking box is smaller, so that face recognition can detect the face
	    // What's more, the information is used to decide whether the tracking box is too far away from the head detectionc
	    selection_box = head_box;
	    
	    track_box.width = head_box.width * scale;
	    track_box.height = head_box.height * scale;
	    track_box.x = (head_box.x + head_box.width/2) - track_box.width/2;
	    track_box.y = (head_box.y + head_box.height/2) - track_box.height/2;

	    // Make sure the tracking box is inside image
	    std::cout << "head box: " << head_box << std::endl;
	    std::cout << "tracking box: " << track_box << std::endl;
	    assert(track_box.x >= 0 && track_box.x + track_box.width <= image_size.width);
	    assert(track_box.y >= 0 && track_box.y + track_box.height <= image_size.height);
	    
	    // Calculate hue, saturation histogram
	    cv::Mat roi(hsv, track_box), mask_roi(mask, track_box);
	  
	    cv::calcHist(&roi, 1, channels, mask_roi, hist, 2, histSize, ranges);
	    cv::normalize(hist, hist, 0, 255, CV_MINMAX);
	  
	    // Store for later tracking
	    
	    track_object = 1;

	  }
	  
	  // Need to track?
	  if (track_object == 1) {

	    cout << "Calculating back project.. " << endl;
	    // Calcuate new rectangle based on the previous one
	    calcBackProject(&hsv, 1, channels, hist, backproj, ranges);
	    backproj &= mask;
	    cout << "backproj size: " << backproj.size() << endl;
	    cout << "track_box before CamShift: " << track_box << endl;
	    // CamShift should always output a box inside the image
	    track_rot_box = CamShift(backproj, track_box,
				     cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1));
	    cout << "track_box after CamShift: " << track_box << endl;
	    assert(track_box.x >= 0 && track_box.x + track_box.width <= image_size.width);
	    assert(track_box.y >= 0 && track_box.y + track_box.height <= image_size.height);
	    
	    // Valid area? 
	    if (track_box.area() <= track_min_box_area) {
	      std::cout << "Tracking box too small!!!" << std::endl;
	      track_object = 0;
	    }

    	    // Valid tracking result?
	    cv::Rect rect = track_rot_box.boundingRect();
	    float scale_max = 1.5, scale_min = 0.3;
	    float center_move_max = 140.0;

	    // Size changed too much?
	    cout << "*********************************" << endl;
	    cout << "rect: " << rect << endl;
	    cout << "selection_box: " << selection_box << endl;
	    
	    if (rect.height > selection_box.height*scale_max || rect.width > selection_box.width*scale_max) {
      	      std::cout << "Tracking box is too bigger than selection" << std::endl;
	      track_object = 0;
	    } else if ( rect.height < selection_box.height*scale_min || rect.width < selection_box.width*scale_min) {
		std::cout << "Tracking box is too smaller than selection" << std::endl;
		track_object = 0;
	      }

           // Center moves too much?
           if ( abs((rect.x + rect.width/2) - (selection_box.x + selection_box.width/2)) > center_move_max ||
         	   abs((rect.y + rect.height/2) - (selection_box.y + selection_box.height/2)) > center_move_max ) {
             std::cout << "Tracking box center moves too much" << std::endl;
             track_object = 0;
	    }

          }

	  // No head found through head detection but one possible head is being tracked
	  if (head_bounding_boxes.size() == 0 && track_object == 1) {
	    // How possible that this is a head?
	    
	    // Add tracking result to head detection result
	    cv::Rect bounding = track_rot_box.boundingRect();
	    // Heuristics, move the box up
    	    bounding.y = (bounding.y-10 >= 0)? bounding.y-10 : 0;

	    // Enlarged box still inside the image? If not, enlarge the box as much as we can
	    /// Too left?
	    if (bounding.x + (bounding.width/2-selection_box.width/2) < 0) {
	      // Go to the most left
	      bounding.x = 0;
	      // New width 
	      bounding.width = selection_box.width + (bounding.x + (bounding.width/2-selection_box.width/2));
	      cout << "Too left!" << endl;
	    } else if (bounding.x + selection_box.width > image_size.width) {
	      // Too right?
	      bounding.x = bounding.x + bounding.width/2 - selection_box.width/2;
	      bounding.width = image_size.width - bounding.x;
	      cout << "Too right!" << endl;
	    } else {
	      bounding.x = bounding.x + bounding.width/2 - selection_box.width/2;
	      bounding.width = selection_box.width;
	      if (bounding.width + bounding.x > image_size.width)
		bounding.width = image_size.width - bounding.x;
	    }

	    
	    // Too up?
	    if (bounding.y + (bounding.height/2-selection_box.height/2) < 0) {
	      // Go to the most up
	      bounding.y = 0;
	      // New height 
	      bounding.height = selection_box.height + (bounding.y + (bounding.height/2-selection_box.height/2));
	      cout << "Too up!" << endl;
	    } else if (bounding.y + selection_box.height > image_size.height) {
	      // Too down?
	      bounding.y = bounding.y + bounding.height/2 - selection_box.height/2;
	      bounding.height = image_size.height - bounding.y;
	      cout << "Too down!" << endl;
	    } else {
	      bounding.y = bounding.y + bounding.height/2 - selection_box.height/2;
	      bounding.height = selection_box.height;
	      if (bounding.height + bounding.y > image_size.height)
		bounding.height = image_size.height - bounding.y;
	    }	    

	    head_bounding_boxes.push_back(bounding);
	    //	    head_bounding_boxes.push_back(track_rot_box.boundingRect());
	    cout << "Selection_Box Width: " << selection_box.width << ", Height: " << selection_box.height << endl;
	    cout << "Bounding box: " << bounding << endl;
	    assert(bounding.x >= 0 && bounding.x + bounding.width <= image_size.width);
	    assert(bounding.y >= 0 && bounding.y + bounding.height <= image_size.height);

	    // log distance with selection
	    
	    
	  }
	  
	  //	  cv::imshow("Head Test", image);
          char c = (char)cv::waitKey(10);
	}

	
	// publish image patches from head region
	cob_people_detection_msgs::ColorDepthImageArray image_array;
	image_array.header = pointcloud->header;
	image_array.head_detections.resize(head_bounding_boxes.size());
	for (unsigned int i=0; i<head_bounding_boxes.size(); i++)
	{
		cv_bridge::CvImage cv_ptr;
		image_array.head_detections[i].head_detection.x = head_bounding_boxes[i].x;
		image_array.head_detections[i].head_detection.y = head_bounding_boxes[i].y;
		image_array.head_detections[i].head_detection.width = head_bounding_boxes[i].width;
		image_array.head_detections[i].head_detection.height = head_bounding_boxes[i].height;
		// Note the way to select part of a cv Mat
		cv::Mat depth_patch = depth_image(head_bounding_boxes[i]);
		// cv_bridge CvImage
		cv_ptr.image = depth_patch;
		cv_ptr.encoding = sensor_msgs::image_encodings::TYPE_32FC3;	// CV32FC3
		// Convert cv_bridge CvImage to image msg
		image_array.head_detections[i].depth_image = *(cv_ptr.toImageMsg());
		cv::Mat color_patch = color_image(head_bounding_boxes[i]);
		cv_ptr.image = color_patch;
		cv_ptr.encoding = sensor_msgs::image_encodings::BGR8;
		image_array.head_detections[i].color_image = *(cv_ptr.toImageMsg());
	}
	head_position_publisher_.publish(image_array);

	if (display_timing_ == true)
		ROS_INFO("%d HeadDetection: Time stamp of pointcloud message: %f. Delay: %f.", pointcloud->header.seq, pointcloud->header.stamp.toSec(), ros::Time::now().toSec()-pointcloud->header.stamp.toSec());
//	ROS_INFO("Head Detection took %f ms.", tim.getElapsedTimeInMilliSec());
	
}

unsigned long HeadDetectorNode::convertPclMessageToMat(const sensor_msgs::PointCloud2::ConstPtr& pointcloud, cv::Mat& depth_image, cv::Mat& color_image)
{
	pcl::PointCloud<pcl::PointXYZRGB> depth_cloud; // point cloud
	pcl::fromROSMsg(*pointcloud, depth_cloud);
	depth_image.create(depth_cloud.height, depth_cloud.width, CV_32FC3);
	color_image.create(depth_cloud.height, depth_cloud.width, CV_8UC3);
	uchar* depth_image_ptr = (uchar*) depth_image.data;
	uchar* color_image_ptr = (uchar*) color_image.data;
	for (int v=0; v<(int)depth_cloud.height; v++)
	{
		int depth_base_index = depth_image.step*v;
		int color_base_index = color_image.step*v;
		for (int u=0; u<(int)depth_cloud.width; u++)
		{
			int depth_index = depth_base_index + 3*u*sizeof(float);
			float* depth_data_ptr = (float*)(depth_image_ptr+depth_index);
			int color_index = color_base_index + 3*u*sizeof(uchar);
			uchar* color_data_ptr = (uchar*)(color_image_ptr+color_index);
			pcl::PointXYZRGB point_xyz = depth_cloud(u,v);
			depth_data_ptr[0] = point_xyz.x;
			depth_data_ptr[1] = point_xyz.y;
			depth_data_ptr[2] = (isnan(point_xyz.z)) ? 0.f : point_xyz.z;
			color_data_ptr[0] = point_xyz.r;
			color_data_ptr[1] = point_xyz.g;
			color_data_ptr[2] = point_xyz.b;
			//if (u%100 == 0) std::cout << "u" << u << " v" << v << " z" << data_ptr[2] << "\n";
		}
	}
	return ipa_Utils::RET_OK;
}


//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "head_detector");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	// Create HeadDetectorNode class instance
	HeadDetectorNode head_detector_node(nh);

	// Create action nodes
	//DetectObjectsAction detect_action_node(object_detection_node, nh);
	//AcquireObjectImageAction acquire_image_node(object_detection_node, nh);
	//TrainObjectAction train_object_node(object_detection_node, nh);

	ros::spin();

	return 0;
}
