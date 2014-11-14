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

using namespace ipa_PeopleDetector;
using namespace std;

typedef struct CamShiftRecord {
  cv::Mat hist;
  cv::Rect track_box;
  cv::Rect select_box;
  cv::Rect head_box;
} CamShiftRecord;


// Shrink a rectangel that is already inside an image
// Assume scale <= 1
void shrink_box(cv::Rect& out, const cv::Rect& in, float scale) {
  assert(scale <= 1);
  out.width = in.width * scale;
  out.height = in.height * scale;
  out.x = (in.x + in.width/2) - out.width/2;
  out.y = (in.y + in.height/2) - out.height/2;
}


void calc_hist(cv::Mat &hist, const cv::Mat& image, const cv::Rect& box, const cv::Mat& mask, const int* histSize, const float** ranges) {
  int channels[] = {0, 1};
  cv::Mat roi(image, box), mask_roi(mask, box);
  cv::calcHist(&roi, 1, channels, mask_roi, hist, 2, histSize, ranges);
  cv::normalize(hist, hist, 0, 255, CV_MINMAX);
}


int calc_camshift_box(cv::Rect& track_box, cv::RotatedRect& track_rot_box, const cv::Mat& image, const cv::Mat& mask, const cv::Mat& hist, const int* histSize, const float** ranges, const cv:: Rect& select_box) {
  
  int channels[] = {0, 1};
  cv::Mat backproj;
  cv::Size image_size = image.size();
  
  // Backproject hist on the current image, and get backproject image for CamShift
  calcBackProject(&image, 1, channels, hist, backproj, ranges);
  
  // Calcuate new rectangle based on the previous one
  backproj &= mask;

  // CamShift should always output a box inside the image
  track_rot_box = CamShift(backproj, track_box,
			   cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1));
  assert(track_box.x >= 0 && track_box.x + track_box.width <= image_size.width);
  assert(track_box.y >= 0 && track_box.y + track_box.height <= image_size.height);
 
  // Valid tracking result? Whether the current tracking deviates too much from the selection?
  cv::Rect rect = track_rot_box.boundingRect();
  float scale_max = 1.5, scale_min = 0.4;
  float center_move_max = 80.0;
  int track_max_box_area = 30000;  
  
  // Check whether the output tracking box is
  // Histogram similarity
  cv::Mat track_hist;
  calc_hist(track_hist, image, track_box, mask, histSize, ranges);
  double hist_comparison = compareHist(track_hist, hist, CV_COMP_CORREL);
  double hist_comparison_min = 0.3;
  if (hist_comparison < hist_comparison_min) {
    cout << "Histogram differs too much: " << hist_comparison << endl;
    return 5;
  }

  
  // Not too large area?
  if (track_box.area() >= track_max_box_area) {
    std::cout << "Tracking box too big!!!" << std::endl;
    return 1;
  }

  // Size changed too much?
  if (rect.height > select_box.height*scale_max || rect.width > select_box.width*scale_max) {
    std::cout << "Tracking box is too bigger than select" << std::endl;
    return 2;
  } else if ( rect.height < select_box.height*scale_min || rect.width < select_box.width*scale_min) {
    std::cout << "Tracking box is too smaller than select" << std::endl;
    return 3;
  }

  // Center moves too much?
  if ( abs((rect.x + rect.width/2) - (select_box.x + select_box.width/2)) > center_move_max ||
       abs((rect.y + rect.height/2) - (select_box.y + select_box.height/2)) > center_move_max ) {
    std::cout << "Tracking box center moves too much" << std::endl;
    return 4;
  }

  return 0;
}


// Calculate head box inside the image based on the selection box and rotated tracking box
void calc_head_box(cv::Rect& head_box, const cv::Size& image_size, const cv::RotatedRect& track_rot_box, const cv::Rect& select_box) {
  
  head_box = track_rot_box.boundingRect();
  // !! Heuristics, move the box up
  head_box.y = (head_box.y-5 >= 0)? head_box.y-5 : 0;

  cout << "Try to make head box similar to selection box" << endl;
  cout << "Head box: " << head_box << ", selection box: " << select_box << endl;
  
  // Enlarged box still inside the image? If not, enlarge the box as much as we can
  head_box.x = head_box.x + head_box.width/2-select_box.width/2;
  head_box.width = select_box.width;
  head_box.y = head_box.y + head_box.height/2-select_box.height/2;
  head_box.height = select_box.height;
  
  // Too left
  if (head_box.x < 0) {
    head_box.x = 0;
  }

  // Too right
  if (head_box.x + head_box.width > image_size.width) {
    head_box.width = image_size.width - head_box.x;
  }

  // Too up
  if (head_box.y < 0) {
    head_box.y = 0;
  }

  // Too down
  if (head_box.y + head_box.height > image_size.height) {
    head_box.height = image_size.height - head_box.y;
  }
  


  
  // /// Too left?
  // if (head_box.x + (head_box.width/2-select_box.width/2) < 0) {
  //   // Go to the most left
  //   head_box.x = 0;
  //   // New width 
  //   head_box.width = select_box.width + (head_box.x + (head_box.width/2-select_box.width/2));
  //   cout << "Too left!" << endl;
  // } else if (head_box.x + select_box.width > image_size.width) {
  //   // Too right?
  //   //    head_box.x = head_box.x + head_box.width/2 - select_box.width/2;
  //   head_box.x = head_box.x;
  //   head_box.width = image_size.width - head_box.x;
  //   cout << "Too right!" << endl;
  // } else {
  //   head_box.x = head_box.x + head_box.width/2 - select_box.width/2;
  //   head_box.width = select_box.width;
  //   if (head_box.width + head_box.x > image_size.width)
  //     head_box.width = image_size.width - head_box.x;
  // }

  // // Too up?
  // if (head_box.y + (head_box.height/2-select_box.height/2) < 0) {
  //   // Go to the most up
  //   head_box.y = 0;
  //   // New height 
  //   head_box.height = select_box.height + (head_box.y + (head_box.height/2-select_box.height/2));
  //   cout << "Too up!" << endl;
  // } else if (head_box.y + select_box.height > image_size.height) {
  //   // Too down?
  //   //    head_box.y = head_box.y + head_box.height/2 - select_box.height/2;
  //   head_box.y = head_box.y;
  //   head_box.height = image_size.height - head_box.y;
  //   cout << "Too down!" << endl;
  // } else {
  //   head_box.y = head_box.y + head_box.height/2 - select_box.height/2;
  //   head_box.height = select_box.height;
  //   if (head_box.height + head_box.y > image_size.height)
  //     head_box.height = image_size.height - head_box.y;
  // }	    

  cout << "Final head_box:" << head_box << endl;

  // Head box should be inside the image
  assert(head_box.x >= 0 && head_box.x + head_box.width <= image_size.width);
  assert(head_box.y >= 0 && head_box.y + head_box.height <= image_size.height);
  
				 
}


// Maybe not just use dist, use histogram to calculate similarities also
// Then we need to save the patch alsoh
inline float get_dist(const cv::Rect& a, const cv::Rect& b) {
  return sqrt(pow(a.x-b.x, 2.) + pow(a.y-b.y, 2.));
}


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

  // detect heads in the depth image
  // Detection results are cv Rectangles
  std::vector<cv::Rect> head_bounding_boxes;
  head_detector_.detectRangeFace(depth_image, head_bounding_boxes, fill_unassigned_depth_values_);

  float max_head_area = 180 * 240;
  for (vector<cv::Rect>::iterator it = head_bounding_boxes.begin(); it != head_bounding_boxes.end(); ) {
    if (it->area() > max_head_area) {
      it = head_bounding_boxes.erase(it);
    } else {
      ++it;
    }

  }
	
  static int track_object = 0;
	
  int channels[] = {0, 1};
  int hbins = 30, sbins = 32;
  int histSize[] = {hbins, sbins};
  float hranges[] = {0, 180};
  float sranges[] = {0, 256};
  const float* ranges[] = {hranges, sranges};
  int track_min_box_area = 1000;
  cv::Size image_size = color_image.size();

  // For trackers
  static list<CamShiftRecord> tracker_list;
	
  float scale = 0.2;
  cv::Mat backproj;
  cv::RotatedRect track_rot_box;
  cv::Mat hsv, mask;
  //	cv::Mat image;
  cv::cvtColor(color_image, hsv, cv::COLOR_RGB2BGR);
  cv::cvtColor(hsv, hsv, cv::COLOR_BGR2HSV);	
  //	cv::imshow("HSV Test", hsv);
  char c = (char)cv::waitKey(10);	
  cv::inRange(hsv, cv::Scalar(0, 15, 10), cv::Scalar(180, 256, 256), mask);

  // For head detection
  vector<CamShiftRecord> detect_vec;
  int retval;	  

  // Number of heads we were tracking
  int ntrackers;
  // Number of head detected from this fram
  int ndetected = head_bounding_boxes.size();
	  
  // Has head detected?
  // Calculate hist and CamShift box
  cout << "Detected: " << ndetected << endl;
  if ( ndetected >= 1) {

    // Resize vector
    detect_vec.resize(ndetected);

    // Go through all the newly detected heads, calcuate hist, select_box, track_box and head_box
    for (int i = 0; i < ndetected; i++) {
      // Save height and width, keep the head area the same although the tracking box is smaller, so that face recognition can detect the face
      // What's more, the information is used to decide whether the tracking box is too far away from the head detectionc
      detect_vec[i].select_box = head_bounding_boxes[i];
	      
      // Shrink head box so that face is in it
      shrink_box(detect_vec[i].track_box, detect_vec[i].select_box, scale);
	    
      // Calculate hue, saturation histogram
      calc_hist(detect_vec[i].hist, hsv, detect_vec[i].track_box, mask, histSize, ranges);

      // Update track_box and get track_rot_box
      retval = calc_camshift_box(detect_vec[i].track_box, track_rot_box, hsv, mask, detect_vec[i].hist, histSize, ranges, detect_vec[i].select_box);
	      
      // Not good tracking
      // First track of the head detection should be always OK
      assert(retval == 0);
	      
      // Convert track_rot_box
      calc_head_box(detect_vec[i].head_box, hsv.size(), track_rot_box, detect_vec[i].select_box);
    }
  }
	  
  // Have previous trackers?
  // Calcualte CamShift
  // Go through all the trackers, calcualte new track_box and head_box
  //	  cout << "Now start prepareing trackers" << endl;
  list<CamShiftRecord>::iterator itr = tracker_list.begin();
  if (tracker_list.size() > 0) {
    //	    cout << "Testing being()->head_box:" << tracker_list.begin()->head_box << endl;
  }
	   
  while (itr != tracker_list.end()) {
    retval = calc_camshift_box(itr->track_box, track_rot_box, hsv, mask, itr->hist, histSize, ranges, itr->select_box);
    // Not good tracking anymore
    // Remove the tracker
    // In the meantime, the itr is incremented
    if (retval > 0) {
      itr = tracker_list.erase(itr);
      continue;
    }

    // Convert track_rot_box
    calc_head_box(itr->head_box, hsv.size(), track_rot_box, itr->select_box);
    ++itr;
  }
    
  ntrackers = tracker_list.size();
  cout << "Trackers: " << ntrackers << endl;


  // Match head detections with trackers
  //	  float close_thresh = 80;
  float close_thresh = 40;
  float dist;
  list<CamShiftRecord>::iterator closest_itr;
  double similarity_mag = -50;
  cv::Mat track_hist;
  double hist_comparison;

  // Construct the cost matrix
  vector<vector<int> > cost_matrix(ntrackers, vector<int>(ndetected, 0));
  itr = tracker_list.begin();
  for (int ti = 0; ti < ntrackers; ++ti) {
    for (int di = 0; di < ndetected; ++di) {
      calc_hist(track_hist, hsv, itr->track_box, mask, histSize, ranges);
      hist_comparison = compareHist(track_hist, detect_vec[di].hist, CV_COMP_CORREL);
      dist = get_dist(itr->track_box, detect_vec[di].track_box);
      cost_matrix[ti][di] = dist + hist_comparison * similarity_mag;
    }
    ++itr;
  }

  // Delete the trackers that do not have close detections
  itr = tracker_list.begin();
  int hasclose;
    
  for (int ti = 0; ti < ntrackers; ++ti) {
    hasclose = 0;
    for (int di = 0; di < ndetected; ++di) {
      if (cost_matrix[ti][di] <= close_thresh) {
	hasclose = 1;
	break;
      }
    }
    if (hasclose) {
      itr = tracker_list.erase(itr);
    } else {
      ++itr;
    }
  }

  // Add detections to trackers
  for (int di = 0; di < ndetected; ++di) {
    tracker_list.push_back(detect_vec[di]);
  }
    

  // Construct heading boxes (do we need to use the original detection head boxes?)
  head_bounding_boxes.clear();
  for (itr = tracker_list.begin(); itr != tracker_list.end(); ++itr) {
    head_bounding_boxes.push_back(itr->head_box);
  }

  cout << "Number of head box to display: " << head_bounding_boxes.size() << endl;
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


