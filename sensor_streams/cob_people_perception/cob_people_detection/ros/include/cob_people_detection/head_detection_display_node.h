#ifndef _HEAD_DETECTION_DISPLAY_
#define _HEAD_DETECTION_DISPLAY_

// standard includes
#include <sstream>
#include <string>
#include <vector>

// ROS includes
#include <ros/ros.h>
#include <ros/package.h>

// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cob_people_detection_msgs/ColorDepthImageArray.h>

// topics
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// opencv
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// boost
#include <boost/bind.hpp>

// point cloud
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

// external includes
#include "cob_vision_utils/GlobalDefines.h"


namespace ipa_PeopleDetector {

class HeadDetectionDisplayNode
{
protected:

	image_transport::ImageTransport* it_;
	image_transport::SubscriberFilter colorimage_sub_; ///< Color camera image topic
	message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_people_detection_msgs::ColorDepthImageArray, sensor_msgs::Image> >* sync_input_2_;
	message_filters::Subscriber<cob_people_detection_msgs::ColorDepthImageArray> head_detection_subscriber_; ///< receives the face messages from the face detector
	image_transport::Publisher head_detection_image_pub_; ///< topic for publishing the image containing the head positions

	ros::NodeHandle node_handle_; ///< ROS node handle
	// parameters
	bool display_; ///< if on, several debug outputs are activated
	bool display_timing_;

public:

	HeadDetectionDisplayNode(ros::NodeHandle nh);
	~HeadDetectionDisplayNode();

	/// Converts a color image message to cv::Mat format.
	unsigned long convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image);

	void inputCallback(const cob_people_detection_msgs::ColorDepthImageArray::ConstPtr& head_detection_msg, const sensor_msgs::Image::ConstPtr& colorimage_msg);
};

};

#endif

