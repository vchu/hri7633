#include <cob_people_detection/head_detection_display_node.h>

using namespace ipa_PeopleDetector;

inline bool isnan_(double num) { return (num != num); };

HeadDetectionDisplayNode::HeadDetectionDisplayNode(ros::NodeHandle nh)
  : node_handle_(nh)
{
  it_ = 0;
  sync_input_2_ = 0;

  // parameters
  std::cout << "\n---------------------------\nHead Detection Display Parameters:\n---------------------------\n";
  node_handle_.param("display", display_, false);
  std::cout << "display = " << display_ << "\n";
  node_handle_.param("display_timing", display_timing_, false);
  std::cout << "display_timing = " << display_timing_ << "\n";

  // subscribers
  it_ = new image_transport::ImageTransport(node_handle_);
  colorimage_sub_.subscribe(*it_, "colorimage_in", 1);
  head_detection_subscriber_.subscribe(node_handle_, "head_detections", 1);

  // input synchronization
  sync_input_2_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_people_detection_msgs::ColorDepthImageArray, sensor_msgs::Image> >(60);
  sync_input_2_->connectInput(head_detection_subscriber_, colorimage_sub_);
  sync_input_2_->registerCallback(boost::bind(&HeadDetectionDisplayNode::inputCallback, this, _1, _2));

  // publishers
  head_detection_image_pub_ = it_->advertise("head_position_image", 1);

  std::cout << "HeadDetectionDisplay initialized." << std::endl;
}
    
HeadDetectionDisplayNode::~HeadDetectionDisplayNode()
{
  if (it_ != 0) delete it_;
  if (sync_input_2_ != 0) delete sync_input_2_;
}

/// Converts a color image message to cv::Mat format.
unsigned long HeadDetectionDisplayNode::convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image)
{
  try
    {
      image_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
    }
  catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("HeadDetection: cv_bridge exception: %s", e.what());
      return ipa_Utils::RET_FAILED;
    }
  image = image_ptr->image;


  return ipa_Utils::RET_OK;
}
    

void HeadDetectionDisplayNode::inputCallback(const cob_people_detection_msgs::ColorDepthImageArray::ConstPtr& head_detection_msg, const sensor_msgs::Image::ConstPtr& color_image_msg)
{
  cv_bridge::CvImageConstPtr color_image_ptr;
  cv::Mat color_image;
  convertColorImageMessageToMat(color_image_msg, color_image_ptr, color_image);

  // insert all detected heads
  // Go over all head detections
  for (int i=0; i<(int)head_detection_msg->head_detections.size(); i++)
    {
      // paint head
      // Light blue
      const cob_people_detection_msgs::Rect& head_rect = head_detection_msg->head_detections[i].head_detection;
      cv::Rect head(head_rect.x, head_rect.y, head_rect.width, head_rect.height);
      cv::rectangle(color_image, cv::Point(head.x, head.y), cv::Point(head.x + head.width, head.y + head.height), CV_RGB(249, 56, 34), 2, 8, 0);

    }

  // display image
  if (display_ == true)
    {
      cv::imshow("Detections and Recognitions", color_image);
      cv::waitKey(10);
    }

  // publish image
  cv_bridge::CvImage cv_ptr;
  cv_ptr.image = color_image;
  cv_ptr.encoding = "bgr8";
  head_detection_image_pub_.publish(cv_ptr.toImageMsg());

  if (display_timing_ == true)
    ROS_INFO("%d Display: Time stamp of image message: %f. Delay: %f.", color_image_msg->header.seq, color_image_msg->header.stamp.toSec(), ros::Time::now().toSec()-color_image_msg->header.stamp.toSec());
}


//#######################
//#### main programm ####
int main(int argc, char** argv)
{
  // Initialize ROS, specify name of node
  ros::init(argc, argv, "head_detection_display");

  // Create a handle for this node, initialize node
  ros::NodeHandle nh;


  HeadDetectionDisplayNode head_detection_display_node(nh);

  ros::spin();

  return 0;
}
