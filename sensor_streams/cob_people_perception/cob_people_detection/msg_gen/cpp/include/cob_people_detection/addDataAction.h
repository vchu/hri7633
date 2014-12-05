/* Auto-generated by genmsg_cpp for file /home/xinyan/groovy_workspace/sandbox/hri7633/sensor_streams/cob_people_perception/cob_people_detection/msg/addDataAction.msg */
#ifndef COB_PEOPLE_DETECTION_MESSAGE_ADDDATAACTION_H
#define COB_PEOPLE_DETECTION_MESSAGE_ADDDATAACTION_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "cob_people_detection/addDataActionGoal.h"
#include "cob_people_detection/addDataActionResult.h"
#include "cob_people_detection/addDataActionFeedback.h"

namespace cob_people_detection
{
template <class ContainerAllocator>
struct addDataAction_ {
  typedef addDataAction_<ContainerAllocator> Type;

  addDataAction_()
  : action_goal()
  , action_result()
  , action_feedback()
  {
  }

  addDataAction_(const ContainerAllocator& _alloc)
  : action_goal(_alloc)
  , action_result(_alloc)
  , action_feedback(_alloc)
  {
  }

  typedef  ::cob_people_detection::addDataActionGoal_<ContainerAllocator>  _action_goal_type;
   ::cob_people_detection::addDataActionGoal_<ContainerAllocator>  action_goal;

  typedef  ::cob_people_detection::addDataActionResult_<ContainerAllocator>  _action_result_type;
   ::cob_people_detection::addDataActionResult_<ContainerAllocator>  action_result;

  typedef  ::cob_people_detection::addDataActionFeedback_<ContainerAllocator>  _action_feedback_type;
   ::cob_people_detection::addDataActionFeedback_<ContainerAllocator>  action_feedback;


  typedef boost::shared_ptr< ::cob_people_detection::addDataAction_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cob_people_detection::addDataAction_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct addDataAction
typedef  ::cob_people_detection::addDataAction_<std::allocator<void> > addDataAction;

typedef boost::shared_ptr< ::cob_people_detection::addDataAction> addDataActionPtr;
typedef boost::shared_ptr< ::cob_people_detection::addDataAction const> addDataActionConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::cob_people_detection::addDataAction_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::cob_people_detection::addDataAction_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace cob_people_detection

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::cob_people_detection::addDataAction_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::cob_people_detection::addDataAction_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::cob_people_detection::addDataAction_<ContainerAllocator> > {
  static const char* value() 
  {
    return "406a1bfd0e93ae3fc2184d9618972807";
  }

  static const char* value(const  ::cob_people_detection::addDataAction_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x406a1bfd0e93ae3fULL;
  static const uint64_t static_value2 = 0xc2184d9618972807ULL;
};

template<class ContainerAllocator>
struct DataType< ::cob_people_detection::addDataAction_<ContainerAllocator> > {
  static const char* value() 
  {
    return "cob_people_detection/addDataAction";
  }

  static const char* value(const  ::cob_people_detection::addDataAction_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::cob_people_detection::addDataAction_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
addDataActionGoal action_goal\n\
addDataActionResult action_result\n\
addDataActionFeedback action_feedback\n\
\n\
================================================================================\n\
MSG: cob_people_detection/addDataActionGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalID goal_id\n\
addDataGoal goal\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalID\n\
# The stamp should store the time at which this goal was requested.\n\
# It is used by an action server when it tries to preempt all\n\
# goals that were requested before a certain time\n\
time stamp\n\
\n\
# The id provides a way to associate feedback and\n\
# result message with specific goal requests. The id\n\
# specified must be unique.\n\
string id\n\
\n\
\n\
================================================================================\n\
MSG: cob_people_detection/addDataGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# Adds data to the training database\n\
#\n\
# goal message\n\
string label				# a label for the data which shall be added\n\
int32 capture_mode			# mode of data capture: 0=manual (i.e. initiate data capture with service messages), 1=continuous\n\
int32 continuous_mode_images_to_capture	# if the continuous mode is selected this number specifies how many images shall be captured\n\
float32 continuous_mode_delay		# if the continuous mode is selected this number specifies the delay time between the recording of two successive images (in seconds)\n\
\n\
================================================================================\n\
MSG: cob_people_detection/addDataActionResult\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
addDataResult result\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalStatus\n\
GoalID goal_id\n\
uint8 status\n\
uint8 PENDING         = 0   # The goal has yet to be processed by the action server\n\
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server\n\
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing\n\
                            #   and has since completed its execution (Terminal State)\n\
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)\n\
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due\n\
                            #    to some failure (Terminal State)\n\
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,\n\
                            #    because the goal was unattainable or invalid (Terminal State)\n\
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing\n\
                            #    and has not yet completed execution\n\
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,\n\
                            #    but the action server has not yet confirmed that the goal is canceled\n\
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing\n\
                            #    and was successfully cancelled (Terminal State)\n\
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be\n\
                            #    sent over the wire by an action server\n\
\n\
#Allow for the user to associate a string with GoalStatus for debugging\n\
string text\n\
\n\
\n\
================================================================================\n\
MSG: cob_people_detection/addDataResult\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# result message\n\
\n\
================================================================================\n\
MSG: cob_people_detection/addDataActionFeedback\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
addDataFeedback feedback\n\
\n\
================================================================================\n\
MSG: cob_people_detection/addDataFeedback\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# feedback message\n\
int32 images_captured			# the number of images already captured\n\
\n\
\n\
";
  }

  static const char* value(const  ::cob_people_detection::addDataAction_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::cob_people_detection::addDataAction_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.action_goal);
    stream.next(m.action_result);
    stream.next(m.action_feedback);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct addDataAction_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cob_people_detection::addDataAction_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::cob_people_detection::addDataAction_<ContainerAllocator> & v) 
  {
    s << indent << "action_goal: ";
s << std::endl;
    Printer< ::cob_people_detection::addDataActionGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.action_goal);
    s << indent << "action_result: ";
s << std::endl;
    Printer< ::cob_people_detection::addDataActionResult_<ContainerAllocator> >::stream(s, indent + "  ", v.action_result);
    s << indent << "action_feedback: ";
s << std::endl;
    Printer< ::cob_people_detection::addDataActionFeedback_<ContainerAllocator> >::stream(s, indent + "  ", v.action_feedback);
  }
};


} // namespace message_operations
} // namespace ros

#endif // COB_PEOPLE_DETECTION_MESSAGE_ADDDATAACTION_H

