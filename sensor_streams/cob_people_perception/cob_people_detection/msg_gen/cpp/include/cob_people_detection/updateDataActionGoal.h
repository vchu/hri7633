/* Auto-generated by genmsg_cpp for file /home/xinyan/groovy_workspace/sandbox/hri7633/sensor_streams/cob_people_perception/cob_people_detection/msg/updateDataActionGoal.msg */
#ifndef COB_PEOPLE_DETECTION_MESSAGE_UPDATEDATAACTIONGOAL_H
#define COB_PEOPLE_DETECTION_MESSAGE_UPDATEDATAACTIONGOAL_H
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

#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "cob_people_detection/updateDataGoal.h"

namespace cob_people_detection
{
template <class ContainerAllocator>
struct updateDataActionGoal_ {
  typedef updateDataActionGoal_<ContainerAllocator> Type;

  updateDataActionGoal_()
  : header()
  , goal_id()
  , goal()
  {
  }

  updateDataActionGoal_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , goal_id(_alloc)
  , goal(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef  ::actionlib_msgs::GoalID_<ContainerAllocator>  _goal_id_type;
   ::actionlib_msgs::GoalID_<ContainerAllocator>  goal_id;

  typedef  ::cob_people_detection::updateDataGoal_<ContainerAllocator>  _goal_type;
   ::cob_people_detection::updateDataGoal_<ContainerAllocator>  goal;


  typedef boost::shared_ptr< ::cob_people_detection::updateDataActionGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cob_people_detection::updateDataActionGoal_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct updateDataActionGoal
typedef  ::cob_people_detection::updateDataActionGoal_<std::allocator<void> > updateDataActionGoal;

typedef boost::shared_ptr< ::cob_people_detection::updateDataActionGoal> updateDataActionGoalPtr;
typedef boost::shared_ptr< ::cob_people_detection::updateDataActionGoal const> updateDataActionGoalConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::cob_people_detection::updateDataActionGoal_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::cob_people_detection::updateDataActionGoal_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace cob_people_detection

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::cob_people_detection::updateDataActionGoal_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::cob_people_detection::updateDataActionGoal_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::cob_people_detection::updateDataActionGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "0b0388e146474b842a2e77050e12d484";
  }

  static const char* value(const  ::cob_people_detection::updateDataActionGoal_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x0b0388e146474b84ULL;
  static const uint64_t static_value2 = 0x2a2e77050e12d484ULL;
};

template<class ContainerAllocator>
struct DataType< ::cob_people_detection::updateDataActionGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "cob_people_detection/updateDataActionGoal";
  }

  static const char* value(const  ::cob_people_detection::updateDataActionGoal_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::cob_people_detection::updateDataActionGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalID goal_id\n\
updateDataGoal goal\n\
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
MSG: cob_people_detection/updateDataGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# Updates data in the training database\n\
#\n\
# goal message\n\
int32 update_mode			# update the label with new_label either for 1=one image given the update_index, 2=all entries labeled with old_label\n\
int32 update_index			# the database entry with this index number shall be updated with a new label\n\
string old_label			# all database entries carrying this label are to be updated with new_label\n\
string new_label			# the new label that is supposed to replace the old one\n\
\n\
";
  }

  static const char* value(const  ::cob_people_detection::updateDataActionGoal_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::cob_people_detection::updateDataActionGoal_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::cob_people_detection::updateDataActionGoal_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::cob_people_detection::updateDataActionGoal_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.goal_id);
    stream.next(m.goal);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct updateDataActionGoal_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cob_people_detection::updateDataActionGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::cob_people_detection::updateDataActionGoal_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "goal_id: ";
s << std::endl;
    Printer< ::actionlib_msgs::GoalID_<ContainerAllocator> >::stream(s, indent + "  ", v.goal_id);
    s << indent << "goal: ";
s << std::endl;
    Printer< ::cob_people_detection::updateDataGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.goal);
  }
};


} // namespace message_operations
} // namespace ros

#endif // COB_PEOPLE_DETECTION_MESSAGE_UPDATEDATAACTIONGOAL_H

