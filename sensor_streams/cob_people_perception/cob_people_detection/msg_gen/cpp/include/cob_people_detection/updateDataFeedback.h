/* Auto-generated by genmsg_cpp for file /home/xinyan/groovy_workspace/sandbox/hri7633/sensor_streams/cob_people_perception/cob_people_detection/msg/updateDataFeedback.msg */
#ifndef COB_PEOPLE_DETECTION_MESSAGE_UPDATEDATAFEEDBACK_H
#define COB_PEOPLE_DETECTION_MESSAGE_UPDATEDATAFEEDBACK_H
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


namespace cob_people_detection
{
template <class ContainerAllocator>
struct updateDataFeedback_ {
  typedef updateDataFeedback_<ContainerAllocator> Type;

  updateDataFeedback_()
  {
  }

  updateDataFeedback_(const ContainerAllocator& _alloc)
  {
  }


  typedef boost::shared_ptr< ::cob_people_detection::updateDataFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cob_people_detection::updateDataFeedback_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct updateDataFeedback
typedef  ::cob_people_detection::updateDataFeedback_<std::allocator<void> > updateDataFeedback;

typedef boost::shared_ptr< ::cob_people_detection::updateDataFeedback> updateDataFeedbackPtr;
typedef boost::shared_ptr< ::cob_people_detection::updateDataFeedback const> updateDataFeedbackConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::cob_people_detection::updateDataFeedback_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::cob_people_detection::updateDataFeedback_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace cob_people_detection

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::cob_people_detection::updateDataFeedback_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::cob_people_detection::updateDataFeedback_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::cob_people_detection::updateDataFeedback_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::cob_people_detection::updateDataFeedback_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::cob_people_detection::updateDataFeedback_<ContainerAllocator> > {
  static const char* value() 
  {
    return "cob_people_detection/updateDataFeedback";
  }

  static const char* value(const  ::cob_people_detection::updateDataFeedback_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::cob_people_detection::updateDataFeedback_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# feedback message\n\
\n\
\n\
";
  }

  static const char* value(const  ::cob_people_detection::updateDataFeedback_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::cob_people_detection::updateDataFeedback_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::cob_people_detection::updateDataFeedback_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct updateDataFeedback_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cob_people_detection::updateDataFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::cob_people_detection::updateDataFeedback_<ContainerAllocator> & v) 
  {
  }
};


} // namespace message_operations
} // namespace ros

#endif // COB_PEOPLE_DETECTION_MESSAGE_UPDATEDATAFEEDBACK_H

