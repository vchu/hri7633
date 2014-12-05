/* Auto-generated by genmsg_cpp for file /home/xinyan/groovy_workspace/sandbox/hri7633/sensor_streams/cob_people_perception/cob_people_detection_msgs/msg/Rect.msg */
#ifndef COB_PEOPLE_DETECTION_MSGS_MESSAGE_RECT_H
#define COB_PEOPLE_DETECTION_MSGS_MESSAGE_RECT_H
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


namespace cob_people_detection_msgs
{
template <class ContainerAllocator>
struct Rect_ {
  typedef Rect_<ContainerAllocator> Type;

  Rect_()
  : x(0)
  , y(0)
  , width(0)
  , height(0)
  {
  }

  Rect_(const ContainerAllocator& _alloc)
  : x(0)
  , y(0)
  , width(0)
  , height(0)
  {
  }

  typedef int32_t _x_type;
  int32_t x;

  typedef int32_t _y_type;
  int32_t y;

  typedef int32_t _width_type;
  int32_t width;

  typedef int32_t _height_type;
  int32_t height;


  typedef boost::shared_ptr< ::cob_people_detection_msgs::Rect_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cob_people_detection_msgs::Rect_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Rect
typedef  ::cob_people_detection_msgs::Rect_<std::allocator<void> > Rect;

typedef boost::shared_ptr< ::cob_people_detection_msgs::Rect> RectPtr;
typedef boost::shared_ptr< ::cob_people_detection_msgs::Rect const> RectConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::cob_people_detection_msgs::Rect_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::cob_people_detection_msgs::Rect_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace cob_people_detection_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::cob_people_detection_msgs::Rect_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::cob_people_detection_msgs::Rect_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::cob_people_detection_msgs::Rect_<ContainerAllocator> > {
  static const char* value() 
  {
    return "4425f1067abc7ec2e487d28194eccff4";
  }

  static const char* value(const  ::cob_people_detection_msgs::Rect_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x4425f1067abc7ec2ULL;
  static const uint64_t static_value2 = 0xe487d28194eccff4ULL;
};

template<class ContainerAllocator>
struct DataType< ::cob_people_detection_msgs::Rect_<ContainerAllocator> > {
  static const char* value() 
  {
    return "cob_people_detection_msgs/Rect";
  }

  static const char* value(const  ::cob_people_detection_msgs::Rect_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::cob_people_detection_msgs::Rect_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 x\n\
int32 y\n\
int32 width\n\
int32 height\n\
\n\
";
  }

  static const char* value(const  ::cob_people_detection_msgs::Rect_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::cob_people_detection_msgs::Rect_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::cob_people_detection_msgs::Rect_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.x);
    stream.next(m.y);
    stream.next(m.width);
    stream.next(m.height);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Rect_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cob_people_detection_msgs::Rect_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::cob_people_detection_msgs::Rect_<ContainerAllocator> & v) 
  {
    s << indent << "x: ";
    Printer<int32_t>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<int32_t>::stream(s, indent + "  ", v.y);
    s << indent << "width: ";
    Printer<int32_t>::stream(s, indent + "  ", v.width);
    s << indent << "height: ";
    Printer<int32_t>::stream(s, indent + "  ", v.height);
  }
};


} // namespace message_operations
} // namespace ros

#endif // COB_PEOPLE_DETECTION_MSGS_MESSAGE_RECT_H

