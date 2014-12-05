/* Auto-generated by genmsg_cpp for file /home/xinyan/groovy_workspace/sandbox/hri7633/sensor_streams/cob_people_perception/cob_people_detection/srv/recognitionTrigger.srv */
#ifndef COB_PEOPLE_DETECTION_SERVICE_RECOGNITIONTRIGGER_H
#define COB_PEOPLE_DETECTION_SERVICE_RECOGNITIONTRIGGER_H
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

#include "ros/service_traits.h"




namespace cob_people_detection
{
template <class ContainerAllocator>
struct recognitionTriggerRequest_ {
  typedef recognitionTriggerRequest_<ContainerAllocator> Type;

  recognitionTriggerRequest_()
  : target_frame_rate(0.0)
  {
  }

  recognitionTriggerRequest_(const ContainerAllocator& _alloc)
  : target_frame_rate(0.0)
  {
  }

  typedef float _target_frame_rate_type;
  float target_frame_rate;


  typedef boost::shared_ptr< ::cob_people_detection::recognitionTriggerRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cob_people_detection::recognitionTriggerRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct recognitionTriggerRequest
typedef  ::cob_people_detection::recognitionTriggerRequest_<std::allocator<void> > recognitionTriggerRequest;

typedef boost::shared_ptr< ::cob_people_detection::recognitionTriggerRequest> recognitionTriggerRequestPtr;
typedef boost::shared_ptr< ::cob_people_detection::recognitionTriggerRequest const> recognitionTriggerRequestConstPtr;



template <class ContainerAllocator>
struct recognitionTriggerResponse_ {
  typedef recognitionTriggerResponse_<ContainerAllocator> Type;

  recognitionTriggerResponse_()
  {
  }

  recognitionTriggerResponse_(const ContainerAllocator& _alloc)
  {
  }


  typedef boost::shared_ptr< ::cob_people_detection::recognitionTriggerResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cob_people_detection::recognitionTriggerResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct recognitionTriggerResponse
typedef  ::cob_people_detection::recognitionTriggerResponse_<std::allocator<void> > recognitionTriggerResponse;

typedef boost::shared_ptr< ::cob_people_detection::recognitionTriggerResponse> recognitionTriggerResponsePtr;
typedef boost::shared_ptr< ::cob_people_detection::recognitionTriggerResponse const> recognitionTriggerResponseConstPtr;


struct recognitionTrigger
{

typedef recognitionTriggerRequest Request;
typedef recognitionTriggerResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct recognitionTrigger
} // namespace cob_people_detection

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::cob_people_detection::recognitionTriggerRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::cob_people_detection::recognitionTriggerRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::cob_people_detection::recognitionTriggerRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "0fb2bbc7af64ffad2d27794be36bc198";
  }

  static const char* value(const  ::cob_people_detection::recognitionTriggerRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x0fb2bbc7af64ffadULL;
  static const uint64_t static_value2 = 0x2d27794be36bc198ULL;
};

template<class ContainerAllocator>
struct DataType< ::cob_people_detection::recognitionTriggerRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "cob_people_detection/recognitionTriggerRequest";
  }

  static const char* value(const  ::cob_people_detection::recognitionTriggerRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::cob_people_detection::recognitionTriggerRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
float32 target_frame_rate\n\
\n\
";
  }

  static const char* value(const  ::cob_people_detection::recognitionTriggerRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::cob_people_detection::recognitionTriggerRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::cob_people_detection::recognitionTriggerResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::cob_people_detection::recognitionTriggerResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::cob_people_detection::recognitionTriggerResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::cob_people_detection::recognitionTriggerResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::cob_people_detection::recognitionTriggerResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "cob_people_detection/recognitionTriggerResponse";
  }

  static const char* value(const  ::cob_people_detection::recognitionTriggerResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::cob_people_detection::recognitionTriggerResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
";
  }

  static const char* value(const  ::cob_people_detection::recognitionTriggerResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::cob_people_detection::recognitionTriggerResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::cob_people_detection::recognitionTriggerRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.target_frame_rate);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct recognitionTriggerRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::cob_people_detection::recognitionTriggerResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct recognitionTriggerResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<cob_people_detection::recognitionTrigger> {
  static const char* value() 
  {
    return "0fb2bbc7af64ffad2d27794be36bc198";
  }

  static const char* value(const cob_people_detection::recognitionTrigger&) { return value(); } 
};

template<>
struct DataType<cob_people_detection::recognitionTrigger> {
  static const char* value() 
  {
    return "cob_people_detection/recognitionTrigger";
  }

  static const char* value(const cob_people_detection::recognitionTrigger&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<cob_people_detection::recognitionTriggerRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "0fb2bbc7af64ffad2d27794be36bc198";
  }

  static const char* value(const cob_people_detection::recognitionTriggerRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<cob_people_detection::recognitionTriggerRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "cob_people_detection/recognitionTrigger";
  }

  static const char* value(const cob_people_detection::recognitionTriggerRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<cob_people_detection::recognitionTriggerResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "0fb2bbc7af64ffad2d27794be36bc198";
  }

  static const char* value(const cob_people_detection::recognitionTriggerResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<cob_people_detection::recognitionTriggerResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "cob_people_detection/recognitionTrigger";
  }

  static const char* value(const cob_people_detection::recognitionTriggerResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // COB_PEOPLE_DETECTION_SERVICE_RECOGNITIONTRIGGER_H
