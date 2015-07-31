// Generated by gencpp from file ardrone_autonomy/navdata_vision_of.msg
// DO NOT EDIT!


#ifndef ARDRONE_AUTONOMY_MESSAGE_NAVDATA_VISION_OF_H
#define ARDRONE_AUTONOMY_MESSAGE_NAVDATA_VISION_OF_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace ardrone_autonomy
{
template <class ContainerAllocator>
struct navdata_vision_of_
{
  typedef navdata_vision_of_<ContainerAllocator> Type;

  navdata_vision_of_()
    : header()
    , drone_time(0.0)
    , tag(0)
    , size(0)
    , of_dx()
    , of_dy()  {
    }
  navdata_vision_of_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , drone_time(0.0)
    , tag(0)
    , size(0)
    , of_dx(_alloc)
    , of_dy(_alloc)  {
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _drone_time_type;
  _drone_time_type drone_time;

   typedef uint16_t _tag_type;
  _tag_type tag;

   typedef uint16_t _size_type;
  _size_type size;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _of_dx_type;
  _of_dx_type of_dx;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _of_dy_type;
  _of_dy_type of_dy;




  typedef boost::shared_ptr< ::ardrone_autonomy::navdata_vision_of_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ardrone_autonomy::navdata_vision_of_<ContainerAllocator> const> ConstPtr;

}; // struct navdata_vision_of_

typedef ::ardrone_autonomy::navdata_vision_of_<std::allocator<void> > navdata_vision_of;

typedef boost::shared_ptr< ::ardrone_autonomy::navdata_vision_of > navdata_vision_ofPtr;
typedef boost::shared_ptr< ::ardrone_autonomy::navdata_vision_of const> navdata_vision_ofConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ardrone_autonomy::navdata_vision_of_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ardrone_autonomy::navdata_vision_of_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ardrone_autonomy

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'ardrone_autonomy': ['/home/cesar/ROS/catkin_ws/src/ardrone_autonomy/msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ardrone_autonomy::navdata_vision_of_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ardrone_autonomy::navdata_vision_of_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ardrone_autonomy::navdata_vision_of_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ardrone_autonomy::navdata_vision_of_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ardrone_autonomy::navdata_vision_of_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ardrone_autonomy::navdata_vision_of_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ardrone_autonomy::navdata_vision_of_<ContainerAllocator> >
{
  static const char* value()
  {
    return "76d31747173a842fbf71db03104edecd";
  }

  static const char* value(const ::ardrone_autonomy::navdata_vision_of_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x76d31747173a842fULL;
  static const uint64_t static_value2 = 0xbf71db03104edecdULL;
};

template<class ContainerAllocator>
struct DataType< ::ardrone_autonomy::navdata_vision_of_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ardrone_autonomy/navdata_vision_of";
  }

  static const char* value(const ::ardrone_autonomy::navdata_vision_of_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ardrone_autonomy::navdata_vision_of_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
float64 drone_time\n\
uint16 tag\n\
uint16 size\n\
float32[] of_dx\n\
float32[] of_dy\n\
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
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::ardrone_autonomy::navdata_vision_of_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ardrone_autonomy::navdata_vision_of_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.drone_time);
      stream.next(m.tag);
      stream.next(m.size);
      stream.next(m.of_dx);
      stream.next(m.of_dy);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct navdata_vision_of_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ardrone_autonomy::navdata_vision_of_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ardrone_autonomy::navdata_vision_of_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "drone_time: ";
    Printer<double>::stream(s, indent + "  ", v.drone_time);
    s << indent << "tag: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.tag);
    s << indent << "size: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.size);
    s << indent << "of_dx[]" << std::endl;
    for (size_t i = 0; i < v.of_dx.size(); ++i)
    {
      s << indent << "  of_dx[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.of_dx[i]);
    }
    s << indent << "of_dy[]" << std::endl;
    for (size_t i = 0; i < v.of_dy.size(); ++i)
    {
      s << indent << "  of_dy[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.of_dy[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // ARDRONE_AUTONOMY_MESSAGE_NAVDATA_VISION_OF_H
