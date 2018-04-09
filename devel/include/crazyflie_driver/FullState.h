// Generated by gencpp from file crazyflie_driver/FullState.msg
// DO NOT EDIT!


#ifndef CRAZYFLIE_DRIVER_MESSAGE_FULLSTATE_H
#define CRAZYFLIE_DRIVER_MESSAGE_FULLSTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

namespace crazyflie_driver
{
template <class ContainerAllocator>
struct FullState_
{
  typedef FullState_<ContainerAllocator> Type;

  FullState_()
    : header()
    , pose()
    , twist()
    , acc()  {
    }
  FullState_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , pose(_alloc)
    , twist(_alloc)
    , acc(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef  ::geometry_msgs::Twist_<ContainerAllocator>  _twist_type;
  _twist_type twist;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _acc_type;
  _acc_type acc;




  typedef boost::shared_ptr< ::crazyflie_driver::FullState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::crazyflie_driver::FullState_<ContainerAllocator> const> ConstPtr;

}; // struct FullState_

typedef ::crazyflie_driver::FullState_<std::allocator<void> > FullState;

typedef boost::shared_ptr< ::crazyflie_driver::FullState > FullStatePtr;
typedef boost::shared_ptr< ::crazyflie_driver::FullState const> FullStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::crazyflie_driver::FullState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::crazyflie_driver::FullState_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace crazyflie_driver

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'crazyflie_driver': ['/home/jan/apurva_joshi/crazygames_ws/src/crazyflie_ros/crazyflie_driver/msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::crazyflie_driver::FullState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::crazyflie_driver::FullState_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::crazyflie_driver::FullState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::crazyflie_driver::FullState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::crazyflie_driver::FullState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::crazyflie_driver::FullState_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::crazyflie_driver::FullState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "710c46575863b90cc689d2b26fefeb38";
  }

  static const char* value(const ::crazyflie_driver::FullState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x710c46575863b90cULL;
  static const uint64_t static_value2 = 0xc689d2b26fefeb38ULL;
};

template<class ContainerAllocator>
struct DataType< ::crazyflie_driver::FullState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "crazyflie_driver/FullState";
  }

  static const char* value(const ::crazyflie_driver::FullState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::crazyflie_driver::FullState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
geometry_msgs/Pose pose\n\
geometry_msgs/Twist twist\n\
geometry_msgs/Vector3 acc\n\
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
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Twist\n\
# This expresses velocity in free space broken into its linear and angular parts.\n\
Vector3  linear\n\
Vector3  angular\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
# It is only meant to represent a direction. Therefore, it does not\n\
# make sense to apply a translation to it (e.g., when applying a \n\
# generic rigid transformation to a Vector3, tf2 will only apply the\n\
# rotation). If you want your data to be translatable too, use the\n\
# geometry_msgs/Point message instead.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const ::crazyflie_driver::FullState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::crazyflie_driver::FullState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.pose);
      stream.next(m.twist);
      stream.next(m.acc);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FullState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::crazyflie_driver::FullState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::crazyflie_driver::FullState_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "twist: ";
    s << std::endl;
    Printer< ::geometry_msgs::Twist_<ContainerAllocator> >::stream(s, indent + "  ", v.twist);
    s << indent << "acc: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.acc);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CRAZYFLIE_DRIVER_MESSAGE_FULLSTATE_H
