// Generated by gencpp from file bebop_msgs/CommonCalibrationStateMagnetoCalibrationStateChanged.msg
// DO NOT EDIT!


#ifndef BEBOP_MSGS_MESSAGE_COMMONCALIBRATIONSTATEMAGNETOCALIBRATIONSTATECHANGED_H
#define BEBOP_MSGS_MESSAGE_COMMONCALIBRATIONSTATEMAGNETOCALIBRATIONSTATECHANGED_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace bebop_msgs
{
template <class ContainerAllocator>
struct CommonCalibrationStateMagnetoCalibrationStateChanged_
{
  typedef CommonCalibrationStateMagnetoCalibrationStateChanged_<ContainerAllocator> Type;

  CommonCalibrationStateMagnetoCalibrationStateChanged_()
    : header()
    , xAxisCalibration(0)
    , yAxisCalibration(0)
    , zAxisCalibration(0)
    , calibrationFailed(0)  {
    }
  CommonCalibrationStateMagnetoCalibrationStateChanged_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , xAxisCalibration(0)
    , yAxisCalibration(0)
    , zAxisCalibration(0)
    , calibrationFailed(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _xAxisCalibration_type;
  _xAxisCalibration_type xAxisCalibration;

   typedef uint8_t _yAxisCalibration_type;
  _yAxisCalibration_type yAxisCalibration;

   typedef uint8_t _zAxisCalibration_type;
  _zAxisCalibration_type zAxisCalibration;

   typedef uint8_t _calibrationFailed_type;
  _calibrationFailed_type calibrationFailed;





  typedef boost::shared_ptr< ::bebop_msgs::CommonCalibrationStateMagnetoCalibrationStateChanged_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::bebop_msgs::CommonCalibrationStateMagnetoCalibrationStateChanged_<ContainerAllocator> const> ConstPtr;

}; // struct CommonCalibrationStateMagnetoCalibrationStateChanged_

typedef ::bebop_msgs::CommonCalibrationStateMagnetoCalibrationStateChanged_<std::allocator<void> > CommonCalibrationStateMagnetoCalibrationStateChanged;

typedef boost::shared_ptr< ::bebop_msgs::CommonCalibrationStateMagnetoCalibrationStateChanged > CommonCalibrationStateMagnetoCalibrationStateChangedPtr;
typedef boost::shared_ptr< ::bebop_msgs::CommonCalibrationStateMagnetoCalibrationStateChanged const> CommonCalibrationStateMagnetoCalibrationStateChangedConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::bebop_msgs::CommonCalibrationStateMagnetoCalibrationStateChanged_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::bebop_msgs::CommonCalibrationStateMagnetoCalibrationStateChanged_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace bebop_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'bebop_msgs': ['/home/carlos/project_ws/src/bebop_autonomy/bebop_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::bebop_msgs::CommonCalibrationStateMagnetoCalibrationStateChanged_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bebop_msgs::CommonCalibrationStateMagnetoCalibrationStateChanged_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bebop_msgs::CommonCalibrationStateMagnetoCalibrationStateChanged_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bebop_msgs::CommonCalibrationStateMagnetoCalibrationStateChanged_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bebop_msgs::CommonCalibrationStateMagnetoCalibrationStateChanged_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bebop_msgs::CommonCalibrationStateMagnetoCalibrationStateChanged_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::bebop_msgs::CommonCalibrationStateMagnetoCalibrationStateChanged_<ContainerAllocator> >
{
  static const char* value()
  {
    return "89c04aa89f066c20fb00b541abd28d8c";
  }

  static const char* value(const ::bebop_msgs::CommonCalibrationStateMagnetoCalibrationStateChanged_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x89c04aa89f066c20ULL;
  static const uint64_t static_value2 = 0xfb00b541abd28d8cULL;
};

template<class ContainerAllocator>
struct DataType< ::bebop_msgs::CommonCalibrationStateMagnetoCalibrationStateChanged_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bebop_msgs/CommonCalibrationStateMagnetoCalibrationStateChanged";
  }

  static const char* value(const ::bebop_msgs::CommonCalibrationStateMagnetoCalibrationStateChanged_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::bebop_msgs::CommonCalibrationStateMagnetoCalibrationStateChanged_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# CommonCalibrationStateMagnetoCalibrationStateChanged\n"
"# auto-generated from up stream XML files at\n"
"#   github.com/Parrot-Developers/libARCommands/tree/master/Xml\n"
"# To check upstream commit hash, refer to last_build_info file\n"
"# Do not modify this file by hand. Check scripts/meta folder for generator files.\n"
"#\n"
"# SDK Comment: Magneto calib process axis state.\n"
"\n"
"Header header\n"
"\n"
"# State of the x axis (roll) calibration : 1 if calibration is done, 0 otherwise\n"
"uint8 xAxisCalibration\n"
"# State of the y axis (pitch) calibration : 1 if calibration is done, 0 otherwise\n"
"uint8 yAxisCalibration\n"
"# State of the z axis (yaw) calibration : 1 if calibration is done, 0 otherwise\n"
"uint8 zAxisCalibration\n"
"# 1 if calibration has failed, 0 otherwise. If this arg is 1, consider all previous arg as 0\n"
"uint8 calibrationFailed\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::bebop_msgs::CommonCalibrationStateMagnetoCalibrationStateChanged_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::bebop_msgs::CommonCalibrationStateMagnetoCalibrationStateChanged_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.xAxisCalibration);
      stream.next(m.yAxisCalibration);
      stream.next(m.zAxisCalibration);
      stream.next(m.calibrationFailed);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CommonCalibrationStateMagnetoCalibrationStateChanged_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::bebop_msgs::CommonCalibrationStateMagnetoCalibrationStateChanged_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::bebop_msgs::CommonCalibrationStateMagnetoCalibrationStateChanged_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "xAxisCalibration: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.xAxisCalibration);
    s << indent << "yAxisCalibration: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.yAxisCalibration);
    s << indent << "zAxisCalibration: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.zAxisCalibration);
    s << indent << "calibrationFailed: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.calibrationFailed);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BEBOP_MSGS_MESSAGE_COMMONCALIBRATIONSTATEMAGNETOCALIBRATIONSTATECHANGED_H
