#include <TFmini.h>
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tfmini_ros_node");
  ros::NodeHandle nh("~");
  std::string id = "range";
  std::string portName;
  int baud_rate;
  benewake::TFmini *tfmini_obj;

  nh.param("serial_port", portName, std::string("/dev/ttyUSB0"));
  nh.param("baud_rate", baud_rate, 115200);

  ROS_INFO_STREAM("Trying to connect to TFMINI ...");
  tfmini_obj = new benewake::TFmini(portName, baud_rate);
  ROS_INFO_STREAM("Connected successfully!!");
  ros::Publisher pub_range = nh.advertise<sensor_msgs::Range>(id, 1000, true);
  ros::Publisher pub_status = nh.advertise<std_msgs::String>("status", 1000, true);
  sensor_msgs::Range TFmini_range;
  std_msgs::String TFmini_status;
  TFmini_range.radiation_type = sensor_msgs::Range::INFRARED;
  TFmini_range.field_of_view = 0.04;
  TFmini_range.min_range = 0.3;
  TFmini_range.max_range = 12;
  TFmini_range.header.frame_id = id;
  float dist = 0;
  ROS_INFO_STREAM("Start processing ...");

  while(ros::master::check() && ros::ok())
  {
    ros::spinOnce();
    dist = tfmini_obj->getDist();
    if(dist > 0 && dist < TFmini_range.max_range)
    {
      TFmini_range.range = dist*1000;
      TFmini_status.data = "ok";
      TFmini_range.header.stamp = ros::Time::now();
      pub_range.publish(TFmini_range);
    }
    else if(dist == -1.0)
    {
      ROS_ERROR_STREAM("Failed to read data. TFmini ros node stopped!");
      TFmini_status.data = "error";
      break;
    }
    else if(dist == 0.0)
    {
      ROS_ERROR_STREAM("Data validation error!");
      TFmini_status.data = "error";
    }
    else {
      TFmini_status.data = "out";
    }
    pub_status.publish(TFmini_status);
  }

  tfmini_obj->closePort();
}
