#pragma once

#include <ros/ros.h>


namespace track_robot_hw
{


class HardwareClient
{
public:
  virtual bool init(ros::NodeHandle& nh) = 0;
  virtual bool set_position(double) = 0;
  virtual bool set_velocity(double) = 0;
  virtual bool set_effort(double) = 0;
  virtual bool get_position(double &) = 0;
  virtual bool get_velocity(double &) = 0;
  virtual bool get_effort(double &) = 0;
};

} // namespace track_robot_hw
