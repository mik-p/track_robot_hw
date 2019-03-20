#pragma once

#include "track_robot_hw/hardware_joint.h"

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>


namespace track_robot_hw
{


class TrackInterface : public hardware_interface::RobotHW, public hardware_interface::HardwareInterface
{
public:
  TrackInterface(){}
  ~TrackInterface(){}

  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_nh);
  void read(const ros::Time& time, const ros::Duration& period);
  void write(const ros::Time& time, const ros::Duration& period);

  double get_update_duration(){return (10.0/1000.0);}

protected:
  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::VelocityJointInterface velocity_joint_interface;

  // hardware joint handles
  std::vector<HardwareJoint> joints;

  // simulation flag
  bool simulated;
};

} // namespace track_robot_hw
