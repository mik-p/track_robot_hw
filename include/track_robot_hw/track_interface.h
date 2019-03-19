#pragma once

#include "track_robot_hw/hardware_client.h"

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

  // actual states
  std::vector<double> position_state;
  std::vector<double> velocity_state;
  std::vector<double> effort_state;

  // given setpoints
  std::vector<double> velocity_command;

  // simulation flag
  bool simulated;

  // hardware network client
  HardwareClient& client;
};

} // namespace track_robot_hw
