#pragma once

#include "track_robot_hw/hardware_client.h"


namespace track_robot_hw
{


class HardwareJoint
{
public:
  // joint name
  std::string name;

  // joint states
  double position_state;
  double velocity_state;
  double effort_state;

  // setpoints
  double position_command;
  double velocity_command;
  double effort_command;

  // hardware network client
  std::shared_ptr<HardwareClient> client;
};

} // namespace track_robot_hw
