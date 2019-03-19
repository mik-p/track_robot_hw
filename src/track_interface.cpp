#include "track_robot_hw/track_interface.h"


namespace track_robot_hw
{


bool TrackInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_nh)
{
  // get hw mode
  robot_nh.getParam("simulated", simulated);

  // get joint names and num of joint
  std::vector<std::string> joint_names;
  robot_nh.getParam("joints", joint_names);
  auto num_joints = joint_names.size();
  position_state.resize(num_joints);
  velocity_state.resize(num_joints);
  effort_state.resize(num_joints);
  velocity_command.resize(num_joints);

  for(auto i = 0; i < num_joints; i++)
  {
    position_state[i] = 0;
    velocity_state[i] = 0;
    effort_state[i] = 0;
    velocity_command[i] = 0;

    hardware_interface::JointStateHandle jointStateHandle(joint_names[i], &position_state[i], &velocity_state[i], &effort_state[i]);
    joint_state_interface.registerHandle(jointStateHandle);

    hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &velocity_command[i]);
    velocity_joint_interface.registerHandle(jointVelocityHandle);
  }

  //Register interfaces
  registerInterface(&joint_state_interface);
  registerInterface(&velocity_joint_interface);
}

void TrackInterface::read(const ros::Time& time, const ros::Duration& period)
{
  for(int i = 0; i < velocity_state.size(); i++) // get feedback from hw
  {
    if(!simulated)
    {
//      position_state[i] = client.get_position();
      velocity_state[i] = client.get_velocity();
    }
    else
    {
      velocity_state[i] = velocity_command[i]; // loopback on simulation
      position_state[i] += velocity_state[i] * period.toSec();
    }
  }
}

void TrackInterface::write(const ros::Time& time, const ros::Duration& period)
{
  if(simulated) return; // do nothing if in simulation

  // probs apply command limits here

  for(int i = 0; i < velocity_state.size(); i++) // publish commands to hw
  {
    client.set_velocity(velocity_command[i]);
  }
}

} // namespace track_robot_hw
