#include "track_robot_hw/track_interface.h"

#include "track_robot_hw/rest_client.h"


namespace track_robot_hw
{


bool TrackInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_nh)
{
  // get hw mode
  robot_nh.getParam("simulated", simulated);
  if(simulated)
  {
    ROS_INFO("hardware simulated");
  }

  // get joint names
  std::vector<std::string> joint_names;
  robot_nh.getParam("joints", joint_names);
  ROS_INFO("found %i joints", (int)joint_names.size());

  // destroy any old resources
  joints.clear();

  // generate hw joint resource list
  for(auto name : joint_names)
  {
    HardwareJoint joint;
    joint.name = name;
    joints.push_back(joint);
    ROS_INFO("generated joint: %s", joint.name.c_str());
  }

  // init hw resources
  for(HardwareJoint& joint : joints)
  {
    joint.position_state = 0;
    joint.velocity_state = 0;
    joint.effort_state = 0;
    joint.velocity_command = 0;

    joint.client = std::make_shared<HWRESTClient>();
    ros::NodeHandle jnh("~/" + joint.name);
    joint.client->init(jnh); // TODO implement simulation client

    hardware_interface::JointStateHandle jointStateHandle(joint.name, &joint.position_state, &joint.velocity_state, &joint.effort_state);
    joint_state_interface.registerHandle(jointStateHandle);

    hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &joint.velocity_command);
    velocity_joint_interface.registerHandle(jointVelocityHandle);

    ROS_INFO("registered joint: %s", joint.name.c_str());
  }

  //Register interfaces
  registerInterface(&joint_state_interface);
  registerInterface(&velocity_joint_interface);
}

void TrackInterface::read(const ros::Time& time, const ros::Duration& period)
{
  for(HardwareJoint& joint : joints) // get feedback from hw
  {
    if(!simulated)
    {
//      position_state[i] = client->get_position();
      joint.velocity_state = joint.client->get_velocity(joint.velocity_state);
    }
    else
    {
      joint.velocity_state = joint.velocity_command; // loopback on simulation
      joint.position_state += joint.velocity_state * period.toSec(); // fix integration
    }
  }
}

void TrackInterface::write(const ros::Time& time, const ros::Duration& period)
{
  if(simulated) return; // do nothing if in simulation

  // probs apply command limits here

  for(HardwareJoint& joint : joints) // publish commands to hw
  {
    joint.client->set_velocity(joint.velocity_command);
  }
}

} // namespace track_robot_hw
