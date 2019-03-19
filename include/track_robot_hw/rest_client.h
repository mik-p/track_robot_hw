#pragma once

#include "track_robot_hw/hardware_client.h"

#include <ros/ros.h>


namespace track_robot_hw
{


class HWRESTClient : public HardwareClient
{
public:
  virtual bool init(ros::NodeHandle& nh)
  {
    // get hw endpoint
    nh.getParam("ip_address", ip_address);
    nh.getParam("port", port);

    control_endpoint.resize(num_joints);
    feedback_endpoint.resize(num_joints);

    nh.param(joint_names[i] + "/control_endpoint", control_endpoint[i]);
    nh.param(joint_names[i] + "/feedback_endpoint", feedback_endpoint[i]);
  }

  feedback_t get(std::string endpoint){return {0, 0};}
  void post(std::string endpoint, double cmd){}

private:
  // hw endpoints
  std::string ip_address;
  std::string port;

  // read/write endpoints
  std::vector<std::string> control_endpoint;
  std::vector<std::string> feedback_endpoint;

//  "http://" + ip_address + ":" + port + feedback_endpoint[i]
//  post("http://" + ip_address + ":" + port + control_endpoint[i], velocity_command[i])
};

} // namespace track_robot_hw
