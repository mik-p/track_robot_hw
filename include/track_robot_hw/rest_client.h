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
    // get hw endpoints
    nh.getParam("ip_address", ip_address);
    nh.getParam("port", port);
    nh.param("velocity_control_endpoint", velocity_control_endpoint);
    nh.param("velocity_feedback_endpoint", velocity_feedback_endpoint);
    return true; // TODO check for errors
  }

  virtual bool set_position(double) {return false;}
  virtual bool set_effort(double) {return false;}

  virtual bool set_velocity(double vel)
  {
    return post(
          "http://" + ip_address + velocity_control_endpoint,
          "cmd_vel=" + std::to_string(vel)
          );
  }

  virtual bool get_position(double &) {return false;}

  virtual bool get_velocity(double& vel)
  {
    std::string data;

    bool result = get(
          "http://" + ip_address + velocity_feedback_endpoint,
          data
          );

    if(result)
    {
      ROS_INFO("HTTP response: %s", data.c_str());
//      vel = data;
      return true;
    }
    else
    {
      ROS_ERROR("invalid HWRESTClient::get() false result");
    }
    return false;
  }

  virtual bool get_effort(double &) {return false;}

  static size_t data_callback(void *contents, size_t size, size_t nmemb, void *userp);

private:
  bool get(std::string url, std::string& data);
  bool post(std::string url, std::string post_fields);

  // hw endpoints
  std::string ip_address;
  std::string port;

  // read/write endpoints
  std::string velocity_control_endpoint;
  std::string velocity_feedback_endpoint;
};

} // namespace track_robot_hw
