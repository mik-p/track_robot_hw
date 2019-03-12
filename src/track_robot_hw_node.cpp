#include <ros/ros.h>

#include <ros/callback_queue.h>
#include <controller_manager/controller_manager.h>

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

    bool init(ros::NodeHandle& root_nh, ros::NodeHandle& rnh, ros::NodeHandle& topics_nh)
    {
      // get hw endpoint
      rnh.getParam("ip_address", ip_address);
      rnh.getParam("port", port);

      // get joint names and num of joint
      std::vector<std::string> joint_names;
      rnh.getParam("joints", joint_names);
      int num_joints = joint_names.size();
      position_state.resize(num_joints);
      velocity_state.resize(num_joints);
      effort_state.resize(num_joints);
      velocity_command.resize(num_joints);
      control_endpoint.resize(num_joints);
      feedback_endpoint.resize(num_joints);

      for(int i = 0; i < num_joints; i++)
      {
        position_state[i] = 0;
        velocity_state[i] = 0;
        effort_state[i] = 0;
        velocity_command[i] = 0;

        hardware_interface::JointStateHandle jointStateHandle(joint_names[i], &position_state[i], &velocity_state[i], &effort_state[i]);
        joint_state_interface.registerHandle(jointStateHandle);

        hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &velocity_command[i]);
        velocity_joint_interface.registerHandle(jointVelocityHandle);

        rnh.param(joint_names[i] + "/control_endpoint", control_endpoint[i]);
        rnh.param(joint_names[i] + "/feedback_endpoint", feedback_endpoint[i]);
      }

      //Register interfaces
      registerInterface(&joint_state_interface);
      registerInterface(&velocity_joint_interface);
    }

    void read(const ros::Time& time, const ros::Duration& period)
    {
      for(int i = 0; i < velocity_state.size(); i++)
      {
        // ROS_INFO("%0.3f", velocity_command[i]);
        velocity_state[i] = velocity_command[i];
        position_state[i] += velocity_state[i] * period.toSec();
      }
    }

    void write(const ros::Time& time, const ros::Duration& period)
    {
      return;
    }

protected:
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::VelocityJointInterface velocity_joint_interface;

    // actual states
    std::vector<double> position_state;
    std::vector<double> velocity_state;
    std::vector<double> effort_state;

    // given setpoints
    std::vector<double> velocity_command;

    // hw endpoints
    std::string ip_address;
    std::string port;

    // read/write endpoints
    std::vector<std::string> control_endpoint;
    std::vector<std::string> feedback_endpoint;
};

} // namespace track_robot_hw


using namespace track_robot_hw;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "track_robot_hw_node");

    ros::CallbackQueue topic_queue;

    ros::AsyncSpinner cm_spinner(1);
    cm_spinner.start();

    ros::NodeHandle nh;
    ros::NodeHandle rnh("~");  //make the robot node handle use the private namespace
    ros::NodeHandle topics_nh("~");  //make the async node handle use the private namespace

    topics_nh.setCallbackQueue(&topic_queue);

    //register realtime publisher

    TrackInterface hw;

    bool init_success = hw.init(nh, rnh, topics_nh);

    controller_manager::ControllerManager cm(&hw, rnh);

    // hw.connect_hardware();

    ros::Duration period(20.0/1000.0); // update rate to suit EnIP implicit io
    //lenze controllers need 4ms to respond to implicit IO

    ROS_INFO("track_robot_hw_node started");
    while(ros::ok()){
        hw.read(ros::Time::now(), period);

        topic_queue.callOne();  // the bus is safe after read

        cm.update(ros::Time::now(), period);

        hw.write(ros::Time::now(), period);

        period.sleep();
    }

    cm_spinner.stop();
    return 0;
}
