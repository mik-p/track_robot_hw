#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include "track_robot_hw/track_interface.h"
#include "track_robot_hw/rest_client.h"


using namespace track_robot_hw;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "track_robot_hw_node");

    ros::NodeHandle nh; // global namespace
    ros::NodeHandle rnh("~"); // robot private namespace

    TrackInterface hw;

    bool init_success = hw.init(nh, rnh);

    ros::Duration period(hw.get_update_duration()); // update rate

    controller_manager::ControllerManager cm(&hw, rnh);

    ros::AsyncSpinner cm_spinner(1); // controller manager callback thread
    cm_spinner.start();

    ROS_INFO("track_robot_hw_node started");

    while(ros::ok()){
        hw.read(ros::Time::now(), period);

        cm.update(ros::Time::now(), period);

        hw.write(ros::Time::now(), period);

        period.sleep();
    }

    cm_spinner.stop();
    return 0;
}
