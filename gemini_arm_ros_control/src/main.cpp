#include <ros/ros.h>
#include <vector>
#include <string>
#include <iostream>
#include <controller_manager/controller_manager.h>

#include "gemini_arm_ros_control/common.h"
#include "gemini_arm_ros_control/hardware_transmission_interface.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "gemini_arm_ros_control");
  ros::NodeHandle nh;

  int update_freq;
  std::vector<int> gear_ratios;
  std::vector<std::string> jnt_names;

  get_rosparam(nh, update_freq, gear_ratios, jnt_names);

  HwTmIntf gemini_arm(gear_ratios, jnt_names, update_freq);
  controller_manager::ControllerManager cm(&gemini_arm, nh);

  ros::Rate rate(update_freq);
  ros::AsyncSpinner spinner(1);
  spinner.start();


  while(ros::ok())
  {
    gemini_arm.update();
    cm.update(gemini_arm.get_time(), gemini_arm.get_period());
    rate.sleep();
  }

  spinner.stop();

  return 0;
}
