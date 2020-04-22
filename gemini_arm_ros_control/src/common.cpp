#include <ros/ros.h>
#include "gemini_arm_ros_control/common.h"

void get_rosparam(
  ros::NodeHandle &nh, 
  int &update_freq,
  std::vector<int> &gear_ratios, 
  std::vector<std::string> &jnt_names)
{
  if(!nh.getParam("gemini_arm/update_freq", update_freq))
    ROS_ERROR("Please specify the update frequency (update_freq)");

  if(!nh.getParam("gemini_arm/gear_ratios", gear_ratios))
    ROS_ERROR("Please specify the value of gear ratios (gear_ratios)");

  if(!nh.getParam("gemini_arm/jnt_names", jnt_names))
    ROS_ERROR("Please specify the names of joints (jnt_names)");
}
