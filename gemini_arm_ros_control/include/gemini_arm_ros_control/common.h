#ifndef COMMON 
#define COMMON 

#include <ros/ros.h>
#include <vector>
#include <string>

void get_rosparam(
  ros::NodeHandle &, int &,
  std::vector<int> &,
  std::vector<std::string> & );

#endif
