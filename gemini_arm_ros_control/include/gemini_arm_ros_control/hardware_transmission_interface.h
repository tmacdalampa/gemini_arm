#ifndef HARDEARE_TRANSMISSION_INTERFACE
#define HARDEARE_TRANSMISSION_INTERFACE

#include "gemini_arm_ros_control/hardware_data.h"
#include "gemini_arm_ros_control/transmission_data.h"
#include <hardware_interface/robot_hw.h>

class HwTmIntf: public hardware_interface::RobotHW
{
  public:
    HwTmIntf(std::vector<int>, std::vector<std::string>, int);
    ~HwTmIntf();
    void update();
    void set_n_dof();
    ros::Time get_time() const;
    ros::Duration get_period() const;

  private:
    void hw_data_init_();
    void tm_data_init_();
    void hw_register_();
    void tm_register_();
    void tm_wrap_();
    //ros::Time get_time_() const;
    //ros::Duration get_period_(int) const;

    HwData hw_data_;
    TmData tm_data_;

    std::vector<int> gear_ratios_;
    std::vector<std::string> jnt_names_;
    int n_dof_;
    int update_freq_;
    
    const int cAxisBias[6]  = {-3445324, 1852664, -2665457, -1830796, -6546544, -318587}; //encoder initial postion
    const int cAxisGearRatio[6]  = {101, 121, 101, 101, 101, 101}; //gear ratio
    //const int cAxisVel[6]  = {15000, 15000, 15000, 20000, 10000, 20000};
    const int cAxisVel[6]  = {20000, 20000, 20000, 20000, 20000, 20000}; //axis maximum velocity


};

#endif
