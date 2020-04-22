#include "gemini_arm_ros_control/hardware_transmission_interface.h"

void HwTmIntf::hw_data_init_()
{
  hw_data_.jnt_curr_pos_.clear();
  hw_data_.jnt_curr_vel_.clear();
  hw_data_.jnt_curr_eff_.clear();

  hw_data_.jnt_cmd_pos_.clear();
  hw_data_.jnt_cmd_vel_.clear();
  hw_data_.jnt_cmd_eff_.clear();

  hw_data_.act_curr_pos_.clear();
  hw_data_.act_curr_vel_.clear();
  hw_data_.act_curr_eff_.clear();

  hw_data_.act_cmd_pos_.clear();
  hw_data_.act_cmd_vel_.clear();
  hw_data_.act_cmd_eff_.clear();


  hw_data_.jnt_curr_pos_.resize(n_dof_);
  hw_data_.jnt_curr_vel_.resize(n_dof_);
  hw_data_.jnt_curr_eff_.resize(n_dof_);

  hw_data_.jnt_cmd_pos_.resize(n_dof_);
  hw_data_.jnt_cmd_vel_.resize(n_dof_);
  hw_data_.jnt_cmd_eff_.resize(n_dof_);

  hw_data_.act_curr_pos_.resize(n_dof_);
  hw_data_.act_curr_vel_.resize(n_dof_);
  hw_data_.act_curr_eff_.resize(n_dof_);

  hw_data_.act_cmd_pos_.resize(n_dof_);
  hw_data_.act_cmd_vel_.resize(n_dof_);
  hw_data_.act_cmd_eff_.resize(n_dof_);
}

void HwTmIntf::hw_register_()
{
  for(size_t i = 0; i < n_dof_; i++)
  {
    hw_data_.jnt_state_interface_.registerHandle(
      hardware_interface::JointStateHandle(
        jnt_names_[i], 
        &hw_data_.jnt_curr_pos_[i],
        &hw_data_.jnt_curr_vel_[i],
        &hw_data_.jnt_curr_eff_[i]));

    hw_data_.jnt_pos_interface_.registerHandle(
      hardware_interface::JointHandle(
        hw_data_.jnt_state_interface_.getHandle(jnt_names_[i]), 
        &hw_data_.jnt_cmd_pos_[i]));
  }
  
  registerInterface(&hw_data_.jnt_state_interface_);
  registerInterface(&hw_data_.jnt_pos_interface_);
}

