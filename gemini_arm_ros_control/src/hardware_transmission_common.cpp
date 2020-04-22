#include "scorpio_arm_ros_control/hardware_transmission_interface.h"
#include "aps168/type_def.h"
#include "aps168/APS168.h"
#include "aps168/ErrorCodeDef.h"
#include "aps168/APS_Define.h"

#include "aps168/sample_main.h"
#include "aps168/APS_Linux_sample.h"

#define OPT_ABS 0
#define PI 3.1415926
//#define ENC_FULL 8388608
#define ENC_FULL 131072

I32 chk_initial = 0;

void Initial_StartFieldBus(void)
{
    I32 ret=0, board_id=0;

    printf("Start Initial & start field bus...............\n");

    ret = APS_initial(&board_id,0);
    printf("Initial, ret=%d\n", ret);

    ret = APS_start_field_bus(0,0,0);
    printf("start field bus ret=%d\n", ret);

    if(ret!=0)
    {
        chk_initial = 0;
        printf("Initial_StartFieldBus failed......\n\n");
    }
    else{
        chk_initial = 1;
        printf("Initial_StartFieldBus success......\n\n");
    }
}

HwTmIntf::HwTmIntf(
  std::vector<int> gear_ratios, 
  std::vector<std::string> jnt_names, 
  int update_freq)
  : n_dof_(jnt_names.size()),
    update_freq_(update_freq)
{
  gear_ratios_ = gear_ratios;
  jnt_names_ = jnt_names;

  // Initialize gear ratios
  for(size_t i = 0; i < gear_ratios.size(); i++)
    tm_data_.trans_.emplace_back(gear_ratios_[i], 0.0);

  // Initialize hardware data
  hw_data_init_();

  // Initialize transmission data
  tm_data_init_();

  // Register hardware data
  hw_register_();

  // Wrap transmission data
  tm_wrap_();

  // Register transmission data
  tm_register_();

  // Start ethercat master
  Initial_StartFieldBus();

  int ret = 0;
  for(int id = 0; id < jnt_names_.size(); id++) 
  //for(int id = 5; id < 6; id++) 
  {
    // Config speed profile parameters.
    ret = APS_set_axis_param_f(id, PRA_SF,  0.5);
    ret = APS_set_axis_param_f(id, PRA_ACC, 15000.0 * cAxisGearRatio[id]);
    ret = APS_set_axis_param_f(id, PRA_DEC, 15000.0 * cAxisGearRatio[id]);
    ret = APS_set_axis_param_f(id, PRA_VM,  cAxisVel[id] * cAxisGearRatio[id]);

    // Servo on
    if(!((APS_motion_io_status(id) >> MIO_SVON) & 1))
    {
      ret = APS_set_servo_on(id, 1);
      //ret = APS_set_command(id, 0);
      //ret = APS_set_position( Axis_ID, 0 );
    }
  }
  //ret = APS_ptp(5, OPT_ABS, cAxisBias[5], 0 );
  //ret = APS_ptp(5, OPT_ABS, 86103, 0 );
}

HwTmIntf::~HwTmIntf()
{
  int ret = 0;
  ret = APS_stop_field_bus(0,0);
  printf("ret=%d\n", ret);

  ret = APS_close();
  printf("ret=%d\n", ret);
}

void HwTmIntf::update()
{
  static int ret = 0;
  static double pos = 0;
  tm_data_.act_to_jnt_state_.propagate();
  tm_data_.jnt_to_act_state_.propagate();

  // Show the commands written into actuator
#if 0
  std::cout << "======= Write ========" << std::endl;
  for(int id = 0; id < jnt_names_.size(); id++)
  {
    std::cout << jnt_names_[id] << ": " << cAxisBias[id] + hw_data_.act_cmd_pos_[id] * (ENC_FULL * cAxisGearRatio[id] / (2 * PI)) << std::endl;
    std::cout << jnt_names_[id] << ": " << hw_data_.act_cmd_pos_[id] << std::endl;
  }
#endif
#if 1
  // Write data to motors
  //for(int id = 0; id < 2; id++) 
  for(int id = 0; id < jnt_names_.size(); id++) 
    ret = APS_ptp(id, OPT_ABS, cAxisBias[id] + hw_data_.act_cmd_pos_[id] * (ENC_FULL * cAxisGearRatio[id] / (2 * PI)), 0 );
    //ret = APS_ptp(id, OPT_ABS, cAxisBias[id], 0 );
#endif
  // Read data from motors
  //for(int id = 0; id < 2; id++) 
  for(int id = 0; id < jnt_names_.size(); id++) 
  {
    ret = APS_get_position_f(id, &pos);
    if(ret != 0)
    {
        ROS_ERROR("log position error!\n");
    }
    //std::cout << jnt_names_[id] << ": " << (pos - cAxisBias[id]) * (2 * PI / ENC_FULL) << std::endl;
    hw_data_.act_curr_pos_[id] = (pos - cAxisBias[id]) * (2 * PI / ENC_FULL / cAxisGearRatio[id]); 
  }


  // Read actuator encoder (fake)
#if 0
  for(int i = 2; i < jnt_names_.size(); i++) 
    hw_data_.act_curr_pos_[i] = hw_data_.act_cmd_pos_[i];
#endif

#if 0
  std::cout << "======= Read ========" << std::endl;
  for(int i = 0; i < jnt_names_.size(); i++)
  {
    //std::cout << jnt_names_[i] << ": " << hw_data_.act_cmd_pos_[i] << std::endl;
    std::cout << jnt_names_[i] << ": " << hw_data_.act_curr_pos_[i] << std::endl;
  }
#endif

}

ros::Time HwTmIntf::get_time() const 
{
    return ros::Time::now();
}

ros::Duration HwTmIntf::get_period() const 
{
    return ros::Duration(1.0 / update_freq_);
}


