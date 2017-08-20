// roscpp
#include <ros/ros.h>
#include <ros/package.h>

// sensor_msgs
#include <sensor_msgs/Joy.h>

// header from px4_teleop_cmds
#include <px4_teleop_cmds.hpp>

// yaml-cpp
#include <yaml-cpp/yaml.h>

// rc mode consts
const int RC_MODE_ONE = 1;
const int RC_MODE_TWO = 2;

const std::string px4_teleop_path = ros::package::getPath("px4_teleop");


int main(int argc, char **argv){

  ros::init(argc, argv, "px4_teleop_joy");
  ros::NodeHandle nh("/px4_teleop_joy");

  ros::Rate rate(20);

  // Config file path
  std::string joy_config_path;
  nh.param<std::string>
      ("joy_config_path", joy_config_path, px4_teleop_path+"/config/f710.yaml");

  // Rc mode
  int joy_rc_mode;
  nh.param<int>("joy_rc_mode", joy_rc_mode, 1);

  ROS_INFO("RC Mode: %d", joy_rc_mode);
  ROS_INFO("Config: %s", joy_config_path.c_str());

  // Read config file
  YAML::Node config;
  try{
    config = YAML::LoadFile(joy_config_path);
  }catch(YAML::Exception& e){
    ROS_ERROR("Failed to read config file");
    return -1;
  }

  //
  switch(joy_rc_mode){
    case RC_MODE_ONE:
      config = config["mode1"];
      break;
    case RC_MODE_TWO:
      config = config["mode2"];
      break;
  }

  return 0;
}