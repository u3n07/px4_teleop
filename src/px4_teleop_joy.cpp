// C++ libraries
#include <vector>
#include <string>
#include <stdexcept>

// roscpp
#include <ros/ros.h>
#include <ros/package.h>

// sensor_msgs
#include <sensor_msgs/Joy.h>

// geometry_msgs
#include <geometry_msgs/TwistStamped.h>

// mavros_msgs
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandHome.h>

// header from px4_teleop_cmds
#include <px4_teleop_cmds.hpp>

// yaml-cpp
#include <yaml-cpp/yaml.h>


// rc mode consts
const int RC_MODE_ONE = 1;
const int RC_MODE_TWO = 2;

const std::string px4_teleop_path = ros::package::getPath("px4_teleop");

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// Callback for curr_gpos_sub
sensor_msgs::NavSatFix curr_gpos;
void curr_gpos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    curr_gpos = *msg;
}

// Callback for local_pos_sub
geometry_msgs::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pos = *msg;
}

sensor_msgs::Joy joy_msg;
void joy_cb(const sensor_msgs::Joy::ConstPtr& msg){
  joy_msg = *msg;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "px4_teleop_joy");
  ros::NodeHandle nh("/");

  ros::Rate rate(20);

  // Publisher
  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
          ("mavros/setpoint_velocity/cmd_vel", 100);

  // Subscriber
  ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>
          ("joy_publisher/joy", 100, joy_cb);
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
          ("mavros/state", 100, state_cb);
  ros::Subscriber curr_gpos_sub = nh.subscribe<sensor_msgs::NavSatFix>
          ("mavros/global_position/pose", 10, curr_gpos_cb);
  ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
          ("mavros/local_position/pose", 100, local_pos_cb);

    // Survice Client
  ros::ServiceClient set_hp_client = 
          nh.serviceClient<mavros_msgs::CommandHome>("mavros/cmd/set_home");
  ros::ServiceClient arming_client = 
          nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient takeoff_client = 
          nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
  ros::ServiceClient landing_client =
          nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
  ros::ServiceClient set_mode_client = 
          nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  // Config file path
  std::string joy_config_path;
  nh.param<std::string>("joy_config_path", 
          joy_config_path, px4_teleop_path+"/config/f710.yaml");

  // Rc mode
  int joy_rc_mode;
  nh.param<int>("joy_rc_mode", joy_rc_mode, 1);

  // Takeoff height
  float takeoff_height;
  nh.param<float>("takeoff_height", takeoff_height, 2.0);

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

  // Wait for connection
  while(ros::ok() and current_state.connected){
    ros::spinOnce();
    rate.sleep();
  }

  // Wait for /mavros/global_position/global
  ROS_INFO("Waiting for message from /joy_publisher/joy");
  const std::string topic_joy = "joy_publisher/joy";
  sensor_msgs::Joy joy_init_msg = 
          *ros::topic::waitForMessage<sensor_msgs::Joy>(topic_joy);
  std::vector<float> joy_axes = joy_init_msg.axes;
  std::vector<int32_t> joy_button = joy_init_msg.buttons;

  // Wait for /mavros/global_position/global
  ROS_INFO("Waiting for message from /mavros/global_position/global");
  const std::string topic = "mavros/global_position/global";
  sensor_msgs::NavSatFix init_gpos = 
          *ros::topic::waitForMessage<sensor_msgs::NavSatFix>(topic);
  double init_latitude = init_gpos.latitude;
  double init_longitude = init_gpos.longitude;
  double init_altitude = init_gpos.altitude;
  ROS_INFO("Initial Lat:%f Lon:%f Alt:%f",
           init_latitude, init_longitude, init_altitude);
  
  // set home position as current position
  mavros_msgs::CommandHome set_hp_cmd;
  set_hp_cmd.request.current_gps = true;
  while( not(set_hp_client.call(set_hp_cmd)) and
         set_hp_cmd.response.success){
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("HP set.");

  // set mode as offboard
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  while( not(set_mode_client.call(offb_set_mode)) and
         offb_set_mode.response.success){
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Offboard enabled.");

  ros::Time last_request = ros::Time::now();

  while(ros::ok()){
    // std::cout << joy_msg.axes.size() << std::endl;    
    // std::cout << joy_axes.size() << std::endl;
    // std::cout << config["axes_map"]["roll"].as<int>() << std::endl;

    // std::cout << joy_msg.buttons.size() << std::endl; 
    // std::cout << joy_button.size() << std::endl;
    // std::cout << config["button_map"]["enable"].as<int>() << std::endl;    

    geometry_msgs::TwistStamped cmd_vel_msg;
    try{
      cmd_vel_msg.twist.linear.x = 
              config["axes_scale"]["pitch"].as<double>() * 
              joy_axes.at(config["axes_map"]["pitch"].as<int>());
      cmd_vel_msg.twist.linear.y = 
              config["axes_scale"]["roll"].as<double>() * 
              joy_axes.at(config["axes_map"]["roll"].as<int>());
      cmd_vel_msg.twist.linear.z = 
              config["axes_scale"]["throttle"].as<double>() * 
              joy_axes.at(config["axes_map"]["throttle"].as<int>());
      cmd_vel_msg.twist.angular.z =
              config["axes_scale"]["yaw"].as<double>() * 
              joy_axes.at(config["axes_map"]["yaw"].as<int>());

      cmd_vel_pub.publish(cmd_vel_msg);

      if(joy_button.at(config["button_map"]["arm"].as<int>()) and
          !current_state.armed){
        arm(arming_client, current_state);
      }else if(joy_button.at(config["button_map"]["disarm"].as<int>())){
        disarm(arming_client);
      }else if(joy_button.at(config["button_map"]["takeoff"].as<int>())){
        takeoff(takeoff_client, current_state, local_pos, init_gpos,
                takeoff_height);
      }else if(joy_button.at(config["button_map"]["land"].as<int>())){
        disarm(arming_client);
      }
    }catch(std::out_of_range& e){
      ROS_ERROR_STREAM(e.what());
    }

    if(current_state.mode!="OFFBOARD" and
      (ros::Time::now() - last_request > ros::Duration(0.1))){
      if((set_mode_client.call(offb_set_mode)) and
           offb_set_mode.response.success){
           ROS_INFO("Offboard enabled.");
      }
      last_request = ros::Time::now();
    }
    joy_axes = joy_msg.axes;
    joy_button = joy_msg.buttons;

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}