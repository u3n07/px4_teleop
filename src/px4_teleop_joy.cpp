// C/C++ Libraries
#include <iostream>
#include <iomanip>
#include <vector>
#include <cstdio>
#include <string>
#include <stdexcept>

#include <fstream>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

// roscpp
#include <ros/ros.h>
#include <ros/package.h>

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

  // Declare joystick file descriptor
  std::string joy_dev;
  nh.param<std::string>("joy_dev", joy_dev, "/dev/input/js0");

  ROS_INFO("RC Mode %d", joy_rc_mode);
  ROS_INFO("Using joystick at %s", joy_dev.c_str());
  ROS_INFO("Config file at %s", joy_config_path.c_str());

  // Read config file
  switch(joy_rc_mode){
    case RC_MODE_ONE:
      break;

    case RC_MODE_TWO:
      break;
  }

  // Set variables about joystick
  int joy_fd = -1;
  int num_of_axis = 0;
  int num_of_buttons = 0;
  char name_of_joystick[80];
  std::vector<char> joy_button;
  std::vector<int> joy_axis;

  // Open joy_dev
  if((joy_fd = open(joy_dev.c_str(), O_RDONLY)) < 0){
    ROS_ERROR("Failed to open %s", joy_dev.c_str());
    return -1;
  }

  // Get info about joystick
  ioctl(joy_fd, JSIOCGAXES, &num_of_axis);
  ioctl(joy_fd, JSIOCGBUTTONS, &num_of_buttons);
  ioctl(joy_fd, JSIOCGNAME(80), &name_of_joystick);

  // Resize vector
  joy_button.resize(num_of_buttons,0);
  joy_axis.resize(num_of_axis,0);

  ROS_INFO("Joystick: %s", name_of_joystick);
  ROS_INFO("Axis: %d", num_of_axis);
  ROS_INFO("Buttons: %d", num_of_buttons);

  // Use non-blocking mode
  fcntl(joy_fd, F_SETFL, O_NONBLOCK);

  while(ros::ok()){

    // https://www.kernel.org/doc/Documentation/input/joystick-api.txt
    //struct js_event{
    //  __u32 time;   /* event timestamp in milliseconds */
    //  __s16 value;  /* value */
    //  __u8 type;    /* event type */
    //  __u8 number;  /* axis/button number */
    //}
    js_event js;

    ssize_t disc = read(joy_fd, &js, sizeof(js_event));

    // the driver will issue synthetic JS_EVENT_INIT on open
    // js.type will be like following if it is issuing INIT BUTTON event
    //     js.type = JS_EVENT_BUTTON | JS_EVENT_INIT
    // So, js.type & ~JS_EVENT_INIT equals
    // JS_EVENT_BUTTON or JS_EVENT_AXIS
    switch(js.type & ~JS_EVENT_INIT){
      case JS_EVENT_AXIS:
        try{
          joy_axis.at((int)js.number) = js.value;
        }catch(std::out_of_range& e){
          break;
        }
        break;

      case JS_EVENT_BUTTON:
        try{
          joy_button.at((int)js.number) = js.value;
        }catch(std::out_of_range& e){
          break;
        }
        break;
    }

    ros::spinOnce();
    rate.sleep();
  }

  close(joy_fd);
  return 0;
}