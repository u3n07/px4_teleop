// C/C++ Libraries
#include <iostream>
#include <iomanip>
#include <vector>
#include <cstdio>
#include <string>
#include <stdexcept>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

// roscpp
#include <ros/ros.h>

// geometry_msgs
#include <geometry_msgs/TwistStamped.h>

// mavros_msgs
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandHome.h>

int main(int argc, char **argv){
  
  ros::init(argc, argv, "px4_teleop_joy");
  ros::NodeHandle nh("/px4_teleop_joy");

  ros::Rate rate(20);

  // Mode
  int rc_mode;
  nh.param<int>("rc_mode", rc_mode, 1);

  // Declare joystick file descriptor
  std::string joy_dev;
  nh.param<std::string>("joy_dev", joy_dev, "/dev/input/js0");

  ROS_INFO("RC Mode %d", rc_mode);
  ROS_INFO("Using joystick at %s", joy_dev.c_str());

  // Set variables about joystick
  int joy_fd = -1;
  int num_of_axis = 0;
  int num_of_buttons = 0;
  char name_of_joystick[80];
  std::vector<char> joy_button;
  std::vector<int> joy_axis;

  // Open joy_dev
  try{
    joy_fd = open(joy_dev.c_str(), O_RDONLY);
  }catch(...){
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

    read(joy_fd, &js, sizeof(js_event));

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

    std::cout << "axis/10000: ";
    for(size_t i(0);i<joy_axis.size();++i)
    std::cout<<" "<<std::setw(2)<<joy_axis[i]/10000;
    std::cout<<std::endl;

    std::cout<<"  button: ";
    for(size_t i(0);i<joy_button.size();++i)
    std::cout<<" "<<(int)joy_button[i];
    std::cout<<std::endl;        

    ros::spinOnce();
    rate.sleep();
  }

  close(joy_fd);
  return 0;
}