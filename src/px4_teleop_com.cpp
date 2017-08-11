// C/C++ Libraries
#include <iostream>
#include <string>
#include <vector>

// roscpp
#include <ros/ros.h>

// geometry msgs
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

// sensor msgs
#include <sensor_msgs/NavSatFix.h>

// mavros msgs
#include <mavros_msgs/CommandBool.h>


void printUsage(){
    std::cout << "Commands:\n";
    std::cout << "\thelp\n";
    std::cout << "\tarm\n";
    std::cout << "\tdisarm\n";
    std::cout << "\ttakeoff\n";
    std::cout << "\tland\n";
    std::cout << "\tset_home\n";
    std::cout << "\tmove\n";
    std::cout << "\tget_gpos\n";
    std::cout << "\tget_lpos\n";
    std::cout << "\tget_vel" << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "px4_teleop_cmd");
    ros::NodeHandle nh;

    ros::Rate rate(20.0);

    std::string command;
    std::vector<std::string> command_args;

    while(ros::ok()){

        std::cout << "> ";
        std::cin >> command;

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
