// C/C++ libraries
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <csignal>

// roscpp
#include <ros/ros.h>

// geometry_msgs
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

// sensor_msgs
#include <sensor_msgs/NavSatFix.h>

// mavros_msgs
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandHome.h>

// Callback for state_sub
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// Callback for curr_gpos_sub
double curr_latitude;
double curr_longitude;
double curr_altitude;
void curr_gpos_cb(const sensor_msgs::NavSatFix::ConstPtr& msgptr){
    sensor_msgs::NavSatFix msg = *msgptr;
    curr_latitude = msg.latitude;
    curr_longitude = msg.longitude;
    curr_altitude = msg.altitude;
}

// Callback for local_pos_sub
geometry_msgs::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pos = *msg;
}

// Print a list of commands
void printUsage(){
    std::cout << "Available Commands:" << std::endl;
    std::cout << "\tarm" << std::endl;
    std::cout << "\tdisarm" << std::endl;
    std::cout << "\ttakeoff" << std::endl;
    std::cout << "\tland" << std::endl;
    std::cout << "\tcmd_vel" << std::endl;
    std::cout << "\thelp" << std::endl;
    std::cout << "\tquit" << std::endl;
}

// Function for arming
void arm(ros::ServiceClient& client){

    ros::Rate rate(20);

    if(current_state.armed){
        ROS_WARN("Arm Rejected. Already armed.");
        return;
    }
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    while( not(client.call(arm_cmd)) and
              arm_cmd.response.success){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Vehicle armed.");
}

// Function for takeoff
void takeoff(ros::ServiceClient& client,
             double home_alt, double home_lon, double home_lat, double height){

    ros::Rate rate(20);

    if(!current_state.armed){
        ROS_WARN("Takeoff rejected. Arm first.");
        return;
    }

    if(local_pos.pose.position.z > 1.0){
        ROS_WARN("Takeoff rejected. Already took off.");
        return;
    }

    mavros_msgs::CommandTOL takeoff_cmd;
    takeoff_cmd.request.altitude = home_alt + height;
    takeoff_cmd.request.longitude = home_lon;
    takeoff_cmd.request.latitude = home_lat;

    ROS_DEBUG("set lat: %f, lon: %f, alt: %f", home_lat, home_lon, home_alt);

    while( not(client.call(takeoff_cmd)) and
               takeoff_cmd.response.success){
        ros::spinOnce();
        rate.sleep();
    }
    while(local_pos.pose.position.z < height-0.1){
        ros::spinOnce();
        rate.sleep();
        ROS_DEBUG("Current lat: %f, lon: %f, alt: %f",
                   curr_latitude, curr_longitude, curr_altitude);
    }
    ROS_INFO("Vehicle tookoff.");
}

// Function for landing
void land(ros::ServiceClient& client, double home_alt){

    ros::Rate rate(20);

    // land
    mavros_msgs::CommandTOL landing_cmd;
    landing_cmd.request.altitude = curr_altitude - (curr_altitude-home_alt) + 0.5;
    landing_cmd.request.longitude = curr_longitude;
    landing_cmd.request.latitude = curr_latitude;

    while( not(client.call(landing_cmd)) and
               landing_cmd.response.success){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Vehicle landing...");

    while(local_pos.pose.position.z > 0.1){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Vehicle landed.");
}

// Function for disarming
void disarm(ros::ServiceClient& client){

    ros::Rate rate(20);

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = false;
    while(!client.call(arm_cmd) && arm_cmd.response.success){
    }
    ROS_INFO("Vehicle disarmed");
}

// Send velocity operation to move vehicle
void cmd_vel(ros::Publisher& pub, ros::ServiceClient& client,
             double dt, double vx, double vy, double vz, double ang_z){

    ros::Rate rate(20);

    ros::Time start = ros::Time::now();
    ros::Time last_request = ros::Time::now();

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    geometry_msgs::TwistStamped target_vel_msg;
    target_vel_msg.twist.linear.x = vx;
    target_vel_msg.twist.linear.y = vy;
    target_vel_msg.twist.linear.z = vz;
    target_vel_msg.twist.angular.z = ang_z;

    ROS_INFO("Vehicle moving ...");

    while (ros::Time::now() - start < ros::Duration(dt)){
        pub.publish(target_vel_msg);

        if(current_state.mode!="OFFBOARD" and
           (ros::Time::now() - last_request > ros::Duration(0.1))){
            if((client.call(offb_set_mode)) and
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled.");
            }
            last_request = ros::Time::now();
        }
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Vehicle arrived destination.");


    mavros_msgs::SetMode loiter_set_mode;
    loiter_set_mode.request.custom_mode = "AUTO.LOITER";
    while( not(client.call(loiter_set_mode)) and
           loiter_set_mode.response.success){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Vehicle hovering.");
}

// Catch Ctrl+C
void interrupt_handler(int sig){
    std::signal(sig, interrupt_handler);
    exit(0);
}

int main(int argc, char** argv){

    ros::init(argc, argv, "px4_teleop_com");
    ros::NodeHandle nh;

    // Subscriber
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 100, state_cb);
    ros::Subscriber curr_gpos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/pose", 10, curr_gpos_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 100, local_pos_cb);

    // Publisher
    ros::Publisher target_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 100);

    // Survice Client
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
               ("mavros/cmd/arming");
    ros::ServiceClient set_hp_client = nh.serviceClient<mavros_msgs::CommandHome>
               ("mavros/cmd/set_home");
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>
               ("mavros/cmd/takeoff");
    ros::ServiceClient landing_client = nh.serviceClient<mavros_msgs::CommandTOL>
               ("mavros/cmd/land");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
               ("mavros/set_mode");

    ros::Rate rate(20);

    // Command string and vector of commands
    std::string command, buffer;
    std::vector<std::string> command_args;

    // Set signal handler
    // std::signal() returns a pointer to the handler function
    // that was in charge of handling this signal before the call of std::signal()
    void (*old)(int); // Pointer to old handler function
    old = std::signal(SIGINT, interrupt_handler);

    // Wait for connection
    while(ros::ok() and current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // Wait for /mavros/global_position/global
    ROS_INFO("Waiting for message from /mavros/global_position/global");
    const std::string topic = "mavros/global_position/global";
    sensor_msgs::NavSatFix init_gpos = *ros::topic::waitForMessage<sensor_msgs::NavSatFix>(topic);
    double init_latitude = init_gpos.latitude;
    double init_longitude = init_gpos.longitude;
    double init_altitude = init_gpos.altitude;
    ROS_INFO("Initial Lat:%f Lon:%f Alt:%f", init_latitude, init_longitude, init_altitude);

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


    while(ros::ok()){

        // Wait for input
        std::cout << "com> ";
        std::getline(std::cin, command);
        if(std::cin.fail() or std::cin.eof()){
            std::cin.clear();
        }

        std::stringstream ss(command);

        command_args.clear();
        command_args.shrink_to_fit();

        while(not ss.eof()){
            ss >> buffer;
            command_args.push_back(buffer);
        }

        if(command_args[0]=="arm" and command_args.size()==1)
        {
            arm(arming_client);
        }
        else if(command_args[0]=="takeoff" and command_args.size()<3)
        {
            try{
                takeoff(takeoff_client,
                        init_altitude, init_longitude, init_latitude,
                        std::stod(command_args.at(1)));
            }catch(...){
                std::cout << "Invalid argument." << std::endl;
                std::cout << "Usage:\n\ttakeoff height" << std::endl;
            }
        }
        else if(command_args[0]=="land" and command_args.size()==1)
        {
            land(landing_client, init_altitude);
        }
        else if(command_args[0]=="disarm" and command_args.size()==1)
        {
            disarm(arming_client);
        }
        else if(command_args[0]=="help" and command_args.size()==1)
        {
            printUsage();
        }
        else if(command_args[0]=="quit" and command_args.size()==1)
        {
            exit(0);
        }
        else if(command_args[0]=="cmd_vel" and command_args.size()<8)
        {
            try{
                cmd_vel(target_vel_pub, set_mode_client,
                        std::stod(command_args.at(1)),
                        std::stod(command_args.at(2)),
                        std::stod(command_args.at(3)),
                        std::stod(command_args.at(4)),
                        std::stod(command_args.at(5)));
            }catch(...){
                std::cout << "Invalid argument." << std::endl;
                std::cout << "Usage:\n\tcmd_vel dt vx vy vz ang_z" << std::endl;
            }
        }
        else
        {
            std::cout << "Invalid command." << std::endl;
            std::cout << "Type \"help\" to print available commands." << std::endl;
        }

        ros::spinOnce();
        rate.sleep();
    }

    // Restore initial handler
    std::signal(SIGINT, old);
    return 0;
}
