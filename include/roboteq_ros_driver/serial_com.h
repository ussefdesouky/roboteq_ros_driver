#pragma once 

#include <iostream>
#include <string>
#include <sstream>
#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include "roboteq_ros_driver/WheelRPM.h"
#include "roboteq_ros_driver/config.h"

class RoboteqSerialDriver{
    private:
        serial::Serial ser;
        int baudrate;
        std::string port;

    public:
        ros::NodeHandle node;
        ros::Subscriber cmd_vel;
        // Custom rosmsg provide the speed of both the left and right wheel in [RPM]
        ros::Publisher wheel_vel;

        geometry_msgs::Twist wheel_twist;
        geometry_msgs::Twist cmd_twist;

        roboteq_ros_driver::WheelRPM robot_vel;
        roboteq_ros_driver::WheelRPM robot_cmd;

        // Channels
        const std::uint8_t left_wheel = 0;
        const std::uint8_t right_wheel = 1;



        bool init ();
        bool connect (std::string port, int baudrate);
        void run ();

        void velCallback(const geometry_msgs::Twist &msg);

        void initDriver ();
        void initMotor ();
        void initIO ();
        void initBLDC ();
        void maintenance ();

        // Runtime commands
        // Note: The param arg could be the wheel number or the IO bit to set or reset
        void setCommand (std::string cmd, std::uint8_t param,  std::int32_t value);
        void setCommand (std::string cmd, std::uint8_t param);
        void setCommand (std::string cmd);

        // Runtime queries
        std::int32_t getQuery (std::string query_type, std::uint8_t param,  std::int32_t value);
        std::int32_t getQuery (std::string query_type, std::uint8_t param);
        std::int32_t getQuery (std::string query_type);

        void getRPM(int right_wheel_rpm, int left_wheel_rpm);

        // Motor, IO & BLDC configurations
        // Write
        void setConfig (std::string config_type, std::uint8_t param, std::uint8_t action, std::uint8_t motor);
        void setConfig (std::string config_type, std::uint8_t param, std::int32_t value);
        void setConfig (std::string config_type, std::uint8_t param);
        void setConfig (std::string config_type);
        // Read
        std::uint32_t getConfig (std::string config_type, std::uint8_t param);
        std::uint32_t getConfig (std::string config_type);

        RoboteqSerialDriver (){
            init();
        }

        ~RoboteqSerialDriver(){
            if (ser.isOpen ())
            {
                ser.close ();
            }
        }
};
