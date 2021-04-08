#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

#include "vehicle.h"

// using namespace boost;
// using namespace std;
// using namespace boost::program_options;

namespace vehicle
{
	class ros_warpper:YHS_DGT001M
	{
    private:
        geometry_msgs::Twist current_twist_;
        std_msgs::Bool key_flag_;
        boost::mutex twist_mutex_;

	public:
		explicit ros_warpper(const std::string &ifname):YHS_DGT001M(ifname){};

        void twist_callback(const geometry_msgs::Twist::ConstPtr &msg) { 
            twist_mutex_.lock();
            // last_twist_time_ = ros::Time::now();
            current_twist_ = *msg.get();
            // _keyBoardControlcounter++;
            // _keyBoardControlcounter &= 0xf;
            // std::cout << "keyBoardControl on" << std::endl;
            twist_mutex_.unlock();
	    }

        void flag_callback(const std_msgs::Bool::ConstPtr &msg) {
            // twist_mutex_.lock();
            // last_twist_time_ = ros::Time::now();
            key_flag_ = *msg.get();
            // _keyBoardControlcounter++;
            // _keyBoardControlcounter &= 0xf;
            // std::cout << "keyBoardControl on" << std::endl;
            // twist_mutex_.unlock();
        }
    };
};