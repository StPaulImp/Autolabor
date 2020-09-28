// Step 1:  Include Library Headers:
//发布imu的数据到IMU_data
#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h> 

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    nav_msgs::Odometry odom_data = *msg;
    // ROS_INFO("child_frame_id: [%s]", msg->data.child_frame_id.c_str());
    // ROS_INFO("child_frame_id: [%s]", msg->data.child_frame_id.c_str());
}

int main(int argc, char **argv)
{
    // Step 2: Initialization:
    ros::init(argc, argv, "imu_sim");     
    ros::NodeHandle nh;  
    ros::NodeHandle private_node("~");
    // ros::Subscriber odom_sub = nh.subscribe("odom", 100, odomCallback);  
    ros::Publisher IMU_pub = nh.advertise<sensor_msgs::Imu>("imu",100);
    ros::Rate loop_rate(50);

    while(ros::ok())
    {         
        sensor_msgs::Imu imu_data;
        imu_data.header.stamp = ros::Time::now();
        imu_data.header.frame_id = "base_link";
        //四元数位姿,所有数据设为固定值，可以自己写代码获取ＩＭＵ的数据，，然后进行传递
        imu_data.orientation.x = 0.0;
        imu_data.orientation.y = 0.0;
        imu_data.orientation.z = 0.0;
        imu_data.orientation.w = 1.0;
        //线加速度
        imu_data.linear_acceleration.x = 0.0; 
        imu_data.linear_acceleration.y = 0.0;
        imu_data.linear_acceleration.z = 0.01;
        //角速度
        imu_data.angular_velocity.x = 0; 
        imu_data.angular_velocity.y = 0; 
        imu_data.angular_velocity.z = 0.017;
        IMU_pub.publish(imu_data);
        ros::spinOnce();  
        loop_rate.sleep();  
    }
    return 0;
}