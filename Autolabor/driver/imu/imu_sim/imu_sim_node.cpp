// Step 1:  Include Library Headers:
// 发布imu的数据到IMU_data
#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h> 
// #include <geometry_msgs/TransformStamped.h>

nav_msgs::Odometry odom;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom = *msg;
    ROS_INFO("child_frame_id: [%s]", msg->child_frame_id.c_str());
    // ROS_INFO("child_frame_id: [%s]", msg->data.child_frame_id.c_str());
}

int main(int argc, char **argv)
{
    // Step 2: Initialization:
    ros::init(argc, argv, "imu_sim");
    ros::NodeHandle nh;  
    ros::NodeHandle private_node("~");
    ros::Subscriber odom_sub = nh.subscribe("odom", 100, odomCallback);  
    ros::Publisher IMU_pub = nh.advertise<sensor_msgs::Imu>("imu",100);
    tf::TransformBroadcaster imu_broadcaster;

    ros::Rate loop_rate(40);
    ros::Time current_time,last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    double vx = 0.0;
    double vy = 0.0;
    double vth = 0.0;

    while(ros::ok())
    {         
        sensor_msgs::Imu imu_data;
        current_time = ros::Time::now();

        /*** 
         * 速度位移公式
         * double delta_th = vth * dt;
         * th += delta_th;
         * double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
         * double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
         * x += delta_x;
         * y += delta_y;
        ***/

        //compute imu in a typical way given the accelerated velocities of the robot
        double dt = (current_time - last_time).toSec();
        double current_vx = odom.twist.twist.linear.x;
        // ROS_INFO("odom.twist.twist.linear.x:[%lf]", odom.twist.twist.linear.x);
        double current_vy = odom.twist.twist.linear.y;
        //set the position
        x = odom.pose.pose.position.x;
        y = odom.pose.pose.position.y;
        // odom.pose.pose.position.z = 0.0;
        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped imu_trans;
        imu_trans.header.stamp = current_time;
        imu_trans.header.frame_id = "imu_link";
        imu_trans.child_frame_id = "base_link";
        imu_trans.transform.translation.x = x;
        imu_trans.transform.translation.y = y;
        imu_trans.transform.translation.z = 0.0;
        imu_trans.transform.rotation = odom.pose.pose.orientation;
        //send the transform
        // imu_broadcaster.sendTransform(imu_trans);

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = odom.pose.pose.orientation;
        ROS_INFO("odom.pose.pose.orientation.w:[%lf]", odom.pose.pose.orientation.w);
        imu_data.header.stamp = ros::Time::now();
        imu_data.header.frame_id = "base_link";
        //四元数位姿,所有数据设为固定值，可以自己写代码获取ＩＭＵ的数据，，然后进行传递
        imu_data.orientation.x = odom_quat.x;
        // ROS_INFO("odom_quat.x:[%lf]", odom_quat.x);
        imu_data.orientation.y = odom_quat.y;
        // ROS_INFO("odom_quat.y:[%lf]", odom_quat.y);
        imu_data.orientation.z = odom_quat.z;
        // ROS_INFO("odom_quat.z:[%lf]", odom_quat.z);
        imu_data.orientation.w = odom_quat.w;
        ROS_INFO("odom_quat.w:[%lf]", odom_quat.w);
        //线加速度
        imu_data.linear_acceleration.x = 0.0;//(current_vx - vx) / dt; 
        ROS_INFO("linear_acceleration.x:[%lf]", imu_data.linear_acceleration.x);
        imu_data.linear_acceleration.y = 0.0;//(current_vy - vy) / dt; 
        ROS_INFO("linear_acceleration.y:[%lf]", imu_data.linear_acceleration.y);
        imu_data.linear_acceleration.z = 0.001;
        //角速度
        imu_data.angular_velocity.x = 0.0; 
        imu_data.angular_velocity.y = 0.0; 
        imu_data.angular_velocity.z = odom.twist.twist.angular.z;
        IMU_pub.publish(imu_data);

        last_time = current_time;
        vx = odom.twist.twist.linear.x;
        vy = odom.twist.twist.linear.y;
        vth = odom.twist.twist.angular.z;
        ros::spinOnce();  
        loop_rate.sleep();  
    }
    return 0;
}