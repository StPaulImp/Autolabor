#include <iostream>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/program_options.hpp>
#include <thread>
#include <boost/endian/arithmetic.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#include "net.h"
#include "vehicle.h"

using namespace boost;
using namespace std;
using namespace boost::program_options;

namespace vehicle
{
	class ros_warpper:public YHS_DGT001M
	{
    private:
        geometry_msgs::Twist current_twist_, current_twist_remote_;
        std_msgs::Bool key_flag_;
        boost::mutex twist_mutex_;
		boost::mutex twist_mutex_remote;
		boost::mutex flag_mutex_;
		ros::Time last_twist_time_;
    	nav_msgs::Odometry odom_;

		int control_rate_;
		int sensor_rate_;

		// tf2_ros::TransformBroadcaster br_;
		// geometry_msgs::TransformStamped transformStamped_;

		tf::TransformBroadcaster br_;
		geometry_msgs::TransformStamped transformStamped_; 

		ros::Time last_time_, now_;
		bool publish_tf_;
		std::string port_name_;
		int baud_rate_;
		std::string odom_frame_, base_frame_;

		bool start_flag_;
		double delta_time_;
		double accumulation_x_, accumulation_y_, accumulation_th_;
		int cur_left_, cur_right_, rev_left_, rev_right_, delta_left_, delta_right_;
		float linear_gain_, angular_gain_;

		ros::Subscriber cmd_sub;
		ros::Subscriber remote_cmd_sub;
		ros::Subscriber flag_sub;
		ros::Publisher odom_pub_,vel_feedback_pub_;

		virtual void doControl(yhs_wire_protocol::ControlData& cd) { 
			// std::cout << "doControl-roswarrper" << std::endl;
			static int _prekeyBoardControlcounter = 0; //预设按键板控制计数器
			_previousPtReady = _ptReady.load();  
			_prekeyBoardControlcounter = _keyBoardControlcounter; //定义键盘控制初始值为0
			uint8_t gear = 0x3;
			cd.Ctrl_cmd().Auto_Gear_Target(gear);
			float pvs = _vehicleSpeed *(-0.25);
            float ts = _targetSpeed;
            ts = std::clamp(ts, pvs - 0.05f, pvs + 0.05f);
            ts = std::clamp(ts, -0.2f, 0.2f);
			cd.targetSpeed(-1.0f * _targetSpeed);
			_previousVehicleSpeed = ts;
			// std::cout <<  "_targetAngleSpeed:" << _targetAngleSpeed << std::endl;
			float pas = _vehicleAngleSpeed * (0.5);
            float tas = _targetAngleSpeed;
			tas = std::clamp(tas, pas - 2.0f, pas + 2.0f);
            tas = std::clamp(tas, -30.0f, 30.0f);
			cd.targetAngleSpeed(_targetAngleSpeed);
			_previousAngleSpeed = tas;
    	}

        virtual void _txFunc(){
			std::cout << "_txFunc-roswarrper" << std::endl;
			int divider = 0;
			int counter = 0;
			yhs_wire_protocol::ControlData cd;
			while (ros::ok()) {
				// ros::spinOnce();
				doControl(cd);
				cd.rollingCounter(counter++);  
				counter &= 0xf;
				cd.calcCheckSum();
				can_frame frame;
				/*---------------------ctrl_cmd---------------------*/
				frame.can_id = CAN_EFF_FLAG | 0x18C4D1D0;
				frame.can_dlc = cd.Ctrl_cmd().data().size();
				// std::cout << "Ctrl_cmd.data().size():" << (int)frame.can_dlc << std::endl;
				std::copy(cd.Ctrl_cmd().data().begin(), cd.Ctrl_cmd().data().end(),
						frame.data);
				// std::cout << "Ctrl_cmd:";
				// for (int i = 0; i < sizeof(frame.data); i++){
				//    std::cout << std::hex << (int)frame.data[i] << " ";
				// }
				// std::cout << std::endl;
				_controlSocket.send(boost::asio::buffer(&frame, sizeof(frame)));
				/*---------------------io_cmd---------------------*/
				frame.can_id = CAN_EFF_FLAG | 0x18C4D7D0;
				frame.can_dlc = cd.Io_cmd().data().size();
				// std::cout << "Io_cmd.data().size():" << (int)frame.can_dlc << std::endl;
				std::copy(cd.Io_cmd().data().begin(), cd.Io_cmd().data().end(),
						frame.data);
				// std::cout << "Io_cmd:";
				// for (int i = 0; i < sizeof(frame.data); i++){
				//    std::cout << std::hex << (int)frame.data[i] << " ";
				// }
				// std::cout << std::endl;
				_controlSocket.send(boost::asio::buffer(&frame, sizeof(frame)));
				std::this_thread::sleep_for(100ms);
			}
		}

		virtual void _rxFunc(){
			std::cout << "_rxFunc-roswarrper" << std::endl;
			while (ros::ok()) {
				// ros::spinOnce();
				can_frame frame = {0};
				_controlSocket.receive(boost::asio::buffer(&frame, sizeof(frame)));
				if ((CAN_EFF_FLAG|0x18C4D1EF) ==  frame.can_id) {
					yhs_wire_protocol::ctrl_fb gearFeedBack(frame.data);
					_vcuCurGearPos = static_cast<CurrentGearPositon>(gearFeedBack.Auto_Gear_Statu());
					// std::cout << "_vcuCurGearPos 1:" << (int)gearFeedBack.Auto_Gear_Statu() << std::endl;
					yhs_wire_protocol::ctrl_fb driveFeedBack(frame.data);
					_vehicleSpeed = driveFeedBack.Auto_Drive_Speed() * (-0.001);
					if (fabs(_vehicleSpeed) < 0.01){
						_vehicleSpeed = 0.0f;
					}
					// std::cout << "_vehicleSpeed 1:" << _vehicleSpeed << std::endl;
					yhs_wire_protocol::ctrl_fb driveangleFeedBack(frame.data);
					_vehicleAngleSpeed = driveangleFeedBack.Auto_Drive_AngleSpeed() * (0.01) / 180 * M_PI;
					if (fabs(_vehicleAngleSpeed) < 0.017){
						_vehicleAngleSpeed = 0.0f;
					}
					// std::cout << "_vehicleAngleSpeed 1:" << _vehicleAngleSpeed << std::endl;
				}else if((CAN_EFF_FLAG|0x18C4D7EF) == frame.can_id) {
					yhs_wire_protocol::l_wheel_fb leftspeedFeedBack(frame.data);
					_curLeftSpeed = static_cast<float>(leftspeedFeedBack.Auto_WheelSpd_LeftBack());
					// std::cout << "_curLeftSpeed " << (int)leftspeedFeedBack.Auto_WheelSpd_LeftBack() << std::endl;
					yhs_wire_protocol::l_wheel_fb leftplusFeedBack(frame.data);
					_curLeftPlusSpeed = static_cast<float>(leftplusFeedBack.Auto_WheelPulse_LeftBack());
					// std::cout << "_curLeftPlusSpeed " << (int)leftplusFeedBack.Auto_WheelPulse_LeftBack() << std::endl;
				}else if((CAN_EFF_FLAG|0x18C4D8EF)  == frame.can_id) {    
					yhs_wire_protocol::r_wheel_fb rightspeedFeedBack(frame.data);
					_curRightSpeed = static_cast<float>(rightspeedFeedBack.Auto_WheelSpd_RightBack());
					// std::cout << "_curRightSpeed " << (int)rightspeedFeedBack.Auto_WheelSpd_RightBack() << std::endl;
					yhs_wire_protocol::r_wheel_fb rightplusFeedBack(frame.data);
					_curRightPlusSpeed = static_cast<float>(rightplusFeedBack.Auto_WheelPulse_RightBack());
					// std::cout << "_curRightPlusSpeed " << (int)rightplusFeedBack.Auto_WheelPulse_RightBack() << std::endl;
				}else if((CAN_EFF_FLAG|0x18C4DAEF ) == frame.can_id) {
					yhs_wire_protocol::io_fb SafeParkingFeedBack(frame.data);
					bool spb = SafeParkingFeedBack.SafeParking_Back();
					yhs_wire_protocol::io_fb EmergencyStopFeedBack(frame.data);
					bool esb = EmergencyStopFeedBack.EmergencyStop_Back();
					yhs_wire_protocol::io_fb RemoteControlFeedBack(frame.data);
					bool rcb = RemoteControlFeedBack.RemoteControl_Back();
				}
			}
		}

		void publish_odom_callback(const ros::TimerEvent &){
				now_ = ros::Time().now();
				delta_time_ = (now_ - last_time_).toSec();
				// std::cout << "delta_time_:" << delta_time_ << std::endl;

				// if (delta_time_ >= (1.0 / control_rate_)) {
					// // std::cout << "delta_time_:" << delta_time_ << std::endl;
					// double delta_theta = _vehicleAngleSpeed * delta_time_;
					// double v_theta = _vehicleAngleSpeed;
				
					// double delta_dis = _vehicleSpeed * delta_time_;
					// double v_dis = _vehicleSpeed;

					// geometry_msgs::Twist feedback_twist_;
					// feedback_twist_.linear.x = _vehicleSpeed;
					// feedback_twist_.angular.z = _vehicleAngleSpeed;
					// // std::cout << "feedback_twist_.linear.x:" << _vehicleSpeed << std::endl;
					// // std::cout << "feedback_twist_.angular.z:" << _vehicleAngleSpeed << std::endl;
					// vel_feedback_pub_.publish(feedback_twist_);

					// double delta_x, delta_y;
					// if (delta_theta == 0) {
					// 	delta_x = delta_dis;
					// 	delta_y = 0.0;
					// } else {
					// 	delta_x = delta_dis * (sin(delta_theta) / delta_theta);
					// 	delta_y = delta_dis * ((1 - cos(delta_theta)) / delta_theta);
					// }

					// accumulation_x_ += (cos(accumulation_th_) * delta_x - sin(accumulation_th_) * delta_y);
					// accumulation_y_ += (sin(accumulation_th_) * delta_x + cos(accumulation_th_) * delta_y);
					// accumulation_th_ += delta_theta;

					
					// accumulation_x_ += cos(accumulation_th_) * _vehicleSpeed * delta_time_;
					// accumulation_y_ += -sin(accumulation_th_) * _vehicleSpeed * delta_time_;
					// accumulation_th_ += _vehicleAngleSpeed * delta_time_;

					// self.dx = d / elapsed
                	// self.dr = th / elapsed

					double d = _vehicleSpeed * delta_time_;
					double th = _vehicleAngleSpeed * delta_time_;

					if (d != 0){
						double x = cos(th)*d;
						double y = -sin(th)*d;
						accumulation_x_ += (cos(accumulation_th_)*x - sin(accumulation_th_)*y);
						accumulation_y_ += (sin(accumulation_th_)*x + cos(accumulation_th_)*y);
					}
					if (th != 0){
						accumulation_th_ += th;
					}

					// tf2::Quaternion q;
					// q.setRPY(0, 0, accumulation_th_);
					tf::Quaternion q;
					q.setRPY(0, 0, accumulation_th_);
					// transformStamped_.setRotation(q);

					if (publish_tf_) {
						transformStamped_.header.stamp = ros::Time::now();
						transformStamped_.header.frame_id = odom_frame_;
						transformStamped_.child_frame_id = base_frame_;
						transformStamped_.transform.translation.x = accumulation_x_;
						transformStamped_.transform.translation.y = accumulation_y_;
						transformStamped_.transform.translation.z = 0.0;
						transformStamped_.transform.rotation.x = 0.0; //q.x();
						transformStamped_.transform.rotation.y = 0.0; //q.y();
						transformStamped_.transform.rotation.z = sin(accumulation_th_/2); //q.z();
						transformStamped_.transform.rotation.w = cos(accumulation_th_/2); //q.w();
						br_.sendTransform(transformStamped_);
					}
					odom_.header.frame_id = odom_frame_;
					odom_.child_frame_id = base_frame_;
					odom_.header.stamp = now_;
					odom_.pose.pose.position.x = accumulation_x_;
					odom_.pose.pose.position.y = accumulation_y_;
					odom_.pose.pose.position.z = 0;
					odom_.pose.pose.orientation.x = 0; //q.getX();
					odom_.pose.pose.orientation.y = 0; //q.getY();
					odom_.pose.pose.orientation.z = sin(accumulation_th_/2);//q.getZ();
					odom_.pose.pose.orientation.w = cos(accumulation_th_/2);//q.getW();
					odom_.twist.twist.linear.x = _vehicleSpeed; //v_dis;
					odom_.twist.twist.linear.y = 0;
					odom_.twist.twist.angular.z = _vehicleAngleSpeed; //v_theta;
             
					// odom_.twist.covariance[0] = 1e-3;
					// odom_.twist.covariance[7] = 1e-3; 
					// odom_.twist.covariance[8] = 1e-9;
					// odom_.twist.covariance[14] = 1e6;
					// odom_.twist.covariance[21] = 1e6;
					// odom_.twist.covariance[28] = 1e6;
					// odom_.twist.covariance[35] = 1e3;
					odom_pub_.publish(odom_);
					ROS_DEBUG_STREAM("accumulation_x: " << accumulation_x_ << "; accumulation_y: " << accumulation_y_ << "; accumulation_th: " << accumulation_th_);
					last_time_ = now_;
				// }
		}
	public:
		explicit ros_warpper(const std::string &ifname)
		:YHS_DGT001M(ifname),start_flag_(true){
			ros::NodeHandle node;
			ros::NodeHandle private_node("~");

			odom_pub_ = node.advertise<nav_msgs::Odometry>("/wheel_odom", 10, true);
			vel_feedback_pub_ = node.advertise<geometry_msgs::Twist>("/feedback_vel", 10, true);
			cmd_sub = node.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, boost::bind(&ros_warpper::twist_callback, this,_1));
			remote_cmd_sub = node.subscribe<geometry_msgs::Twist>("/cmd_vel_remote", 10, boost::bind(&ros_warpper::twist_callback_remote, this,_1));
			flag_sub = node.subscribe<std_msgs::Bool>("/send_flag", 10, boost::bind(&ros_warpper::flag_callback, this,_1));
			
			
			private_node.param<std::string>("port_name", port_name_, std::string("can0"));
			private_node.param<std::string>("odom_frame", odom_frame_, std::string("odom"));
			private_node.param<std::string>("base_frame", base_frame_, std::string("base_footprint"));

			private_node.param<int>("baud_rate", baud_rate_, 500000);
			private_node.param<int>("control_rate", control_rate_, 10);
			private_node.param<int>("sensor_rate", sensor_rate_, 10);
			private_node.param<bool>("publish_tf", publish_tf_, false);

			
			//多线程thread
			_rx = std::thread(&YHS_DGT001M::_rxFunc,this);
			_tx = std::thread(&YHS_DGT001M::_txFunc,this);
			start();
			ros::Timer odom_publish_timer = node.createTimer(ros::Duration(1.0 / sensor_rate_), &ros_warpper::publish_odom_callback, this);
		};

		void twist_callback (const geometry_msgs::Twist::ConstPtr &msg) { 
			twist_mutex_.lock();
			last_twist_time_ = ros::Time::now();
			current_twist_ = *msg.get();
			// current_twist_.linear.x *= 0.9;
			// current_twist_.angular.z *= 0.9;
            twist_mutex_.unlock();
		}

        void twist_callback_remote(const geometry_msgs::Twist::ConstPtr &msg) { 
            twist_mutex_remote.lock();
			last_twist_time_ = ros::Time::now();
			current_twist_remote_ = *msg.get();
            twist_mutex_remote.unlock();
	    }

        void flag_callback(const std_msgs::Bool::ConstPtr &msg) {
            flag_mutex_.lock();
			key_flag_ = *msg.get();
            flag_mutex_.unlock();
        }

		void keyFlag(std_msgs::Bool& value){key_flag_ = value;}
		inline bool keyFlag(){return key_flag_.data;}
		inline ros::Time lastTwistTime(){return last_twist_time_;}
		inline geometry_msgs::Twist currentTwist(){return current_twist_;}
		inline geometry_msgs::Twist currentTwistRemote(){return current_twist_remote_;}
    };
}

int main(int argc, char *argv[]) {
	options_description desc("options");
	variables_map vm;
	desc.add_options()
		("port", value<string>()->default_value("can0"), "can port")
		("steering", value<float>()->default_value(0.0f), "Steering angle")
		("speed", value<float>()->default_value(0.0f), "Speed")
		("aspeed",value<float>()->default_value(0.0f),"Angle Speed");
	try {
		store(parse_command_line(argc, argv, desc), vm);
		notify(vm);
	}catch (const error &ex) {
		std::cerr << ex.what() << '\n';
		return 1;
	}

	ros::init(argc, argv, "test_vehicle");
	ros::start();

	vehicle::ros_warpper yhs_dgt001m(std::string("can0"));
	// yhs_dgt001m.ros_warpper_init();

	while(ros::ok()) {
		ros::spinOnce();
		bool key_flag = yhs_dgt001m.keyFlag();
		double linear_speed, angular_speed;
		//		if ((ros::Time::now() - yhs_dgt001m.lastTwistTime()).toSec() <= 1.0) {
		//			linear_speed = yhs_dgt001m.currentTwistRemote().linear.x;
		//			angular_speed = yhs_dgt001m.currentTwistRemote().angular.z;
		//		} else {
		//			linear_speed = 0;
		//			angular_speed = 0;
		//		}
		// std::cout << "key_flag:" << key_flag << "linear:" << linear << "angular:" << angular << std::endl;

		if (key_flag){
			yhs_dgt001m.targetSpeed(yhs_dgt001m.currentTwistRemote().linear.x);
			yhs_dgt001m.targetAngleSpeed(yhs_dgt001m.currentTwistRemote().angular.z);
			// std::cout << "targetSpeed:" << yhs_dgt001m.targetSpeed() << std::endl;
			// std::cout << "targetAngleSpeed:" << yhs_dgt001m.targetAngleSpeed() << std::endl;
		}else{
			yhs_dgt001m.targetSpeed(yhs_dgt001m.currentTwist().linear.x);                  
			// yhs.targetSteeringAngle(vm["steering"].as<float>());    		   
			yhs_dgt001m.targetAngleSpeed(yhs_dgt001m.currentTwist().angular.z);            
		}
        std::this_thread::sleep_for(100ms);
	}
	// yhs_dgt001m.targetSpeed(0);
	// yhs_dgt001m.targetSteeringAngle(0);
	return 0;
}
 