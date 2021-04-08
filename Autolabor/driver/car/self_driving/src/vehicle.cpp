#include <numeric>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/format.hpp>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <pthread.h>
#include <stdexcept>

#include "../include/vehicle.h"

/*----------车体命名空间------------*/
namespace vehicle {
    using namespace boost;
    // using namespace std;
    const float PI = 3.14159265358979323846f;

    //设置串口参数
	void setupSerialPort(boost::asio::serial_port & port) {
		port.set_option(asio::serial_port_base::baud_rate(500000));
		port.set_option(asio::serial_port_base::character_size(8));//字符大小
		port.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one)); //停止位
		port.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));  //parity奇偶校验
		port.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));//流量控制
	}
    //发送串口/命令/错误代码 -----当发现error，return 0
	Status send(boost::asio::serial_port & port, const Command & command, boost::system::error_code & ec) {  
		Status ret = { 0 };
		asio::write(port, asio::buffer(&command, sizeof(command)), ec);//本身并不申请内存，只是提供了一个对现有内存的封装，并可以构造拷贝
		if (ec)
			return ret;
		asio::read(port, asio::buffer(&ret, sizeof(ret)), ec);
		return ret;
	}
    //post ？？
	void post(boost::asio::serial_port & port, const Command & command, boost::system::error_code & ec) { 
		asio::write(port, asio::buffer(&command, sizeof(command)), ec);
	}
    //post_serial 序列号？
    void post_serial(boost::asio::serial_port & port, const std::vector<char>& command, boost::system::error_code & ec) { 
        asio::write(port, asio::buffer(&command, sizeof(command)), ec);
    }

    /*----------车体直线或转弯到达目标点模型------------*/  //模型需要改

    //与前方直线距离          
    //optional容纳一个对象值或是为空
    boost::optional<float> Geometry::straightFrontDistance(const Eigen::Vector2f & point) const {  
        if(std::abs(point.x()) > vehicleWidth()/2 + safeDistance())
            return boost::none;  //如果if语句成立则执行该行
        return point.y(); 
    }

	boost::optional<float> Geometry::turningFrontDistance(const Eigen::Vector2f & point, float steeringAngle) const { //转弯前距
        /*
        float r = radiusOfTurning(steeringAngle);
        float a = wheelBase() + frontOverhang(); //轮距+前悬距离
        float bi = r - (vehicleWidth() / 2 + safeDistance());
        float bo = r + (vehicleWidth() / 2 + safeDistance());  
        float innerR = std::sqrt(a*a + bi * bi);  //内半径
        float outerR = std::sqrt(a*a + bo * bo);  //外半径
        Eigen::Vector2f c = centerOfTurning(r, steeringAngle); //转向中心（转向半径，转向角度）
        Eigen::Vector2f pointInC = point - c;
        float n = pointInC.norm();

        if (n < innerR || n > outerR)
            return boost::none;
        if(c.x() < 0)
            return std::atan2(pointInC.y() , pointInC.x()) * n;  //atan2是一个函数，在C语言里返回的是指方位角，C 语言中atan2的函数原型为 double atan2(double y, double x) ，返回以弧度表示的 y/x 的反正切
        else
            return std::atan2(pointInC.y() , -pointInC.x()) * n;
        */
	}

    
	boost::optional<float> Geometry::frontDistance(const Eigen::Vector2f & point, float steeringAngle) const {
		if (std::abs(steeringAngle) < 1)
			return straightFrontDistance(point);
		else
			return turningFrontDistance(point, steeringAngle);
	}

    /*-----------------------YHS--------------------------*/
    YHS::YHS(const std::string &ifname): _controlSocket(_io){}
    void YHS::predoControl(yhs_wire_protocol::ControlData& cd){}
    void YHS::doControl(yhs_wire_protocol::ControlData& cd) {}
	void YHS::_rxFunc() {}
    void YHS::_txFunc() {}

    /*-----------------------YHS_DGT001M--------------------------*/
    YHS_DGT001M::YHS_DGT001M(const std::string &ifname)
            : _controlSocket(_io){
        _controlSocket.open();
        _controlSocket.bind(net::CanRaw::endpoint(ifname));

		// //多线程thread
        // _rx = std::thread(&YHS_DGT001M::_rxFunc, this);
        // _tx = std::thread(&YHS_DGT001M::_txFunc, this);
    }

    //预先控制
    void YHS_DGT001M::predoControl(yhs_wire_protocol::ControlData& cd){  
        return;
	}

    //执行控制	
    void YHS_DGT001M::doControl(yhs_wire_protocol::ControlData& cd) { 
        std::cout << "doControl" << std::endl;
        // static int _prekeyBoardControlcounter = 0; //预设按键板控制计数器
        // _previousPtReady = _ptReady.load();  
        // _prekeyBoardControlcounter = _keyBoardControlcounter; //定义键盘控制初始值为0
        // uint8_t gear = 0x3;
        // cd.Ctrl_cmd().Auto_Gear_Target(gear);
        // cd.targetSpeed(_targetSpeed);
        // // std::cout <<  "_targetAngleSpeed:" << _targetAngleSpeed << std::endl;
        // cd.targetAngleSpeed(_targetAngleSpeed);
    }

    /*-----------------------YHS，rxFunc--------------------------*/  
	void YHS_DGT001M::_rxFunc() {
        std::cout << "_rxFunc" << std::endl;
		// for (;;) {
        //     try{
        //         can_frame frame = {0};
        //         _controlSocket.receive(boost::asio::buffer(&frame, sizeof(frame)));
        //         if ((CAN_EFF_FLAG|0x18C4D1EF) ==  frame.can_id) {
        //             yhs_wire_protocol::ctrl_fb gearFeedBack(frame.data);
        //             _vcuCurGearPos = static_cast<CurrentGearPositon>(gearFeedBack.Auto_Gear_Statu());
        //             // std::cout << "_vcuCurGearPos" << (int)gearFeedBack.Auto_Gear_Statu() << std::endl;
        //             yhs_wire_protocol::ctrl_fb driveFeedBack(frame.data);
        //             _vehicleSpeed = driveFeedBack.Auto_Drive_Speed() * 0.001;
        //             // std::cout << "_vehicleSpeed" << _vehicleSpeed << std::endl;
        //             yhs_wire_protocol::ctrl_fb driveangleFeedBack(frame.data);
        //             // _vehicleAngleSpeed = driveangleFeedBack.Auto_Drive_AngleSpeed() * 0.01;
        //             //std::cout << "_vehicleAngleSpeed" << _vehicleAngleSpeed << std::endl;
        //         }else if((CAN_EFF_FLAG|0x18C4D7EF) == frame.can_id) {
        //             yhs_wire_protocol::l_wheel_fb leftspeedFeedBack(frame.data);
        //             _curLeftSpeed = static_cast<float>(leftspeedFeedBack.Auto_WheelSpd_LeftBack());
        //             // std::cout << "_curLeftSpeed " << (int)leftspeedFeedBack.Auto_WheelSpd_LeftBack() << std::endl;
        //             yhs_wire_protocol::l_wheel_fb leftplusFeedBack(frame.data);
        //             _curLeftPlusSpeed = static_cast<float>(leftplusFeedBack.Auto_WheelPulse_LeftBack());
        //             // std::cout << "_curLeftPlusSpeed " << (int)leftplusFeedBack.Auto_WheelPulse_LeftBack() << std::endl;
        //         }else if((CAN_EFF_FLAG|0x18C4D8EF)  == frame.can_id) {    
        //             yhs_wire_protocol::r_wheel_fb rightspeedFeedBack(frame.data);
        //             _curRightSpeed = static_cast<float>(rightspeedFeedBack.Auto_WheelSpd_RightBack());
        //             // std::cout << "_curRightSpeed " << (int)rightspeedFeedBack.Auto_WheelSpd_RightBack() << std::endl;
        //             yhs_wire_protocol::r_wheel_fb rightplusFeedBack(frame.data);
        //             _curRightPlusSpeed = static_cast<float>(rightplusFeedBack.Auto_WheelPulse_RightBack());
        //             // std::cout << "_curRightPlusSpeed " << (int)rightplusFeedBack.Auto_WheelPulse_RightBack() << std::endl;
        //         }else if((CAN_EFF_FLAG|0x18C4DAEF ) == frame.can_id) {
        //             yhs_wire_protocol::io_fb SafeParkingFeedBack(frame.data);
        //             bool spb = SafeParkingFeedBack.SafeParking_Back();
        //             yhs_wire_protocol::io_fb EmergencyStopFeedBack(frame.data);
        //             bool esb = EmergencyStopFeedBack.EmergencyStop_Back();
        //             yhs_wire_protocol::io_fb RemoteControlFeedBack(frame.data);
        //             bool rcb = RemoteControlFeedBack.RemoteControl_Back();
        //         }
        //     }catch(std::exception& err){
        //         std::cerr << err.what() << std::endl;
        //     }

        // }
    }

    /*-----------------------YHS，txFunc--------------------------*/  
    //功能函数，各类功能具体执行开始/结束标志
    void YHS_DGT001M::_txFunc() { 
        std::cout << "_txFunc" << std::endl;           
	    // int divider = 0;
	    // int counter = 0;
        // yhs_wire_protocol::ControlData cd;
        // for (;;) {
        //     try{
        //         doControl(cd);
        //         cd.rollingCounter(counter++);  
        //         counter &= 0xf;
        //         cd.calcCheckSum();
        //         can_frame frame;
        //         /*---------------------ctrl_cmd---------------------*/
        //         frame.can_id = CAN_EFF_FLAG | 0x18C4D1D0;
        //         frame.can_dlc = cd.Ctrl_cmd().data().size();
        //         // std::cout << "Ctrl_cmd.data().size():" << (int)frame.can_dlc << std::endl;
        //         std::copy(cd.Ctrl_cmd().data().begin(), cd.Ctrl_cmd().data().end(),
        //                 frame.data);
        //         // std::cout << "Ctrl_cmd:";
        //         // for (int i = 0; i < sizeof(frame.data); i++){
        //         //    std::cout << std::hex << (int)frame.data[i] << " ";
        //         // }
        //         // std::cout << std::endl;
        //         _controlSocket.send(boost::asio::buffer(&frame, sizeof(frame)));
        //         /*---------------------io_cmd---------------------*/
        //         frame.can_id = CAN_EFF_FLAG | 0x18C4D7D0;
        //         frame.can_dlc = cd.Io_cmd().data().size();
        //         // std::cout << "Io_cmd.data().size():" << (int)frame.can_dlc << std::endl;
        //         std::copy(cd.Io_cmd().data().begin(), cd.Io_cmd().data().end(),
        //                 frame.data);
        //         // std::cout << "Io_cmd:";
        //         // for (int i = 0; i < sizeof(frame.data); i++){
        //         //    std::cout << std::hex << (int)frame.data[i] << " ";
        //         // }
        //         // std::cout << std::endl;
        //         _controlSocket.send(boost::asio::buffer(&frame, sizeof(frame)));
        //         std::this_thread::sleep_for(10ms);
        //     }catch(std::exception &err){
        //         std::cerr << err.what() << std::endl;
        //     }
        // }
	}
};