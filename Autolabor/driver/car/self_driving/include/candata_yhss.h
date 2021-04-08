#pragma once

#include "can.h"
#include <algorithm>
//计算网络//can2.0
namespace yhs {
    /*
    * Msg_ID 0x18C4D1EF 
    * Rate 10ms
    * Delay_Time none
    */
    class Auto_GearFeedBack : public can::CanData {   //档位反馈
    public:
        explicit Auto_GearFeedBack(unsigned char *data) : can::CanData(data, 8) {}  //explicit(显式)构造函数
    public:
        //当前线控档位系统状态信息
        //起始位 0 长度 1
        bool Auto_Gear_Active() {
            return get<bool>(0, 1, can::CanData::LITTLE_ENDIAN);
        }
        //当前档位状态
        //起始位 8 长度 3
        uint8_t Auto_Gear_Statu() {
            return get<uint8_t>(8, 3, can::CanData::LITTLE_ENDIAN);
        }
        //心跳信号(循环计数器)
        //起始位 52 长度 4
        uint8_t Auto_Gear_AliveCounter() {
            return get<uint8_t>(52, 4, can::CanData::LITTLE_ENDIAN);
        }
        //消息校验
        //起始位 56 长度 8
        uint8_t Auto_Gear_CheckSum() {
            return get<uint8_t>(56, 8, can::CanData::LITTLE_ENDIAN);
        }
    };
    /*
    * Msg_ID 0x18C4D2EF 
    * Rate 10ms
    * Delay_Time none
    */
    class Auto_SteeringFeedBack : public can::CanData {           //转向反馈
    public:
        explicit Auto_SteeringFeedBack(unsigned char *data) : can::CanData(data, 8) {}
    public:
        //当前线控转向系统状态信息
        //起始位 0 长度 1
        bool Auto_Steering_Active() {
            return get<bool>(0, 1, can::CanData::LITTLE_ENDIAN);
        }
        //当前方向盘转角(deg)
        //起始位 8 长度 12
        uint16_t Auto_Steering_Statu() {
            return get<uint16_t>(8, 12, can::CanData::LITTLE_ENDIAN);
        }
        //心跳信号(循环计数器)
        //起始位 52 长度 4
        uint8_t Auto_Steering_AliveCounter() {
            return get<uint8_t>(52, 4, can::CanData::LITTLE_ENDIAN);
        }
        //消息校验
        //起始位 56 长度 8
        uint8_t Auto_Steering_CheckSum() {
            return get<uint8_t>(56, 8, can::CanData::LITTLE_ENDIAN);
        }
    };
    /*
    * Msg_ID 0x18C4D3EF 
    * Rate 10ms
    * Delay_Time none
    * forward feedback
    */
    class Auto_DriveFeedBack : public can::CanData {
    public:
        explicit Auto_DriveFeedBack(unsigned char *data) : can::CanData(data, 8) {}   //驱动器反馈
    public:
        //当前线控驱动系统状态信息
        //起始位 0 长度 1
        bool Auto_Drive_Active() {
            return get<bool>(0, 1, can::CanData::LITTLE_ENDIAN);
        }
        //当前车速(m/s)
        //起始位 8 长度 16
        int16_t Auto_Drive_Speed() {
            return get<int16_t>(8, 16, can::CanData::LITTLE_ENDIAN);
        }
        //心跳信号(循环计数器)
        //起始位 52 长度 4
        int8_t Auto_Drive_AliveCounter() {
            return get<int8_t>(52, 4, can::CanData::LITTLE_ENDIAN);
        }
        //消息校验
        //起始位 56 长度 8
        int8_t Auto_Drive_CheckSum() {
            return get<int8_t>(56, 8, can::CanData::LITTLE_ENDIAN);
        }
    };
    /*
    * Msg_ID 0x18C4D4EF 
    * Rate 10ms
    * Delay_Time none
    */
    class Auto_BrakingFeedBack : public can::CanData {
    public:
        explicit Auto_BrakingFeedBack(unsigned char *data) : can::CanData(data, 8) {}  //制动反馈
    public:
        //当前制动转向系统状态信息
        //起始位 0 长度 1
        bool Auto_Braking_Active() {
            return get<bool>(0, 1, can::CanData::LITTLE_ENDIAN);
        }
        //当前制动状态
        //起始位 20 长度 2
        int8_t Auto_Braking_Staus() {
            return get<int8_t>(20, 2, can::CanData::LITTLE_ENDIAN);
        }
        //心跳信号(循环计数器)
        //起始位 52 长度 4
        int8_t Auto_Braking_AliveCounter() {
            return get<int8_t>(52, 4, can::CanData::LITTLE_ENDIAN);
        }
        //消息校验
        //起始位 56 长度 8
        int8_t Auto_Braking_CheckSum() {
            return get<int8_t>(56, 8, can::CanData::LITTLE_ENDIAN);
        }
    };

    /*
    * Msg_ID 0x18C4D5EF 
    * Rate 10ms
    * Delay_Time none
    */
    class Auto_PakingFeedBack : public can::CanData {
    public:
        explicit Auto_PakingFeedBack(unsigned char *data) : can::CanData(data, 8) {} //驻车反馈
    public:
        //当前电子驻车开关状态信息
        //起始位 0 长度 1
        bool Auto_Paking_Active() {
            return get<bool>(0, 1, can::CanData::LITTLE_ENDIAN);
        }
        //当前驻车状态
        //起始位 8 长度 8
        int8_t Auto_Paking_Statu() {
            return get<int8_t>(8, 8, can::CanData::LITTLE_ENDIAN);
        }
        //心跳信号(循环计数器)
        //起始位 52 长度 4
        int8_t Auto_Paking_AliveCounter() {
            return get<int8_t>(52, 4, can::CanData::LITTLE_ENDIAN);
        }
        //消息校验
        //起始位 56 长度 8
        int8_t Auto_Paking_CheckSum() {
            return get<int8_t>(56, 8, can::CanData::LITTLE_ENDIAN);
        }
    };
    
    /*
    * Msg_ID 0x18C4D7EF 
    * Rate 50ms
    * Delay_Time none
    */
    class Auto_WheelSpdFeedback : public can::CanData {
    public:
        explicit Auto_WheelSpdFeedback(unsigned char *data) : can::CanData(data, 8) {} //轮速反馈
    public:
        //轮速反馈信息有效位
        //起始位 0 长度 1
        bool Auto_WheelSpd_Active() {
            return get<bool>(0, 1, can::CanData::LITTLE_ENDIAN);
        }
        //左后轮轮速
        //起始位 8 长度 16
        int16_t Auto_WheelSpd_LeftBack() {
            return get<int16_t>(8, 16, can::CanData::LITTLE_ENDIAN);
        }
        //右后轮轮速
        //起始位 24 长度 16
        int16_t Auto_WheelSpd_RightBack() {
            return get<int16_t>(24, 16, can::CanData::LITTLE_ENDIAN);
        }
        //心跳信号(循环计数器)
        //起始位 52 长度 4
        int8_t Auto_WheelSpd_AliveCounter() {
            return get<int8_t>(52, 4, can::CanData::LITTLE_ENDIAN);
        }
        //消息校验
        //起始位 56 长度 8
        int8_t Auto_WheelSpd_CheckSum() {
            return get<int8_t>(56, 8, can::CanData::LITTLE_ENDIAN);
        }
    };
    /*
    * Msg_ID 0x18C4D8EF 
    * Rate 50ms
    * Delay_Time none
    */
    class Auto_WheelPulseFeedback : public can::CanData {
    public:
        explicit Auto_WheelPulseFeedback(unsigned char *data) : can::CanData(data, 8) {}   //车轮脉冲
    public:
        //脉冲数反馈信息有效位
        //起始位 0 长度 1
        bool Auto_WheelPulse_Active() {
            return get<bool>(0, 1, can::CanData::LITTLE_ENDIAN);
        }
        //左后轮脉冲数
        //起始位 8 长度 16
        int16_t Auto_WheelPulse_LeftBack() {
            return get<int16_t>(8, 16, can::CanData::LITTLE_ENDIAN);
        }
        //右后轮脉冲数
        //起始位 24 长度 16
        int16_t Auto_WheelPulse_RightBack() {
            return get<int16_t>(24, 16, can::CanData::LITTLE_ENDIAN);
        }
        //心跳信号(循环计数器)
        //起始位 52 长度 4
        int8_t Auto_WheelPulse_AliveCounter() {
            return get<int8_t>(52, 4, can::CanData::LITTLE_ENDIAN);
        }
        //消息校验
        //起始位 56 长度 8
        int8_t Auto_WheelPulse_CheckSum() {
            return get<int8_t>(56, 8, can::CanData::LITTLE_ENDIAN);
        }
    };
    /*
    * Msg_ID 0x18C4D9EF 
    * Rate 20ms
    * Delay_Time none
    */
    class Auto_MileageAndBodyFeedback : public can::CanData {
    public:
        explicit Auto_MileageAndBodyFeedback(unsigned char *data) : can::CanData(data, 8) {}  ////总里程及碰撞反馈
    public:
        //总里程及碰撞反馈信息有效位
        //起始位 0 长度 1
        bool Auto_MileageAndBody_Active() {
            return get<bool>(0, 1, can::CanData::LITTLE_ENDIAN);
        }
        //车辆累计里程
        //起始位 8 长度 24
        uint32_t Auto_Mileage() {
            return get<uint32_t>(8, 24, can::CanData::LITTLE_ENDIAN);
        }
        //车体前部碰撞
        //起始位 32 长度 1
        bool Auto_Collision_Front() {
            return get<bool>(32, 1, can::CanData::LITTLE_ENDIAN);
        }
        //车体后部碰撞
        //起始位 36 长度 1
        bool Auto_Collision_Back() {
            return get<bool>(36, 1, can::CanData::LITTLE_ENDIAN);
        }
        //车辆运行模式
        //起始位 40 长度 2
        uint8_t VCU_ModeControlFeedback() {          //模式状态 
            return get<uint8_t>(40, 2, can::CanData::LITTLE_ENDIAN);
        }
        //心跳信号(循环计数器)
        //起始位 52 长度 4
        uint8_t Auto_MileageAndBody_AliveCounter() {
            return get<uint8_t>(52, 4, can::CanData::LITTLE_ENDIAN);
        }
        //消息校验
        //起始位 56 长度 8
        uint8_t Auto_MileageAndBody_CheckSum() {
            return get<uint8_t>(56, 8, can::CanData::LITTLE_ENDIAN);
        }
    };
    /*
    * Msg_ID 0x18C4DAEF 
    * Rate 50ms
    * Delay_Time none
    */
    class Auto_LightFeedback : public can::CanData {
    public:
        explicit Auto_LightFeedback(unsigned char *data) : can::CanData(data, 8) {}  //前大灯使能反馈
    public:
        //前大灯使能反馈
        //起始位 0 长度 1
        bool Auto_FrontLight_EnableStatu() {
            return get<bool>(0, 1, can::CanData::LITTLE_ENDIAN);
        }
        //右转向灯使能反馈
        //起始位 8 长度 1
        bool Auto_RightLight_EnableStatu() {
            return get<bool>(8, 1, can::CanData::LITTLE_ENDIAN);
        }
        //左转向灯使能反馈
        //起始位 16 长度 1
        bool Auto_LeftLight_EnableStatu() {
            return get<bool>(16, 1, can::CanData::LITTLE_ENDIAN);
        }
        //心跳信号(循环计数器)
        //起始位 52 长度 4
        uint8_t Auto_Light_AliveCounter() {
            return get<uint8_t>(52, 4, can::CanData::LITTLE_ENDIAN);
        }
        //消息校验
        //起始位 56 长度 8
        uint8_t Auto_Light_CheckSum() {
            return get<uint8_t>(56, 8, can::CanData::LITTLE_ENDIAN);
        }
    };
    /*
    * Msg_ID 0x18C4DBEF 
    * Rate 50ms
    * Delay_Time none
    */
    class Auto_SpeakerFeedback : public can::CanData {
    public:
        explicit Auto_SpeakerFeedback(unsigned char *data) : can::CanData(data, 8) {}   //扬声器 1 使能反馈
    public:
        //扬声器 1 使能反馈
        //起始位 0 长度 1
        bool Auto_Speaker_Enable() {
            return get<bool>(0, 1, can::CanData::LITTLE_ENDIAN);
        }
        //心跳信号(循环计数器)
        //起始位 52 长度 4
        int8_t Auto_Speaker_AliveCounter() {
            return get<int8_t>(52, 4, can::CanData::LITTLE_ENDIAN);
        }
        //消息校验
        //起始位 56 长度 8
        int8_t Auto_Speaker_CheckSum() {
            return get<int8_t>(56, 8, can::CanData::LITTLE_ENDIAN);
        }
    };
    /*
    * Msg_ID 0x18C4D1D0  
    * Rate 10ms
    * Delay_Time none
    */
    class Auto_GearCmd : public can::CanData { //档位线控功能使能标志位//向下发送一个档位，上面顶上那个是反馈
    public:
        Auto_GearCmd() : can::CanData(8) {}
    public:
        //档位线控功能使能标志位
        //起始位 0 长度 1
        void Auto_Gear_Enable(bool value) {
            set<bool>(0, 1, can::CanData::LITTLE_ENDIAN, value);
        }
        //目标档位
        //起始位 8 长度 3
        void Auto_Gear_Target(uint8_t value) {
            set<uint8_t>(8, 3, can::CanData::LITTLE_ENDIAN, value);
        }
        //心跳信号(循环计数器)
        //起始位 52 长度 4
        void Auto_Gear_AliveCounter(int8_t value) {
            set<int8_t>(52, 4, can::CanData::LITTLE_ENDIAN,value);
        }
        //消息校验
        //起始位 56 长度 8
        void Auto_Gear_CheckSum(int8_t value) {
            set<int8_t>(56, 8, can::CanData::LITTLE_ENDIAN,value);
        }
    };
    /*
    * Msg_ID 0x18C4D2D0 
    * Rate 10ms
    * Delay_Time none
    */
    class Auto_SteeringCmd : public can::CanData {  //转向线控功能使能标志位，向下发送的转速
    public:
        Auto_SteeringCmd() : can::CanData(8) {}
    public:
        //转向线控功能使能标志位
        //起始位 0 长度 1
        void Auto_Steering_Enable(bool value) {
            set<bool>(0, 1, can::CanData::LITTLE_ENDIAN, value);
        }
        //目标转向角度
        //起始位 8 长度 12
        void Auto_Steering_Target(int16_t value) {
            set<int16_t>(8, 12, can::CanData::LITTLE_ENDIAN, value);
        }
        //心跳信号(循环计数器)
        //起始位 52 长度 4
        void Auto_Steering_AliveCounter(int8_t value) {
            set<int8_t>(52, 4, can::CanData::LITTLE_ENDIAN,value);
        }
        //消息校验
        //起始位 56 长度 8
        void Auto_Steering_CheckSum(int8_t value) {
            set<int8_t>(56, 8, can::CanData::LITTLE_ENDIAN,value);
        }
    };
    /*
    * Msg_ID 0x18C4D3D0 
    * Rate 10ms
    * Delay_Time none
    * forward control
    */
    class Auto_DriveCmd : public can::CanData {  //驱动线控功能使能标志位
    public:
        Auto_DriveCmd() : can::CanData(8) {}
    public:
        //驱动线控功能使能标志位
        //起始位 0 长度 1
        void Auto_Drive_Enable(bool value) {
            set<bool>(0, 1, can::CanData::LITTLE_ENDIAN, value);
        }
        //目标速度
        //起始位 8 长度 16
        void Auto_Drive_Target(int16_t value) {
            set<int16_t>(8, 16, can::CanData::LITTLE_ENDIAN, value);
        }
        //心跳信号(循环计数器)
        //起始位 52 长度 4
        void Auto_Drive_AliveCounter(int8_t value) {
            set<int8_t>(52, 4, can::CanData::LITTLE_ENDIAN,value);
        }
        //消息校验
        //起始位 56 长度 8
        void Auto_Drive_CheckSum(int8_t value) {
            set<int8_t>(56, 8, can::CanData::LITTLE_ENDIAN,value);
        }
    };
    /*
    * Msg_ID 0x18C4D4D0 
    * Rate 10ms
    * Delay_Time none
    */
    class Auto_BrakingCmd : public can::CanData {//制动线控功能使能标志位
    public:
        Auto_BrakingCmd() : can::CanData(8) {}
    public:
        //制动线控功能使能标志位
        //起始位 0 长度 1
        void Auto_Braking_Enable(bool value) {
            set<bool>(0, 1, can::CanData::LITTLE_ENDIAN, value);
        }
        //车辆制动踏板开度
        //起始位 8 长度 8
        void Auto_Braking_Target(int8_t value) {
            set<int8_t>(8, 8, can::CanData::LITTLE_ENDIAN, value);
        }
        //心跳信号(循环计数器)
        //起始位 52 长度 4
        void Auto_Braking_AliveCounter(int8_t value) {
            set<int8_t>(52, 4, can::CanData::LITTLE_ENDIAN,value);
        }
        //消息校验
        //起始位 56 长度 8
        void Auto_Braking_CheckSum(int8_t value) {
            set<int8_t>(56, 8, can::CanData::LITTLE_ENDIAN,value);
        }
    };
    /*
    * Msg_ID 0x18C4D5D0 
    * Rate 10ms
    * Delay_Time none
    */
    class Auto_ParkingCmd : public can::CanData { //驻车线控功能使能标志位 //手刹，自动挡手刹
    public:  
        Auto_ParkingCmd() : can::CanData(8) {}
    public:
        //驻车线控功能使能标志位
        //起始位 0 长度 1
        void Auto_Parking_Enable(bool value) {
            set<bool>(0, 1, can::CanData::LITTLE_ENDIAN, value);
        }
        //驻车请求
        //起始位 8 长度 1
        void Auto_Parking_Apply(bool value) {
            set<bool>(8, 1, can::CanData::LITTLE_ENDIAN, value);
        }
        //心跳信号(循环计数器)
        //起始位 52 长度 4
        void Auto_Parking_AliveCounter(int8_t value) {
            set<int8_t>(52, 4, can::CanData::LITTLE_ENDIAN,value);
        }
        //消息校验
        //起始位 56 长度 8
        void Auto_Parking_CheckSum(int8_t value) {
            set<int8_t>(56, 8, can::CanData::LITTLE_ENDIAN,value);
        }
    };
    /*
    * Msg_ID 0x18C4D6D0 
    * Rate 10ms
    * Delay_Time none
    */
    class Auto_OdometerCmd : public can::CanData {//里程计控制指令使能
    public:
        Auto_OdometerCmd() : can::CanData(8) {}
    public:
        //里程计控制指令使能
        //起始位 0 长度 1
        void Auto_Odometer_Enable(bool value) {
            set<bool>(0, 1, can::CanData::LITTLE_ENDIAN, value);
        }
        //车辆总里程清零指令
        //起始位 16 长度 1
        void Auto_Odometer_Clear(bool value) {
            set<bool>(16, 1, can::CanData::LITTLE_ENDIAN, value);
        }
        //心跳信号(循环计数器)
        //起始位 52 长度 4
        void Auto_Odometer_AliveCounter(int8_t value) {
            set<int8_t>(52, 4, can::CanData::LITTLE_ENDIAN,value);
        }
        //消息校验
        //起始位 56 长度 8
        void Auto_Odometer_CheckSum(int8_t value) {
            set<int8_t>(56, 8, can::CanData::LITTLE_ENDIAN,value);
        }
    };
    /*
    * Msg_ID 0x18C4D7D0 
    * Rate 10ms
    * Delay_Time none
    */
    class Auto_LightCmd : public can::CanData {   //车灯
    public:
        Auto_LightCmd() : can::CanData(8) {}
    public:
        //前大灯使能控制
        //起始位 0 长度 1
        void Auto_FrontLight_Enable(bool value) {
            set<bool>(0, 1, can::CanData::LITTLE_ENDIAN, value);
        }
        //右转向灯使能控制
        //起始位 8 长度 1
        void Auto_RightLight_Enable(bool value) {
            set<bool>(8, 1, can::CanData::LITTLE_ENDIAN, value);
        }
        //左转向灯使能控制
        //起始位 16 长度 1
        void Auto_LeftLight_Enable(bool value) {
            set<bool>(16, 1, can::CanData::LITTLE_ENDIAN, value);
        }
        //心跳信号(循环计数器)
        //起始位 52 长度 4
        void Auto_Light_AliveCounter(int8_t value) {
            set<int8_t>(52, 4, can::CanData::LITTLE_ENDIAN,value);
        }
        //消息校验
        //起始位 56 长度 8
        void Auto_Light_CheckSum(int8_t value) {
            set<int8_t>(56, 8, can::CanData::LITTLE_ENDIAN,value);
        }
    };
    /*
    * Msg_ID 0x18C4D8D0 
    * Rate 10ms
    * Delay_Time none
    */
    class Auto_SpeakerCmd : public can::CanData {  //喇叭
    public:
        Auto_SpeakerCmd() : can::CanData(8) {}
    public:
        //扬声器 1 使能控制
        //起始位 0 长度 1
        void Auto_Speaker_Enable(bool value) {
            set<bool>(0, 1, can::CanData::LITTLE_ENDIAN, value);
        }
        //心跳信号(循环计数器)
        //起始位 52 长度 4
        void Auto_Light_AliveCounter(int8_t value) {
            set<int8_t>(52, 4, can::CanData::LITTLE_ENDIAN,value);
        }
        //消息校验
        //起始位 56 长度 8
        void Auto_Light_CheckSum(int8_t value) {
            set<int8_t>(56, 8, can::CanData::LITTLE_ENDIAN,value);
        }
    };

    class ControlData {   //定义控制数据函数//控制命令封装集
    private:
        Auto_GearCmd _auto_GearCmd;
        Auto_SteeringCmd _auto_SteeringCmd;
        Auto_DriveCmd _auto_DriveCmd;
        Auto_BrakingCmd _auto_BrakingCmd;
        Auto_ParkingCmd _auto_ParkingCmd;
        Auto_LightCmd _auto_LightCmd;
        uint8_t crc(std::vector<unsigned char>::const_iterator begin, std::vector<unsigned char>::const_iterator end) {   //遍历容器内的元素，并访问这些元素的值。iterator可以改元素值,但const_iterator不可改
            uint8_t ret = 0xff;
            for (auto it = begin; it != end; it++) {
                ret ^= *it;
            }
            return ret ^ 0xff;
        }
    public:
        Auto_GearCmd &auto_GearCmd() {
            return _auto_GearCmd;
        }
        Auto_SteeringCmd &auto_SteeringCmd() {
            return _auto_SteeringCmd;
        }
        Auto_DriveCmd &auto_DriveCmd() {
            return _auto_DriveCmd;
        }
        Auto_BrakingCmd &auto_BrakingCmd() {
            return _auto_BrakingCmd;
        }
        Auto_ParkingCmd &auto_ParkingCmd() {
            return _auto_ParkingCmd;
        }
        Auto_LightCmd &auto_LightCmd() {
            return _auto_LightCmd;
        }
    public:
        void targetSteeringAngle(float degree) {//目标转向角函数
            degree = degree < -24 ? -24 : degree; //如果大于-24，则为-24，否则为当前给定角度
            degree = degree > 24 ? 24 : degree;
            degree -= 90; //偏移量
            degree /= 0.043945; //精确度，单位度 //出现在目标转向角度的说明里0x18C4D2D0
            _auto_SteeringCmd.Auto_Steering_Target(static_cast<int16_t>(degree));   //强制类型转换
        }

        void targetSpeed(float targetSpeed) {  //目标速度
            targetSpeed *= 1000;  //分辨率 1毫米每妙
            _auto_DriveCmd.Auto_Drive_Target(static_cast<int16_t>(targetSpeed));
        }

        void targetBraking(float targetBrakingPedalAperture) {  //目标制动  目标刹车踏板，刹车开度
            targetBrakingPedalAperture /= 0.390625; //分辨率
            _auto_BrakingCmd.Auto_Braking_Target(static_cast<int8_t>(targetBrakingPedalAperture));
        }

        // void turnSignal(uint8_t turnSignal){
        //     switch(turnSignal){
        //         case 0:
        //             _auto_LightCmd.Auto_FrontLight_Enable(false);
        //         case 1:
        //         case 2:
        //         case 3:
        //     }
        // }

        void rollingCounter(uint8_t rc) { //循环计数器//一个控制过程每个周期参与的，10ms一个周期
            _auto_GearCmd.Auto_Gear_AliveCounter(rc);
            _auto_SteeringCmd.Auto_Steering_AliveCounter(rc);
            _auto_DriveCmd.Auto_Drive_AliveCounter(rc);
            _auto_BrakingCmd.Auto_Braking_AliveCounter(rc);
            _auto_ParkingCmd.Auto_Parking_AliveCounter(rc);
            _auto_LightCmd.Auto_Light_AliveCounter(rc);
        }

        void calcCheckSum() {  //消息校验
            _auto_GearCmd.Auto_Gear_CheckSum(crc(_auto_GearCmd.data().cbegin(), _auto_GearCmd.data().cend()-1));
            _auto_SteeringCmd.Auto_Steering_CheckSum(crc(_auto_SteeringCmd.data().cbegin(), _auto_SteeringCmd.data().cend()-1));
            _auto_DriveCmd.Auto_Drive_CheckSum(crc(_auto_DriveCmd.data().cbegin(), _auto_DriveCmd.data().cend()-1));
            _auto_BrakingCmd.Auto_Braking_CheckSum(crc(_auto_BrakingCmd.data().cbegin(), _auto_BrakingCmd.data().cend()-1));
            _auto_ParkingCmd.Auto_Parking_CheckSum(crc(_auto_ParkingCmd.data().cbegin(), _auto_ParkingCmd.data().cend()-1));
            _auto_LightCmd.Auto_Light_CheckSum(crc(_auto_LightCmd.data().cbegin(), _auto_LightCmd.data().cend()-1));
        }
    };
}
