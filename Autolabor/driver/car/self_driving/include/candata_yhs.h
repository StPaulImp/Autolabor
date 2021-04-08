#pragma once

#include "can.h"
#include <algorithm>
//计算网络//can2.0
namespace yhs_wire_protocol {
    /*
    * 运动学指令控制帧，注意和自由控制指令控制帧，只能2选1
    * Msg_ID 0x18C4D1D0  
    * Rate 10ms
    * Delay_Time none
    */
    class ctrl_cmd : public can::CanData { 
    public:
        ctrl_cmd() : can::CanData(8) {}
    public:
        //目标档位
        //起始位 0 长度 4
        void Auto_Gear_Target(uint8_t value) {
            set<uint8_t>(0, 4, can::CanData::LITTLE_ENDIAN, value);
        }
        //目标速度
        //起始位 4 长度 16
        void Auto_Drive_Target(int16_t value) {
            set<int16_t>(4, 16, can::CanData::LITTLE_ENDIAN, value);
        }
        //目标角速度
        //起始位 20 长度 16
        void Auto_Drive_AngleTarget(int16_t value) {
            set<int16_t>(20, 16, can::CanData::LITTLE_ENDIAN, value);
        }
        //心跳信号(循环计数器)
        //起始位 52 长度 4
        void Auto_Gear_AliveCounter(uint8_t value) {
            set<uint8_t>(52, 4, can::CanData::LITTLE_ENDIAN,value);
        }
        //消息校验
        //起始位 56 长度 8
        void Auto_Gear_CheckSum(uint8_t value) {
            set<uint8_t>(56, 8, can::CanData::LITTLE_ENDIAN,value);
        }
    };
    /*
    * 自由控制指令控制帧，注意和运动学指令控制帧，只能2选1
    * Msg_ID 0x18C4D2D0  
    * Rate 10ms
    * Delay_Time none
    */
    class free_ctrl_cmd : public can::CanData { 
    public:
        free_ctrl_cmd() : can::CanData(8) {}
    public:
        //目标档位
        //起始位 0 长度 4
        void Auto_free_Gear_Target(uint8_t value) {
            set<uint8_t>(0, 4, can::CanData::LITTLE_ENDIAN, value);
        }
        //左轮目标速度
        //起始位 4 长度 16
        void Auto_free_Left_Drive_Target(int16_t value) {
            set<int16_t>(4, 16, can::CanData::LITTLE_ENDIAN, value);
        }
        //右轮目标速度
        //起始位 20 长度 16
        void Auto_free_Right_Drive_Target(int16_t value) {
            set<int16_t>(20, 16, can::CanData::LITTLE_ENDIAN, value);
        }
        //心跳信号(循环计数器)
        //起始位 52 长度 4
        void Auto_free_Gear_AliveCounter(int8_t value) {
            set<int8_t>(52, 4, can::CanData::LITTLE_ENDIAN,value);
        }
        //消息校验
        //起始位 56 长度 8
        void Auto_free_Gear_CheckSum(int8_t value) {
            set<int8_t>(56, 8, can::CanData::LITTLE_ENDIAN,value);
        }
    };
    /*
    * 其他指令控制帧
    * Msg_ID 0x18C4D7D0
    * 报文发送类型为 IfActive
    * Rate 0ms
    */
    class io_cmd : public can::CanData { 
    public:
        io_cmd() : can::CanData(8) {}
    public:
        //安全停车解锁开关
        //起始位 1 长度 1
        void Safe_Parking(bool value) {
            set<bool>(1, 1, can::CanData::LITTLE_ENDIAN,value);
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
    * 档位/车体线速度/车体角速度反馈
    * Msg_ID 0x18C4D1EF
    * Rate 10ms
    */ 
    class ctrl_fb : public can::CanData {
    public:
        explicit ctrl_fb(unsigned char *data) : can::CanData(data, 8) {}  
    public:
        //当前档位状态
        //起始位 0 长度 4
        uint8_t Auto_Gear_Statu() {
            return get<uint8_t>(0, 4, can::CanData::LITTLE_ENDIAN);
        }
        //当前车速(m/s)
        //起始位 4 长度 16
        int16_t Auto_Drive_Speed() {
            return get<int16_t>(4, 16, can::CanData::LITTLE_ENDIAN);
        }
        //当前角速度(度/秒)
        //起始位 4 长度 16
        int16_t Auto_Drive_AngleSpeed() {
            return get<int16_t>(20, 16, can::CanData::LITTLE_ENDIAN);
        }
        //心跳信号(循环计数器)
        //起始位 52 长度 4
        uint8_t Auto_Drive_AliveCounter() {
            return get<uint8_t>(52, 4, can::CanData::LITTLE_ENDIAN);
        }
        //消息校验
        //起始位 56 长度 8
        uint8_t Auto_Drive_CheckSum() {
            return get<uint8_t>(56, 8, can::CanData::LITTLE_ENDIAN);
        }
    };
    /*
    * Msg_ID 0x18C4D7EF 
    * Rate 10ms
    * Delay_Time none
    */
    class l_wheel_fb : public can::CanData {
    public:
        explicit l_wheel_fb(unsigned char *data) : can::CanData(data, 8) {} //轮速反馈
    public:
        //左后轮轮速
        //起始位 0 长度 16
        int16_t Auto_WheelSpd_LeftBack() {
            return get<int16_t>(0, 16, can::CanData::LITTLE_ENDIAN);
        }
        //左后轮脉冲数
        //起始位 16 长度 32
        int32_t Auto_WheelPulse_LeftBack() {
            return get<int32_t>(16, 32, can::CanData::LITTLE_ENDIAN);
        }
        //心跳信号(循环计数器)
        //起始位 52 长度 4
        uint8_t Auto_LeftWheelSpd_AliveCounter() {
            return get<uint8_t>(52, 4, can::CanData::LITTLE_ENDIAN);
        }
        //消息校验
        //起始位 56 长度 8
       uint8_t Auto_LeftWheelSpd_CheckSum() {
            return get<uint8_t>(56, 8, can::CanData::LITTLE_ENDIAN);
        }
    };

    /*
    * Msg_ID 0x18C4D8EF 
    * Rate 10ms
    * Delay_Time none
    */
    class r_wheel_fb : public can::CanData {
    public:
        explicit r_wheel_fb(unsigned char *data) : can::CanData(data, 8) {} //轮速反馈
    public:
        //右后轮轮速
        //起始位 0 长度 16
        int16_t Auto_WheelSpd_RightBack() {
            return get<int16_t>(0, 16, can::CanData::LITTLE_ENDIAN);
        }
        //右后轮脉冲数
        //起始位 16 长度 32
        int32_t Auto_WheelPulse_RightBack() {
            return get<int32_t>(16, 32, can::CanData::LITTLE_ENDIAN);
        }
        //心跳信号(循环计数器)
        //起始位 52 长度 4
        uint8_t Auto_RightWheelSpd_AliveCounter() {
            return get<uint8_t>(52, 4, can::CanData::LITTLE_ENDIAN);
        }
        //消息校验
        //起始位 56 长度 8
        uint8_t Auto_RightWheelSpd_CheckSum() {
            return get<uint8_t>(56, 8, can::CanData::LITTLE_ENDIAN);
        }
    };
    /*
    * Msg_ID 0x18C4DAEF
    * Rate 50ms
    * Delay_Time none
    */
    class io_fb : public can::CanData {
    public:
        explicit io_fb(unsigned char *data) : can::CanData(data, 8) {} //轮速反馈
    public:
        //安全停车解锁状态反馈
        //起始位 1 长度 1
        bool SafeParking_Back() {
            return get<bool>(1, 1, can::CanData::LITTLE_ENDIAN);
        }
         //急停开关状态反馈
        //起始位 40 长度 1
        bool EmergencyStop_Back() {
            return get<bool>(1, 1, can::CanData::LITTLE_ENDIAN);
        }
        //遥控器控制状态反馈
        //起始位 40 长度 1
        bool RemoteControl_Back() {
            return get<bool>(41, 1, can::CanData::LITTLE_ENDIAN);
        }
        //心跳信号(循环计数器)
        //起始位 52 长度 4
        int8_t Auto_EmergencyStop_RemoteControl_SafeParking_AliveCounter() {
            return get<int8_t>(52, 4, can::CanData::LITTLE_ENDIAN);
        }
        //消息校验
        //起始位 56 长度 8
        int8_t Auto_EmergencyStop_RemoteControl_SafeParking_CheckSum() {
            return get<int8_t>(56, 8, can::CanData::LITTLE_ENDIAN);
        }
    };
    /*
    * Msg_ID 0x18C4E1EF
    * Rate 100ms
    * Delay_Time none
    */
    class bms_fb : public can::CanData {
    public:
        explicit bms_fb(unsigned char *data) : can::CanData(data, 8) {} //轮速反馈
    public:
        //当前电池电压
        //起始位 0 长度 16
        int16_t Current_Battery_Voltage() {
            return get<int16_t>(0, 16, can::CanData::LITTLE_ENDIAN);
        }
        //当前电池电流
        //起始位 16 长度 16
        int16_t Current_Battery_Current() {
            return get<int16_t>(16, 16, can::CanData::LITTLE_ENDIAN);
        }
        //当前电池电流
        //起始位 32 长度 16
        int16_t Current_Remaining_Battery_Capacity() {
            return get<int16_t>(32, 16, can::CanData::LITTLE_ENDIAN);
        }
        //心跳信号(循环计数器)
        //起始位 52 长度 4
        int8_t Auto_Battery_AliveCounter() {
            return get<int8_t>(52, 4, can::CanData::LITTLE_ENDIAN);
        }
        //消息校验
        //起始位 56 长度 8
        int8_t Auto_Battery_CheckSum() {
            return get<int8_t>(56, 8, can::CanData::LITTLE_ENDIAN);
        }
    };
    /*
    * Msg_ID 0x18C4E2EF
    * Rate 100ms
    * Delay_Time none
    */
    class bms_flag_fb : public can::CanData {
    public:
        explicit bms_flag_fb(unsigned char *data) : can::CanData(data, 8) {} //轮速反馈
    public:
         //当前剩余电量百分比
        //起始位 0 长度 8
        int8_t Current_Remaining_Battery_Percentage() {
            return get<int8_t>(0, 8, can::CanData::LITTLE_ENDIAN);
        }
         //单体过压保护
        //起始位 8 长度 1
        bool Monomer_UnderVoltage_Protection() {
            return get<bool>(8, 1, can::CanData::LITTLE_ENDIAN);
        }
         //单体欠压保护
        //起始位 9 长度 1
        bool Monomer_OverVoltage_Protection() {
            return get<bool>(8, 1, can::CanData::LITTLE_ENDIAN);
        }
         //整组过压保护
        //起始位 10长度 1
        bool Whole_UnderVoltage_Protection() {
            return get<bool>(10, 1, can::CanData::LITTLE_ENDIAN);
        }
         //整组欠压保护
        //起始位 11 长度 1
        bool Whole_OverVoltage_Protection() {
            return get<bool>(11, 1, can::CanData::LITTLE_ENDIAN);
        }
       //充电过温保护
        //起始位 12长度 1
        bool Charge_High_Temperature_Protection() {
            return get<bool>(12, 1, can::CanData::LITTLE_ENDIAN);
        }
         //充电低温保护
        //起始位 13 长度 1
        bool Charge_Low_Temperature_Protection() {
            return get<bool>(13, 1, can::CanData::LITTLE_ENDIAN);
        }
       //放电过温保护
        //起始位 14 长度 1
        bool DisCharge_High_Temperature_Protection() {
            return get<bool>(14, 1, can::CanData::LITTLE_ENDIAN);
        }
         //放电低温保护
        //起始位 15 长度 1
        bool DisCharge_Low_Temperature_Protection() {
            return get<bool>(15, 1, can::CanData::LITTLE_ENDIAN);
        }
         //充电过流保护
        //起始位 16 长度 1
        bool Charge_Overcurrent_Protection() {
            return get<bool>(16, 1, can::CanData::LITTLE_ENDIAN);
        }
       //放电过流保护
        //起始位 17 长度1
        bool DisCharge_Overcurrent_Protection() {
            return get<bool>(17, 1, can::CanData::LITTLE_ENDIAN);
        }
       //短路保护
        //起始位 18 长度1
        bool Short_Circuit_Protection() {
            return get<bool>(18, 1, can::CanData::LITTLE_ENDIAN);
        }
       //前端检测IC错误
        //起始位 19 长度1
        bool FrontEndDetection_IC_Error() {
            return get<bool>(19, 1, can::CanData::LITTLE_ENDIAN);
        }
       //软件锁定MOS
        //起始位 20 长度1
        bool Software_Lock_MOS() {
            return get<bool>(20, 1, can::CanData::LITTLE_ENDIAN);
        }
       //充电标志位
        //起始位 21 长度1
        bool Charging_Flag() {
            return get<bool>(21, 1, can::CanData::LITTLE_ENDIAN);
        }
        //当前电池最高温度
        //起始位 28 长度 12
        int16_t Current_Battery_MaxTemperature() {
            return get<int16_t>(28, 12, can::CanData::LITTLE_ENDIAN);
        }
        //当前电池最低温度
        //起始位 40 长度 12
        int16_t Current_Battery_MinTemperature() {
            return get<int16_t>(40, 12, can::CanData::LITTLE_ENDIAN);
        }
        //心跳信号(循环计数器)
        //起始位 52 长度 4
        int8_t Auto_Battery_Flag_AliveCounter() {
            return get<int8_t>(52, 4, can::CanData::LITTLE_ENDIAN);
        }
        //消息校验
        //起始位 56 长度 8
        int8_t Auto_Battery_Flag_CheckSum() {
            return get<int8_t>(56, 8, can::CanData::LITTLE_ENDIAN);
        }
    };

    /*--------------------------控制------------------------*/
     class ControlData { 
    private:
        ctrl_cmd _ctrl_cmd;
        free_ctrl_cmd _free_ctrl_cmd;
        io_cmd _io_cmd;
        uint8_t crc(std::vector<unsigned char>::const_iterator begin, std::vector<unsigned char>::const_iterator end) {   //遍历容器内的元素，并访问这些元素的值。iterator可以改元素值,但const_iterator不可改
            uint8_t ret = 0xff;
            for (auto it = begin; it != end; it++) {
                ret ^= *it;
            }
            return ret ^ 0xff;
        }
    public:
        ctrl_cmd &Ctrl_cmd() {
            return _ctrl_cmd;
        }
        free_ctrl_cmd &Free_ctrl_cmd() {
            return _free_ctrl_cmd;
        }
        io_cmd &Io_cmd() {
            return _io_cmd;
        }
    public:
        /*---------------------------运动学控制指令帧-----------------------------*/
        void targetSpeed(float targetSpeed) {  //目标速度（这四行正确）
            targetSpeed *= 1000;  //分辨率 1毫米每妙
            _ctrl_cmd.Auto_Drive_Target(static_cast<int16_t>(targetSpeed));
        }

        void targetAngleSpeed(float targetAngleSpeed) {  //（这四行已改）
            targetAngleSpeed *= 100;  //分辨率 0.01度每妙
            _ctrl_cmd.Auto_Drive_AngleTarget(static_cast<int16_t>(targetAngleSpeed));
        }
        /*----------------------------自由控制指令帧------------------------------*/
        /*
        void targetLeftSpeed(float targetLeftSpeed) {  //左轮目标速度
            targetLeftSpeed *= 1000;  //分辨率 1毫米每妙
            _free_ctrl_cmd.Auto_free_Left_Drive_Target(static_cast<int16_t>(targetLeftSpeed));
        }
        void targetRightSpeed(float targetRightSpeed) {  //右轮目标速度
            targetRightSpeed *= 1000;  //分辨率 1毫米每妙
            _free_ctrl_cmd.Auto_free_Right_Drive_Target(static_cast<int16_t>(targetRightSpeed));
        }
*/
        /*---------------------------------其他--------------------------------------*/


        // void turnSignal(uint8_t turnSignal){
        //     switch(turnSignal){
        //         case 0:
        //             _auto_LightCmd.Auto_FrontLight_Enable(false);
        //         case 1:
        //         case 2:
        //         case 3:
        //     }
        // }

        void rollingCounter(uint8_t rc) { 
            _ctrl_cmd.Auto_Gear_AliveCounter(rc);
            _free_ctrl_cmd.Auto_free_Gear_AliveCounter(rc);
            _io_cmd.Auto_Parking_AliveCounter(rc);
        }

        void calcCheckSum() {  //消息校验
            _ctrl_cmd.Auto_Gear_CheckSum(crc(_ctrl_cmd.data().cbegin(), _ctrl_cmd.data().cend()-1));
            _free_ctrl_cmd.Auto_free_Gear_CheckSum(crc(_free_ctrl_cmd.data().cbegin(), _free_ctrl_cmd.data().cend()-1));
            _io_cmd.Auto_Parking_CheckSum(crc(_io_cmd.data().cbegin(), _io_cmd.data().cend()-1));
        }
    };
}
