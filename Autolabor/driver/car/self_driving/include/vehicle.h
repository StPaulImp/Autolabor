#pragma once

#include <boost/endian/arithmetic.hpp>
#include <boost/optional.hpp>
#include <boost/thread.hpp>
#include <Eigen/Dense>
#include <vector>
#include <Eigen/StdVector>
#include <atomic>
#include <thread>
#include <mutex>
#include <string>
#include <iostream>
#include <math.h>

#include "net.h"
#include "candata_yhs.h"

//负数是顺时针
namespace vehicle {
    using namespace boost::endian;
    using namespace std::chrono;
    using namespace std::chrono_literals;
    // using namespace serial;

    struct Command {
        // little_uint8_t hdr1;
        // little_uint8_t hdr2;
        // little_uint8_t hdr3;
        // little_uint8_t len;
        // little_uint8_t cmd;
        // little_int16_t targetSteeringAngle;
        little_int16_t targetAngleSpeed;
        little_int16_t targetSpeed;
        little_uint8_t targetGear;
        little_uint8_t stopOrGo;
        little_uint8_t controlEnable;
        //little_uint8_t targetLedStatus;
        //little_uint8_t lightSwitch;
        little_uint8_t speedGear;
        little_uint8_t sum;
        void set(float targetSteeringAngle, float targetSpeed, uint8_t speedGear, bool control);
	};

	struct Status {
		big_uint8_t hdr1;
		big_uint8_t hdr2;
		big_uint8_t hdr3;
		big_uint8_t len;
		big_uint8_t cmd;
		big_uint8_t emBrakerStatus;
		big_uint8_t epsStatus;
		big_uint8_t torqueStatus;
		big_uint8_t decStatus;
		big_uint8_t sysStatus;
		big_uint8_t ucGearCtrlStatus;
		big_uint8_t brakePedalStatus;
		big_uint8_t cruiseStatus;
		big_uint8_t ucGearPosition;
		big_int16_t uiSpeed;
		big_int16_t uiSteerAngle;
		big_uint8_t sum;
	};

	void setupSerialPort(boost::asio::serial_port & port);         //设置串口
	Status send(boost::asio::serial_port & port,const Command & command, boost::system::error_code & ec);
	void post(boost::asio::serial_port & port, const Command & command, boost::system::error_code & ec);
    void post_serial(boost::asio::serial_port & port, const std::vector<char> & command, boost::system::error_code & ec);

	//straight  直行
	//turning  转弯
	class Geometry { //几何模型
	private:
		float _wheelBase;  //轴距
		float _vehicleWidth; //车宽度
		float _frontOverhang; //前悬
		float _rearOverhang;  //后悬
		float _safeDistance;  //安全距离
	private:
		boost::optional<float> straightFrontDistance(const Eigen::Vector2f & point) const;
		boost::optional<float> turningFrontDistance(const Eigen::Vector2f & point, float steeringAngle) const;
		float radiusOfTurning(float steeringAngle) const;
		Eigen::Vector2f centerOfTurning(float r, float steeringAngle) const;
	public:
		float wheelBase() const { return _wheelBase; }
		float vehicleWidth() const { return _vehicleWidth; }
		float frontOverhang() const { return _frontOverhang; }
		float rearOverhang() const { return _rearOverhang; }
		float safeDistance() const { return _safeDistance; }
	public:
		Geometry(float wheelBase, float vehicleWidth, float frontOverhang, float rearOverhang, float safeDistance) :
			_wheelBase(wheelBase), _vehicleWidth(vehicleWidth), _frontOverhang(frontOverhang), _rearOverhang(rearOverhang), _safeDistance(safeDistance) {

		}
		boost::optional<float> frontDistance(const Eigen::Vector2f & point, float steeringAngle) const;
	};

    class YHS {
    public:
        enum class TurnSignal{
            Off = 0x00,
            Left = 0x01,
            Right = 0x02,
            Both = 0x03
        };

        enum class CurrentGearPositon {
            Disable =0x00,
            Parking = 0x01,
            Neutral  = 0x02,
            Kinematics  = 0x03,
            Free = 0x04
        };

        enum class TargetGearPositon {
            Parking = 0x01,
            Reverse = 0x02,
            Neutral = 0x03,
            Drive = 0x04
        };
    private:
        enum class BrakePedalStatusValueTable {
            NotPressed = 0x0,
            Pressed = 0x1,
            Stopped = 0x2
        };

        enum class VCU_Mode{
            Auto = 0x00,
            Remote = 0x01,
            Stop = 0x02
        };

        enum class PakingStatus {   //停止状态
            ParkApplied = 0x01,
            CompletelyReleased = 0x03
        };

        std::thread _rx, _tx;
        boost::asio::io_service _io;
        net::CanRaw::socket _controlSocket;
        void predoControl(yhs_wire_protocol::ControlData & cd);
        void doControl(yhs_wire_protocol::ControlData & cd);
        void _rxFunc();
        void _txFunc();
        int _ptReadyTimer { 0 };
        std::atomic_bool _epasFailed { true };
        std::atomic_bool _controlFeedback { false };
        std::atomic_bool _previousControlFeedback { false };
        std::atomic_bool _controlSwitchButton { 0 };
        std::atomic_bool _previousControlSwitchButton { 0 };
        std::atomic<BrakePedalStatusValueTable> _emsBrakePedalStatus{ BrakePedalStatusValueTable::NotPressed};
        std::atomic<PakingStatus> _emsPakingStatus{ PakingStatus::ParkApplied };
        std::atomic<CurrentGearPositon> _vcuCurGearPos { CurrentGearPositon::Parking };
        std::atomic<float> _currentSteeringAngle { 0.0f };
        std::atomic<float> _previousSteeringAngle { 0.0f };
        std::atomic_bool _vehicleSpeedValid { false };
        std::atomic<float> _previousVehicleSpeed { 0.0f };
        std::atomic<float> _previousAngleSpeed { 0.0f };
        std::atomic<float> _vehicleSpeed { 0.0f };
        std::atomic<float> _vehicleAngleSpeed {0.0f};
        std::atomic<float> _targetSteeringAngle { 0.0f };
        std::atomic<float> _targetSpeed { 0.0f };
        std::atomic<float> _targetAngleSpeed {0.0f};
        std::atomic<float> _curLeftSpeed {0.0f};
        std::atomic<float>  _curLeftPlusSpeed{0.0f};
        std::atomic<float>  _curRightSpeed{0.0f};
        std::atomic<float> _curRightPlusSpeed{0.0f};
        std::atomic<float> _acch { 1.5f };
        std::atomic<float> _accl { -7.0f };
        std::atomic<TargetGearPositon> _gear { TargetGearPositon::Parking };
        std::atomic_int  _preAutocontrol{ 0 };
        std::atomic_bool _controlEnable { false };
        std::atomic_uint8_t _escApaLcStatus { 0 };
        std::atomic<VCU_Mode>  _ptReady { VCU_Mode::Auto };
        std::atomic<VCU_Mode>  _previousPtReady { VCU_Mode::Auto };
        std::atomic_bool _collisionFront { false };
        std::atomic_bool _collisionBack { false };
        int _keyBoardControlcounter { 0 };
        std::atomic_bool _frontCollision { false };
        std::atomic_bool _backCollision { false };
        std::atomic_bool  _fireUpHandle { false };
        std::atomic<TurnSignal> _turnSignal;
        //std::atomic<std::string &> _sPort;
    public:
        explicit YHS(const std::string &ifname);

        inline float targetSteeringAngle() { return _targetSteeringAngle; }
        void targetSteeringAngle(float value) { _targetSteeringAngle = value; }

        inline float targetSpeed() { return _targetSpeed; }
        void targetSpeed(float value) {
            if(value < 0.1)
                value = 0;
            _targetSpeed= value;
        }

        inline float targetAngleSpeed(){ return _targetAngleSpeed; }
        void targetAngleSpeed(float value){
            _targetAngleSpeed = value/M_PI*180.0f;
        }

        inline float accHighLimit() { return _acch; }   //限制上限
        void accHighLimit(float value) { _acch = value; }

        inline float accLowLimit() { return _accl; }    //限制下限
        void accLowLimit(float value) { _accl = value; }

        inline TargetGearPositon gear() { return _gear; }  //目标档位
        void gear(TargetGearPositon value) { _gear = value; }

        //std::string sPort() {return _sPort;}
        //void sPort(std::string& value) {std::copy(value.cbegin(),value.cend(),_sPort);}

        bool fireUpHandle(){ return _fireUpHandle; }  //手柄
        void fireUpHandle(bool vaule){ _fireUpHandle = vaule; }

        void start(){ _preAutocontrol = 1; }

        TurnSignal turnSignal(){ return _turnSignal.load(); }
        void turnSignal(TurnSignal vaule){ _turnSignal = vaule; }
    };

    class YHS_DGT001M {
    public:
        enum class TurnSignal{
            Off = 0x00,
            Left = 0x01,
            Right = 0x02,
            Both = 0x03
        };

        enum class CurrentGearPositon {
            Disable =0x00,
            Parking = 0x01,
            Neutral  = 0x02,
            Kinematics  = 0x03,
            Free = 0x04
        };

        enum class TargetGearPositon {
            Parking = 0x01,
            Reverse = 0x02,
            Neutral = 0x03,
            Drive = 0x04
        };
    private:
        enum class BrakePedalStatusValueTable {
            NotPressed = 0x0,
            Pressed = 0x1,
            Stopped = 0x2
        };

        enum class VCU_Mode{
            Auto = 0x00,
            Remote = 0x01,
            Stop = 0x02
        };

        enum class PakingStatus {   //停止状态
            ParkApplied = 0x01,
            CompletelyReleased = 0x03
        };
    public:
        boost::asio::io_service _io;
        net::CanRaw::socket _controlSocket;
        void predoControl(yhs_wire_protocol::ControlData & cd);
        int _ptReadyTimer { 0 };
        std::atomic_bool _epasFailed { true };
        std::atomic_bool _controlFeedback { false };
        std::atomic_bool _previousControlFeedback { false };
        std::atomic_bool _controlSwitchButton { 0 };
        std::atomic_bool _previousControlSwitchButton { 0 };
        std::atomic<BrakePedalStatusValueTable> _emsBrakePedalStatus{ BrakePedalStatusValueTable::NotPressed};
        std::atomic<PakingStatus> _emsPakingStatus{ PakingStatus::ParkApplied };
        std::atomic<CurrentGearPositon> _vcuCurGearPos { CurrentGearPositon::Parking };
        std::atomic<float> _currentSteeringAngle { 0.0f };
        std::atomic<float> _previousSteeringAngle { 0.0f };
        std::atomic<float> _previousVehicleSpeed { 0.0f };
        std::atomic<float> _previousAngleSpeed { 0.0f };
        std::atomic_bool _vehicleSpeedValid { false };
        std::atomic<float> _vehicleSpeed { 0.0f };
        std::atomic<float> _vehicleAngleSpeed {0.0f};
        std::atomic<float> _targetSteeringAngle { 0.0f };
        std::atomic<float> _targetSpeed { 0.0f };
        std::atomic<float> _targetAngleSpeed {0.0f};
        std::atomic<float> _curLeftSpeed {0.0f};
        std::atomic<float>  _curLeftPlusSpeed{0.0f};
        std::atomic<float>  _curRightSpeed{0.0f};
        std::atomic<float> _curRightPlusSpeed{0.0f};
        std::atomic<float> _acch { 1.5f };
        std::atomic<float> _accl { -7.0f };
        std::atomic<TargetGearPositon> _gear { TargetGearPositon::Parking };
        std::atomic_int  _preAutocontrol{ 0 };
        std::atomic_bool _controlEnable { false };
        std::atomic_uint8_t _escApaLcStatus { 0 };
        std::atomic<VCU_Mode>  _ptReady { VCU_Mode::Auto };
        std::atomic<VCU_Mode>  _previousPtReady { VCU_Mode::Auto };
        std::atomic_bool _collisionFront { false };
        std::atomic_bool _collisionBack { false };
        int _keyBoardControlcounter { 0 };
        std::atomic_bool _frontCollision { false };
        std::atomic_bool _backCollision { false };
        std::atomic_bool  _fireUpHandle { false };
        std::atomic<TurnSignal> _turnSignal;
        //std::atomic<std::string &> _sPort;
    public:
        explicit YHS_DGT001M(const std::string &ifname);

        inline float targetSteeringAngle() {return _targetSteeringAngle;}
        void targetSteeringAngle(float value) {_targetSteeringAngle = value;}

        inline float targetSpeed() {return _targetSpeed;}
        void targetSpeed(float value) {
            // if(fabs(value) < 0.1)
            //     value = 0;
            _targetSpeed= value;
        }

        inline float targetAngleSpeed(){return _targetAngleSpeed;}
        void targetAngleSpeed(float value){
            _targetAngleSpeed = value/M_PI*180.0f;
        }

        inline float accHighLimit() {return _acch;}   //限制上限
        void accHighLimit(float value) {_acch = value;}

        inline float accLowLimit() {return _accl;}    //限制下限
        void accLowLimit(float value) {_accl = value;}

        inline TargetGearPositon gear() {return _gear;}  //目标档位
        void gear(TargetGearPositon value) {_gear = value;}

        //std::string sPort() {return _sPort;}
        //void sPort(std::string& value) {std::copy(value.cbegin(),value.cend(),_sPort);}

        bool fireUpHandle(){return _fireUpHandle;}  //手柄
        void fireUpHandle(bool vaule){_fireUpHandle = vaule;}

        void start(){_preAutocontrol = 1;}

        TurnSignal turnSignal(){return _turnSignal.load();}
        void turnSignal(TurnSignal vaule){_turnSignal = vaule;}

        virtual void doControl(yhs_wire_protocol::ControlData & cd);

        std::thread _rx, _tx;
        
        virtual void _rxFunc();
        virtual void _txFunc();
    };
};
