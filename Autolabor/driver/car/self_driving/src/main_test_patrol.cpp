#include <iostream>
#include <fstream>
#include <iomanip>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/program_options.hpp>
#include <boost/optional/optional_io.hpp>
#include <boost/optional.hpp>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <deque>
#include <array>
#include "../include/rtk.h"
#include "../include/vehicle.h"
#include "../include/lidar.h"
#include <net.h>
#include <ros/ros.h>

using namespace boost;
using namespace boost::program_options;
using namespace sensor;
using namespace std::chrono_literals;

static float MAX_SPEED = 5;
static const float PID_P = 100;

float maxSpeedSteeringAngle(float sa) {            //最大速度转向角
    if (std::abs(sa) > 20) {
        return 0.5;
    } else if (std::abs(sa) > 10) {
        return 1;
    } else if (std::abs(sa) > 5) {
        return 2;
    } else {
        return MAX_SPEED;
    }
}

bool closeToTurning(float curvature) {   //关闭转弯
    return curvature > 1.0f / 50.0f;           //曲率
}

static bool closeToStopMark(const Path & path, const LatLonAzimuth & lla) {     //关闭并停止标记
    auto od = path.nextStopMark(lla);
    if(!od) return false;
    return *od < 30;
}

static bool closeToTargetStation(const Path & path, const LatLonAzimuth & lla, boost::optional<int> optStation) {    //关闭目标状态
    if(!optStation) return false;
    auto od = path.stationDistance(lla, *optStation);
    if(!od) return false;
    return *od < 30;
}

float steeringAngle(const PathPoint &pp, const LatLonAzimuth &lla, float shift) {                       //转向角
    boost::optional<Vector2d> result = boost::none;
    Matrix3d e2l = lla.earthToLocalMatrix();                                                                                                 //地球到本地矩阵
    Vector3d local = e2l * (pp.point() + (pp.normal() * shift).cast<double>());                                                //强制类型转换
    double e = local.x() / std::max(local.y(), 0.01);
    double sa = -PID_P * e;                                          //////////////////////////////e和sa是什么
    std::cout << "sa e" << e << std::endl;
    if (sa > 24) sa = 24;
    if (sa < -24) sa = -24;
    return static_cast<float>(sa);
}

template<typename T, typename SpeedType = float>
class SmoothValue {                                                                               //定义平滑值
private:
    T _target{T()};
    T _current{T()};
    const SpeedType _incSpeed;  //上升速度u
    const SpeedType _decSpeed; //下降速度
    std::chrono::steady_clock::time_point _updateTime;    //时间戳更新
public:
    explicit SmoothValue(SpeedType speed) : _incSpeed(speed), _decSpeed(speed),
                                            _updateTime(std::chrono::steady_clock::now()) {}

    explicit SmoothValue(SpeedType incSpeed, SpeedType decSpeed) : _incSpeed(incSpeed), _decSpeed(decSpeed),
                                                                   _updateTime(std::chrono::steady_clock::now()) {}

    void update() {                  //更新返回当前值
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<float> d = now - _updateTime;
        _updateTime = now;
        if (_target > _current)
            _current = std::min(_target, static_cast<T>(_current + _incSpeed * d.count()));
        else if (_target < _current)
            _current = std::max(_target, static_cast<T>(_current - _decSpeed * d.count()));
    }
    T current() const {
        return _current;
    }
    void current(T v) {
        _current = v;
    }
    T target() const {
        return _target;
    }
    void target(T v) {
        _target = v;
    }
};

template<typename T, std::size_t Size>
class MedianFilter {                 //中值滤波器
private:
    std::deque<T> _samples;
public:
    void update(T v) {
        _samples.push_front(v);
        _samples.resize(Size, v);
    }
    T value() const {
        std::array<T, Size> sortedSamples{0.0f};
        std::copy(_samples.begin(), _samples.end(), sortedSamples.begin());
        std::sort(sortedSamples.begin(), sortedSamples.end());
        return sortedSamples[Size >> 1];
    }
};

class BarrierDetector {              //定义障碍物探测器
private:
    const vehicle::Geometry &_vg;
    MedianFilter<float, 7> resultFilter;

    template<typename PointCloud>
    boost::optional<float> _measure(const PointCloud &pointCloud, float steeringAngle) {
        boost::optional<float> ret = boost::none;

        for (const auto &c : pointCloud) {
            for (const auto &p : c) {
                if (p.y() < _vg.wheelBase() + _vg.frontOverhang()) //轴距加前悬
                    continue;
                if (p.z() < -1.1 || p.z() > -0.4)
                    continue;
                //boost::optional<float> d = _vg.frontDistance(Eigen::Vector2f(p.x(), p.y() + _vg.wheelBase() + _vg.frontOverhang()), steeringAngle);
                boost::optional<float> d = _vg.frontDistance(Eigen::Vector2f(p.x(), p.y()),
                                                             steeringAngle);
                if (!d)
                    continue;
                if (ret) {
                    if (*d < *ret)
                        ret = d;
                } else {
                    ret = d;
                }
            }
        }
        return ret;
    }
public:
    explicit BarrierDetector(const vehicle::Geometry &vg) : _vg(vg) {   //障碍物探测器
    }
    template<typename PointCloud>
    boost::optional<float> measure(const PointCloud &pointCloud, float steeringAngle) {   //利用但云信息和转向角度来定义
        auto sampleOpt = _measure(pointCloud, steeringAngle);
        float sample;
        if (sampleOpt)
            sample = *sampleOpt;
        else
            sample = 9999.9f;
        resultFilter.update(sample);
        float result = resultFilter.value();
        if (result < 9999.9f)
            return result;
        else
            return boost::none;
    }
};

class ShiftSolver {                                 ////////////////////////没看懂
private:
    const vehicle::Geometry &_vg;
    MedianFilter<float, 7> shiftFilter;        //移位过滤器
    MedianFilter<float, 7> marginFilter;  //边距过滤器

    template<typename PointCloud>
    void _measure(const PointCloud &pointCloud, const LatLonAzimuth &vehicle,  //点云 方位角  最近点
                  const PathPoint &nearestPoint, float forward) {
        float left = 9999;
        float margin = -9999;
        Affine3d v2p = nearestPoint.earthToLocalAffine() * vehicle.earthToLocalAffine().inverse();   //affine仿射（四维矩阵时提到过）

        for (const auto &c : pointCloud) {
            for (const auto &p : c) {
                if(
                    p.x() > -(_vg.vehicleWidth()/2 + _vg.safeDistance()) &&
                    p.x() < (_vg.vehicleWidth()/2 + _vg.safeDistance() + 0.5) &&
                    p.y() > -_vg.rearOverhang() &&
                    p.y() < (_vg.wheelBase() + _vg.frontOverhang() + _vg.safeDistance()))
                    continue;
                if (p.y() < 0)
                    continue;
                if (p.z() < -1.1 || p.z() > -0.4)
                    continue;
                //Vector3f pointInPath = (v2p * Vector3d(p.x(), p.y() + _vg.wheelBase() + _vg.frontOverhang(), p.z())).cast<float>();
                Vector3f pointInPath = (v2p * Vector3d(p.x(), p.y(), p.z())).cast<float>();

                /*
                if(pointInPath.y()> 15)
                    continue;
                if(pointInPath.x() < -2)
                    continue;
                if(pointInPath.x() < left)
                    left = pointInPath.x();
                */

                if (pointInPath.y() < forward && pointInPath.x() > -1.3 && pointInPath.x() < left)
                    left = pointInPath.x();

                if (pointInPath.y() < (2 + _vg.wheelBase() + _vg.frontOverhang()) &&
                    pointInPath.x() < -(_vg.vehicleWidth() * 0.5f + _vg.safeDistance()) && pointInPath.x() > margin)
                    margin = pointInPath.x();
            }
        }
        shiftFilter.update(std::min(0.0f, left - _vg.vehicleWidth() * 0.5f - _vg.safeDistance() - 0.1f) * 1.5f);
        marginFilter.update(margin + (_vg.vehicleWidth() * 0.5f + _vg.safeDistance() + 0.7));
    }

public:
    explicit ShiftSolver(const vehicle::Geometry &vg) : _vg(vg) {
    }
    template<typename PointCloud>
    float measure(const PointCloud &pointCloud, const LatLonAzimuth &vehicle,
                  const PathPoint &nearestPoint, float forward) {
        _measure(pointCloud, vehicle, nearestPoint, forward);
        float shift = shiftFilter.value();
        float margin = marginFilter.value();
        std::cout << "MARGIN" << margin << std::endl;
        if (shift < margin)
            return 0;
        if (shift < -1.5){
            shift = (-shift);
        }
        return shift;
    }
};

static float maxSpeedForBarrier(boost::optional<float> barrier) {              //障碍物的最大速度（无障碍物/障碍物距离）
    if (!barrier) {
        std::cout << "no barrier" << std::endl;
        return MAX_SPEED;
    }
    float distance = *barrier;
    std::cout << "barrier distance: " << distance << std::endl;
    float ret = distance - 10;
    return std::clamp(ret, 0.0f, MAX_SPEED);
}

// static float maxSpeed(const Path &path, sensor::TrafficLight::LightType lightType, const sensor::LatLonAzimuth &lla) {
//     std::cout << "traffic light: " << lightType << std::endl;
//     if (sensor::TrafficLight::RED != lightType) {
//         return MAX_SPEED;
//     }
//     boost::optional<float> optStopMark = path.nextStopMark(lla);
//     if (!optStopMark) {
//         return MAX_SPEED;
//     }
//     float stopMark = *optStopMark;
//     //std::cout << "stop mark distance: " << stopMark << std::endl;
//     float ret = stopMark - 1;
//     if (ret < 1)
//         ret = 0;
//     return ret;
//     return std::clamp(ret, 0.0f, MAX_SPEED);
// }

static float maxSpeed(net::UDPJSONReceiver &action) {                //最大速度
    auto ov = action.value();
    if (!ov)
        return 0.0f;
    auto v = *ov;
    auto obj = v.get<picojson::object>();
    if (obj["name"].get<std::string>() == "GO")
        return MAX_SPEED;
    else
        return 0;
}

static bool actionGo(net::UDPJSONReceiver &action) {
    auto ov = action.value();
    if (!ov)
        return false;
    auto v = *ov;
    auto obj = v.get<picojson::object>();                         //object目的
    return "GO" == obj["name"].get<std::string>();
}

static boost::optional<int> dest(net::UDPJSONReceiver &action) {
    auto ov = action.value();
    if (!ov)
        return boost::none;
    auto v = *ov;
    if (!v.is<picojson::object>())
        return boost::none;
    auto obj = v.get<picojson::object>();
    if (obj["name"].get<std::string>() != "GO")
        return boost::none;
    if (!obj["dest"].is<std::string>())
        return boost::none;
    std::string d = obj["dest"].get<std::string>();
    if (d == "1")
        return 0;
    else if (d == "2")
        return 1;
    else if (d == "3")
        return 2;
    else if (d == "4")
        return 3;
    else if (d == "5")
        return 4;
    else if (d == "6")
        return 5;
    else if (d == "7")
        return 6;
    else
        return boost::none;
}

static void sendStatus(net::UDPJSONSender &status, double lat, double lon, double azimuth, double speedKmh) {
    picojson::object s;
    picojson::object position;
    position["ready"] = picojson::value(true);
    position["lat"] = picojson::value(lat);
    position["lon"] = picojson::value(lon);
    position["azimuth"] = picojson::value(azimuth);
    position["speed_kmh"] = picojson::value(speedKmh);
    picojson::object voice;
    voice["id"] = picojson::value("0");
    voice["text"] = picojson::value("");
    s["position"] = picojson::value(position);
    s["voice"] = picojson::value(voice);
    status.value(picojson::value(s));
}

/*
template<typename Lidar>
int showObjectTrackingViewer(Lidar &lidar, sensor::ObjectTracking<Lidar> &objectTracking) {
    pcl::visualization::PCLVisualizer viewer("Test Viewer");
    viewer.addCoordinateSystem(10.0);
    while (!viewer.wasStopped()) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        std::chrono::steady_clock::time_point ts;
        int ignore;
        auto om = lidar.current(ts, ignore);
        if (!om)
            continue;
        auto m = *om;
        for (const auto &ch: *m) {
            for (const sensor::LidarPoint &lpt: ch) {
                if (lpt.z() < -1.5 || lpt.z() > 0.2 || lpt.distance() < 0.6)
                    continue;
                if (lpt.y() < 2 && lpt.y() > -2 && lpt.x() < 1 && lpt.x() > -1)
                    continue;
                cloudPtr->push_back(pcl::PointXYZ(lpt.x(), lpt.y(), 0));
            }
        }
        cloudPtr->width = cloudPtr->points.size();
        cloudPtr->height = 1;
        cloudPtr->is_dense = true;
        viewer.removeAllPointClouds();
        viewer.addPointCloud(cloudPtr);
        viewer.removeAllShapes();
        auto objects = objectTracking.objects();
        int shapeIndex = 0;
        for (const auto &object : *objects) {
            std::string cubeName = "shape_" + std::to_string(shapeIndex);
            std::string arrowName = "arrow_" + std::to_string(shapeIndex);
            shapeIndex++;
            const Eigen::Vector3d &center = object.position();
            float cubeSize = 0.5f;
            viewer.addCube(center.x() - cubeSize, center.x() + cubeSize, center.y() - cubeSize, center.y() + cubeSize,
                           0, 0, 1, 0.0, 0.0, cubeName);
            const Eigen::Vector3f ep2 = center.cast<float>() + object.velocity() * 0.5;
            pcl::PointXYZ p1(center.x(), center.y(), 0), p2(ep2.x(), ep2.y(), 0);
            viewer.addLine(p1, p2, 0, 1, 0, arrowName);
        }
        std::this_thread::sleep_for(20ms);
        viewer.spinOnce();
    }
    return 0;
}
*/

static int run(int argc, const char *argv[]) {                               //run
    options_description desc("options");
    variables_map vm;
    desc.add_options()
            ("port", value<std::string>()->default_value("192.168.8.100"), "Rtk port")  //rtkwangluo地址
            ("vport", value<std::string>()->default_value("can0"), "Vehicle port") //can地址
            ("max-speed", value<float>()->default_value(5.0), "Max speed")
            ("sim",value<bool>()->default_value(false),"Sim status");//模拟量
    try {
        store(parse_command_line(argc, argv, desc), vm);
        notify(vm);
    }catch (const boost::program_options::error &ex) {
        std::cerr << ex.what() << '\n';
        return 1;
    }
    
    MAX_SPEED = vm["max-speed"].as<float>();
    net::UDPJSONReceiver action(10003);
    net::UDPJSONSender status("127.0.0.1", 10002);               //127.0.0.1
    std::cout << "load map" << std::endl;
    Path path("map", vm["sim"].as<bool>(), MAX_SPEED);
    std::cout << "load map done" << std::endl;
    StationBehavior sb(path);
    ros::init(argc, (char **)argv, "test_patrol");
    vehicle::YHS yhs(vm["vport"].as<std::string>());
    sensor::M39B m39b(vm["port"].as<std::string>(), vm["sim"].as<bool>());
    // Geometry(float wheelBase, float vehicleWidth, float frontOverhang, float rearOverhang, float safeDistance) 
    vehicle::Geometry vg(0.85f, 0.606f, 0.37f, 0.37f, 0.3f);//车体宽度/安全距离
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.linear() = Eigen::AngleAxisf(0.017444444 * 0, Eigen::Vector3f::UnitZ()).toRotationMatrix();  //angleaxif偏移量
    transform.translation() = Eigen::Vector3f(0, vg.wheelBase()/2, 0);
    sensor::Lidar16<sensor::RS16, int> lidar(6699, transform);

    /*
    sensor::Lidar32<sensor::RS32, int> lidar(6700, transform);
    Eigen::Affine3f transformRight = Eigen::Affine3f::Identity();
    transformRight.linear() = Eigen::AngleAxisf(3.142 * 0.25, Eigen::Vector3f::UnitY()).toRotationMatrix();
    transformRight.translation() = Eigen::Vector3f(66.0f,  vg.wheelBase()/2, 0);
    sensor::Lidar16<sensor::RS16, int> lidarRight(6701, transformRight);
    sensor::ObjectTracking<sensor::Lidar32<sensor::RS32, int>> objectTracking(lidar);
    std::thread showObjectTrackingViewerThread([&lidar, &objectTracking](){showObjectTrackingViewer(lidar, objectTracking);});
    sensor::Lidar16<sensor::VLP16> lidar(2368);
    */

    BarrierDetector barrierDetector(vg);
    ShiftSolver shiftSolver(vg);
    // ShiftSolver rightShiftSolver(vg);//偏转解析值
    // std::cout << "connectiing traffic light" << std::endl;
    // std::cout << "connected" << std::endl;
    SmoothValue<float> smoothShift(0.5, 0.5);//平化处理
    SmoothValue<float> smoothSpeed(0.5);
    bool previousGo = false;

    yhs.start();                                                                 //////yhs  start
    while(ros::ok()) {
        ros::spinOnce();
        picojson::object errorStatus;
        boost::optional<LatLonAzimuth> llaOpt;

        if (path.sim()){
            std::cout << "path.sim() 1" << path.sim() << std::endl;
            llaOpt = m39b.current();
        }else{
            std::cout << "path.sim() 2" << path.sim() << std::endl;
            llaOpt = path.current();
        }
        
        if (!llaOpt) {
            std::cout << "RTK no data" << std::endl;
            errorStatus["rtk"] = picojson::value("NO_DATA");
            continue;
        }

        std::chrono::steady_clock::time_point ts;

        int ignore;
        auto pointCloudOpt = lidar.current(ts, ignore);
        if (!pointCloudOpt) {
            std::cout << "Lidar no data" << std::endl;
            errorStatus["lidar_front"] = picojson::value("NO_DATA");
        }

        if (!llaOpt || !pointCloudOpt ) {
            errorStatus["ready"] = picojson::value(false);
            status.value(picojson::value(errorStatus));
            yhs.targetSpeed(0);
            continue;
        }

        LatLonAzimuth lla = *llaOpt;
        auto pointCloudPtr = *pointCloudOpt;
        // auto pointCloudRightPtr = *pointCloudRightOpt;
        const PathPoint &targetPoint = path.nearest(lla, 3);   //nearst 做一个差值 ，做3m
        // std::cout << "targetPoint=" <<  targetPoint.point() << std::endl;
        const PathPoint &nearestPoint = path.nearest(lla, 0);   //车体和当前点，在追踪起始的时候偏移量
        // std::cout << "nearestPoint=" <<  nearestPoint.point() << std::endl;

        /*
        auto od = dest(action);
        if (od) {
            std::cout << "target station: " << *od << std::endl;
            sb.target(*od);
        }
        */

        // auto shiftRight = rightShiftSolver.measure(*pointCloudRightPtr, lla, nearestPoint, 4);
        // std::cout << "SHIFT_RIGHT: " << shiftRight << std::endl;

        auto shift = shiftSolver.measure(*pointCloudPtr, lla, nearestPoint, 10);                                    // 点云和附近点云                
        std::cout << "SHIFT: " << shift << std::endl;
        // auto shiftFinal = std::min(shift, shiftRight);
        // std::cout << "SHIFT_FINAL: " << shiftFinal << std::endl;
        if(closeToTurning(path.maxCurvature(nearestPoint, 3))) {
            std::cout << "SHIFT close to turning" << std::endl;                  //shift关闭转弯
            smoothShift.target(shift);
        } 

        /*else if(closeToStopMark(path, lla)) {
            std::cout << "SHIFT close to stop mark" << std::endl;
            smoothShift.target(shift);
        } else if(closeToTargetStation(path, lla, od)) {
            std::cout << "SHIFT close to target station" << std::endl;
            smoothShift.target(shift);
        }*/
        /*else {
            smoothShift.target(shiftFinal);
        }*/

        smoothShift.update();                                                                                                                  //////////////smoothShift_update
        // std::cout << "smoothShift.current()" << smoothShift.current() << std::endl;
        float sa = steeringAngle(targetPoint, lla, smoothShift.current());  //计算转角
        std::cout << "turn angle sa： " << sa << std::endl;
        std::vector<float> maxspeeds;
        /*
        auto distance = barrierDetector.measure(*pointCloudPtr, sa);
        std::cout << "distance: " << distance << std::endl;
        maxspeeds.push_back(maxSpeedForBarrier(distance));  //障碍物允许的最大速度
        */
        // maxspeeds.push_back(maxSpeed(path, trafficLightType, lla));
        // maxspeeds.push_back(sb.maxSpeed(lla, MAX_SPEED));
        // maxspeeds.push_back(path.maxSpeed(nearestPoint, MAX_SPEED));
        maxspeeds.push_back(maxSpeed(action));

        //float maxspeed = *std::min_element(maxspeeds.begin(), maxspeeds.end());
        float maxspeed = maxSpeedSteeringAngle(sa);  //角速度补偿，当前偏移量+周期乘以方向盘转角的角速度
        smoothSpeed.target(maxspeed);
        if (smoothSpeed.target() < smoothSpeed.current())
            smoothSpeed.current(smoothSpeed.target());
        smoothSpeed.update();

        sendStatus(status, lla.latLon().lat(), lla.latLon().lon(), lla.azimuth(), smoothSpeed.current());

        if (smoothSpeed.current() > MAX_SPEED) {
            std::cout << "***error speed: " << smoothSpeed.current() << std::endl;
            smoothSpeed.current(MAX_SPEED);
        }

        yhs.targetSteeringAngle(sa);                                                //目标角度输出
        std::cout << "speed: " << smoothSpeed.current() << std::endl;
        // std::cout << "MAX_SPEED" << MAX_SPEED << std::endl;
        yhs.targetSpeed(smoothSpeed.current()); 

        // bool go = actionGo(action);
        // if (go && !previousGo) {
        //     std::cout << "start vehicle" << std::endl;
        //     yhs.start();
        // }
        // previousGo = go;
        
        // Path::Directions directions = path.nearestDirections(nearestPoint, 10);
        // switch (directions) {
        //     case Path::GO_STRAIGHT:
        //         // std::cout << "directions go straight" << std::endl;
        //         yhs.turnSignal(vehicle::YHS::TurnSignal::Off);
        //         break;
        //     case Path::TURN_LEFT:
        //         // std::cout << "directions turn left" << std::endl;
        //         yhs.turnSignal(vehicle::YHS::TurnSignal::Left);
        //         break;
        //     case Path::TURN_RIGHT:
        //         // std::cout << "directions turn right" << std::endl;
        //         yhs.turnSignal(vehicle::YHS::TurnSignal::Right);
        //         break;
        // }

        std::this_thread::sleep_for(20ms);
    }
    // showObjectTrackingViewerThread.join(); //完成最终控制流程
}


int main(int argc, const char *argv[]) {
    run(argc, argv);
    return 0;
}
