#include <iostream>
#include <fstream>
#include <boost/asio.hpp>
#include <thread>
#include <chrono>
#include <vector>
#include <boost/optional.hpp>
#include <Eigen/Dense>
#include "../include/vehicle.h"
#include "../include/lidar.h"

using namespace boost::asio;
using namespace std::chrono_literals;

/*
int readFromUDP(unsigned short port) {
	io_service ios;
	ip::udp::endpoint local(ip::udp::v4(), port);
	ip::udp::socket sock(ios, local);
	std::cout << "here1" << std::endl;
	for (;;) {
		sensor::RS16DataPacket packet;
		boost::system::error_code ec;
		ip::udp::endpoint remote;
		size_t n = sock.receive_from(buffer(&packet, sizeof(packet)), remote, 0, ec);
		if (ec) {
			std::cerr << "error: " << ec << std::endl;
			return 1;
		}
		std::cout << "size: " << n << std::endl;                                                                      //尺寸
		std::cout << "remote address: " << remote.address() << std::endl;            //远端地址
		for (auto db : packet.dataBlocks) {
			if (db.flag != 0xffee) {
				std::cerr << "bad packet" << std::endl;
				return 1;
			}
			std::cout << "azimuth: " << db.azimuth << std::endl;                                     //方位角
			std::cout << "distance: " << db.channels[0][1].distance << std::endl;      //距离
		}
	}
}

int readFromFile() {
	std::ifstream log;
	log.open("log.pcap", std::ios::in | std::ios::binary);
	if (!log) {
		std::cerr << "can't open log file" << std::endl;
		return 1;
	}
	sensor::PcapGlobalHeader pcapGlobal;
	sensor::PcapPacketHeader pcapPacket;
	sensor::VLP16DataPacket packet;
	log.read((char*)&pcapGlobal, sizeof(pcapGlobal));                                                           //读取pcap文件
	while (!log.eof()) {                                                                                                                      //eof函数判断文件是否为空
		log.read((char*)&pcapPacket, sizeof(pcapPacket));
		if (pcapPacket.inclLen != sizeof(packet)) {
			std::unique_ptr<char[]> buffer = std::make_unique<char[]>(pcapPacket.inclLen);
			log.read(buffer.get(), pcapPacket.inclLen);
			continue;
		}
		log.read((char*)&packet, sizeof(packet));
		for (auto db : packet.dataBlocks) {
			if (db.flag != 0xeeff) {
				std::cerr << "bad packet" << std::endl;
				return 1;
			}
			std::cout << "azimuth: " << db.azimuth << std::endl;                                            //方位角
			std::cout << "distance: " << db.channels[0][1].distance << std::endl;               //距离
		}
	}
	return 0;
}

static int nextStep(const std::vector<sensor::LidarPoint> & c, int i) {
	for (int j = i; j < c.size(); j++) {
		if (c[j].azimuth() >= 85)
			return j;
		if (c[j].x() - c[i].x() >= 0.04)
			return j;
	}
	return -1;
}

static float findBorder(const std::vector<sensor::LidarPoint> & c) {   //寻找边界
	for (int i = 0; i < c.size(); i++) {
		if (c[i].azimuth() > 83)
			return -1;
		int next = nextStep(c, i);
		if (next < 0)
			return -1;
		//std::cout << "d: " << c[next].z() - c[i].z() << std::endl;
		if (c[next].z() - c[i].z() > 0.02) {
			//std::cout << "xyz: " << c[i].x() << "," << c[i].y() << "," << c[i].z() << std::endl;
			//std::cout << "distance: " << c[i].distance() << std::endl;
			std::cout << "azimuth: " << c[i].azimuth() << std::endl;
			return c[i].x();
		}
	}
	return -1;
}
*/

template<typename Lidar>
static boost::optional<float> measureBarrier(const vehicle::Geometry & vg, Lidar & lidar, float steeringAngle) {   //measureBarrier衡量障碍物
	boost::optional<float> ret = boost::none;   //optional  容器 ，返回无意义的值
	std::chrono::steady_clock::time_point ts;
	int ignore;
	auto mPtrOpt = lidar.current(ts, ignore);
	if (!mPtrOpt) {
		std::cout << "Lidar no data" << std::endl;
		return ret;
	}
	auto mPtr = *mPtrOpt;
	const auto & m = *mPtr;

	for (const auto & c : m) {
		for (const auto & p : c) {
			if (p.y() < vg.wheelBase() + vg.frontOverhang())  //前悬
				continue;
			if (p.z() < -1.2 || p.z() > -0.4)
				continue;
			;
			//boost::optional<float> d = vg.frontDistance(Eigen::Vector2f(p.x(), p.y() + vg.wheelBase() + vg.frontOverhang()), steeringAngle);
            boost::optional<float> d = vg.frontDistance(Eigen::Vector2f(p.x(), p.y() ), steeringAngle);  //前方距离
			if (!d)
				continue;
			if (ret) {
				if (*d < *ret)
					ret = d;
			}
			else {
				ret = d;
			}
		}
	}
	return ret;
}

int main(int argc, /*const*/ char** argv) {
	// Geometry(float wheelBase, float vehicleWidth, float frontOverhang, float rearOverhang, float safeDistance) 
	vehicle::Geometry vg(2.65f, 1.85f, 1.0f, 0.9f, 0.25f);
	
	// auto d = vg.frontDistance(Eigen::Vector2f(vg.vehicleWidth()/2, vg.wheelBase() + vg.frontOverhang()), 540);
	// if (d)
	// 	std::cout << "distance: " << *d << std::endl;
	// else
	// 	std::cout << "no barrier" << std::endl;
	// ros::init(argc, (char**)argv, "test_lidar");

	try {
	    Eigen::Affine3f transform = Eigen::Affine3f::Identity(); //Affine3f一种四维矩阵变换
	    transform.translation() = Eigen::Vector3f(0, vg.wheelBase()/2, 0);  //wheelbase轴距
		// sensor::Lidar16<sensor::VLP16, int> lidar(2368);
		// sensor::Lidar32<sensor::RS32, int> lidar(6700, transform);
        sensor::Lidar16<sensor::RS16, int> lidar(6699, transform); /*6699*/      

		int i = 0;
		for (;;) { 
			//ros::ok()
			// ros::spinOnce();
			auto distance = measureBarrier(vg, lidar, 0);                 //measureBarrier衡量障碍物
			if (distance) {
				std::cout << "distance: " << *distance << std::endl;         //距离
			}
			else {
				std::cout << "no barrier" << std::endl;           //没有障碍物
			}
			std::this_thread::sleep_for(50ms);
		}
	}
	catch (const std::exception & e) {                                  //异常处理
		std::cout << "e: " << e.what() << std::endl;
	}

	return 0;
}
