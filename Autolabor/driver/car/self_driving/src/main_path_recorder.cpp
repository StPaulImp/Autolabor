
#include <iostream>
#include <fstream>
#include <iomanip>
#include <boost/asio.hpp>
#include <boost/optional.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/program_options.hpp>
#include <boost/timer/timer.hpp>
#include <boost/optional.hpp>
#include <thread>
#include <chrono>
#include <cmath>
#include "../include/rtk.h"

using namespace boost;
using namespace boost::program_options;
using namespace sensor;
using namespace std::chrono_literals;


template <typename RTK> //rtk实时动态（载波相位差分技术）
static void recordPath(RTK & rtk, std::ostream & pathFile) {             //recordPath
    int i = 0;
	for (;;) {
		std::this_thread::sleep_for(10ms);
		boost::optional<LatLonAzimuth> llaOpt = rtk.current();  //LatL在方位角上  //函数计算从一个纬度经度点到另一纬度经度点的方向航向
		if (!llaOpt) {
            std::cout << "RTK no data" << std::endl;
			continue;
		}
		LatLonAzimuth lla = *llaOpt;
		pathFile << lla.latLon().lat() << " " << lla.latLon().lon() << " " << lla.azimuth() << std::endl;
		++i;
		if(100 == i) {
		    i = 0;
            std::cout << lla.latLon().lat() << " " << lla.latLon().lon() << " " << lla.azimuth()<< std::endl;
		}
	}
}

static int record(int argc, const char *argv[]) {                                         //record 
	options_description desc("options");            //desc排序函数
	variables_map vm;                              //变量图vm
	desc.add_options()
		("port", value<std::string>()->required(), "RTK serial port")   //RTK串口
		("path", value<std::string>()->required(), "path file")             //文件路径
		("stop-mark", value<std::string>()->required(), "stop mark file")   //停止标记文件
 		("station", value<std::string>()->required(), "station file");           //站文件
		// ("sim",value<bool>()->default_value(false),"Sim status");
	try {
		store(parse_command_line(argc, argv, desc), vm);
		notify(vm);
	}
	catch (const boost::program_options::error &ex) {
		std::cerr << ex.what() << '\n';
		return 1;
	}
	                //ofstream来记录path/stop-mark/station都属于“cant open”
	std::ofstream pathFile(vm["path"].as<std::string>());
	if (!pathFile) {
		std::cerr << "can't open " << vm["path"].as<std::string>() << std::endl;
		return 1;
	}
	std::ofstream stopMarkFile(vm["stop-mark"].as<std::string>());
	if (!stopMarkFile) {
		std::cerr << "can't open " << vm["stop-mark"].as<std::string>() << std::endl;
		return 1;
	}
    std::ofstream stationFile(vm["station"].as<std::string>());
    if (!stationFile) {
        std::cerr << "can't open " << vm["station"].as<std::string>() << std::endl;
        return 1;
    }

	std::cout << std::setprecision(12);  //设定精度
	pathFile << std::setprecision(12);
	stopMarkFile << std::setprecision(12);
    stationFile << std::setprecision(12);

    sensor::M39B m39b(vm["port"].as<std::string>(),true);    //M39B端口

	std::thread t([&]() {
		recordPath(m39b, pathFile);
	});
	
	std::string line;
	for (;;) {
		std::cin >> line;
		boost::optional<LatLonAzimuth> llaOpt = m39b.current();
		if (!llaOpt) {
			std::cout << "RTK no data" << std::endl;
			continue;
		}
		LatLonAzimuth lla = *llaOpt;
		if("m" == line) {
            stopMarkFile << lla.latLon().lat() << " " << lla.latLon().lon() << " " << lla.azimuth() << std::endl;
            std::cout << "stop mark: " << lla.latLon().lat() << " " << lla.latLon().lon() << " " << lla.azimuth() << std::endl;
		}else if("s" == line) {
            stationFile << lla.latLon().lat() << " " << lla.latLon().lon() << " " << lla.azimuth() << std::endl;
            std::cout << "station: " << lla.latLon().lat() << " " << lla.latLon().lon() << " " << lla.azimuth() << std::endl;
		}
	}
	t.join();
}

int main(int argc, const char *argv[]) {
	record(argc, argv);
	return 0;

}
