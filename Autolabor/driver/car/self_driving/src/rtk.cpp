
#include <iostream>
#include <fstream>
#include <cstdint>
#include <cfloat>
#include <cmath>
#include <stdexcept>
#include "../include/rtk.h"

namespace sensor {                    //命名空间
	static const float MIN_POINT_DISTANCE = 0.5f;  //最小点距离
	static const double EPSILON = 1E-12;
	static const double TIMEOUT = 0.5;
	using namespace std::chrono_literals;
	Matrix3d LatLon::earthToMapMatrix() const {
		return sensor::earthToMapMatrix(earth());                     //地球坐标系转map
	}

	Matrix3d LatLon::earthToLocalMatrix(double azimuth) const {
		return azimuthMatrix(azimuth).transpose() * earthToMapMatrix();                        //地球坐标系转局部坐标系
	}

	Matrix3d azimuthMatrix(double degree) {
		return AngleAxisd(-degree * PI / 180.0, Vector3d(0, 0, 1)).toRotationMatrix();                 //方位角矩阵
	}

	Matrix3d earthToMapMatrix(Vector3d earth) {                                //地球坐标系转map（东北天坐标系）
		Vector3d earthZ(0, 0, 1);
		Vector3d up(earth.x(), earth.y(), earth.z());
		Vector3d east = earthZ.cross(up).normalized();
		Vector3d north = up.cross(east).normalized();
		Matrix3d ret;
		up.normalize();
		ret << east.x(), east.y(), east.z(),
			north.x(), north.y(), north.z(),
			up.x(), up.y(), up.z();
		return ret;
	}

	Matrix3d earthToLocalMatrix(const Vector3d & earth, double azimuth) {                                      //地球坐标系转局部坐标系=方位角矩阵  *   地球坐标系转map
		return azimuthMatrix(azimuth).transpose() * earthToMapMatrix(earth);        
	}

	boost::optional<LatLonAzimuth> M39B::current() {                                               //时间
		std::lock_guard<std::mutex> lock(_mutex);
		auto now = std::chrono::steady_clock::now();                  
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - _updateTime).count();      //持续时间duration cast
		//std::cout << "elapsed: " << elapsed << std::endl;
		if (elapsed > 500)
			return boost::none;
		return _current;
	}

    template <typename SyncReadStream, typename MutableBufferSequence>  //同步读取流
    void readWithTimeout(SyncReadStream& s, const MutableBufferSequence& buffers)  //可变缓冲序列
    {   /////////////io复用，同时发送并监听处理很多socket或者文件读写的事件
        boost::optional<boost::system::error_code> timer_result;
        boost::asio::deadline_timer timer(s.get_io_service());        	//获取io服务
        timer.expires_from_now(boost::posix_time::seconds(1));        //从现在起过期？
        timer.async_wait([&timer_result] (const boost::system::error_code& error) { timer_result.reset(error); });       //异步等待

        boost::optional<boost::system::error_code> read_result;
        boost::asio::async_read(s, buffers, boost::asio::transfer_at_least(boost::asio::buffer_size(buffers)),[&read_result] (const boost::system::error_code& error, size_t) { read_result.reset(error); });  //异步读取

        s.get_io_service().reset();
        while (s.get_io_service().run_one())        //若是获取到了io的服务，那么读取结果或者直接删除为空
        {
            if (read_result)
                timer.cancel();
            else if (timer_result)
                s.cancel();
        }
        if (*read_result) {
            throw boost::system::system_error(*read_result);
        }
    }
	
	void M39B::readNext(boost::asio::ip::tcp::socket & sock) {
		unsigned char c = 0;
		for (;;) {
			readWithTimeout(sock, boost::asio::buffer(&c, 1));
			if (0xaa != c) {
				if (printLog)
					std::cout << "M39B data: " << (int)c << std::endl;
				continue;
			}
			readWithTimeout(sock, boost::asio::buffer(&c, 1));
			if (0x55 != c) {
				continue;
			}
			printLog = false;
			sensor::M39BStruct p;
			readWithTimeout(sock, boost::asio::buffer(&p, sizeof(p)));
			//std::cout << "M39B status: " << (int)p.status << std::endl;
			//std::cout << p.lat << " " << p.lon << " " << p.head << std::endl;
			if ((p.status & 0x7) != 2) {
				std::cout << "RTK status: " << (int)(p.status & 0x7) << std::endl;
			}
			std::uint32_t azimuthBuff = p.head;
			double azimuth = *reinterpret_cast<float*>(&azimuthBuff);  //reinterpret重新诠释
			//double azimuth = p.head * 1E-3 + 4;
			//if (azimuth > 360)
			//	azimuth -= 360;
			std::uint64_t lat = p.lat, lon = p.lon;
			LatLonAzimuth lla(*reinterpret_cast<double*>(&lat), *reinterpret_cast<double*>(&lon), azimuth);
			//LatLonAzimuth lla(p.lat * 1E-7, p.lon * 1E-7, azimuth);
			setCurrent(lla);
			break;
		}
	}

	void M39B::readLoop() {  //读取回环
		if(!_sim) { return; }
		for (;;){
		    try {
		        boost::asio::ip::tcp::socket sock(_io);
                boost::asio::ip::tcp::endpoint server(boost::asio::ip::address::from_string(_ip), 8887);
                sock.connect(server);
                for(;;) {
                    readNext(sock);
                }
		    }
		    catch(const std::exception & e) {
		        // std::cerr << "M39B::readLoop: " << e.what() << std::endl;
		        std::this_thread::sleep_for(1s);
		    }
		}
	}

    template <typename SyncReadStream>
    std::string readLineWithTimeout(SyncReadStream& s, boost::asio::streambuf& b)        //读取超时行
    {
        boost::optional<boost::system::error_code> timer_result;
        boost::asio::deadline_timer timer(s.get_io_service());
        timer.expires_from_now(boost::posix_time::seconds(1));
        timer.async_wait([&timer_result] (const boost::system::error_code& error) { timer_result.reset(error); });

        boost::optional<boost::system::error_code> read_result;
        boost::asio::async_read_until(s, b, '\n',[&read_result] (const boost::system::error_code& error, size_t) { read_result.reset(error); });

        s.get_io_service().reset();                          //获取io服务
		
        while (s.get_io_service().run_one())
        {
            if (read_result)
                timer.cancel();
            else if (timer_result)
                s.cancel();
        }

        if (*read_result) {
            throw boost::system::system_error(*read_result);
        }
        std::istream is(&b);
        std::string line;
        std::getline(is, line);
        return std::move(line);
    }

    // static const std::string_view HEADER = "#INSPVAXA,";
    static void throwIfNpos(std::string_view::size_type pos, const char * what) {
        if(pos == std::string_view::npos)
            throw std::logic_error(what);
    }
	
    static std::string_view nextField(const std::string_view & view, std::string_view::size_type & pos) {
        std::string_view::size_type end = view.find(',', pos);
        throwIfNpos(end, "end of line when parse field");
        std::string_view ret = view.substr(pos, end - pos);
        pos = end + 1;
        return ret;
    }

	static void loadLatLonAzimuthFile(const std::string &filename, std::vector<LatLonAzimuth> &map) {         //加载方位角文件
		std::ifstream in(filename);
		if (!in) {
			throw std::invalid_argument("can't open file: " + filename);
		}
		double lat, lon, head;
		while (in >> lat >> lon >> head) {
			LatLonAzimuth lla(lat, lon, head);
			map.push_back(LatLonAzimuth(lat, lon, head));
		}
	}

	static std::vector<LatLonAzimuth>::const_iterator nextPoint(std::vector<LatLonAzimuth>::const_iterator current, std::vector<LatLonAzimuth> & points) { //nextPoint关系
		for (auto iter = current; iter != points.end(); iter++) {
			Vector3d diff = iter->latLon().earth() - current->latLon().earth();
			if (diff.norm() > MIN_POINT_DISTANCE) {
				return iter;
			}
		}
		return points.end();
	}

	static void loadPoints(const std::string & filename, std::vector<LatLonAzimuth> & points) {                   //loadPoints
		std::vector<LatLonAzimuth> raw;
		loadLatLonAzimuthFile(filename, raw);
		auto current = raw.cbegin();
		while (current != raw.cend()) {
			points.push_back(*current);
			current = nextPoint(current, raw);
		}
	}

	static float curvature(const LatLonAzimuth & p, const LatLonAzimuth & prev) {                //曲率
		float da = static_cast<float>(std::abs(p.azimuth() - prev.azimuth()));
		if (da > 180)
			da = 360 - da;
		float rad_da = da * static_cast<float>(PI) / 180.0f;
		float length = static_cast<float>((prev.latLon().earth() - p.latLon().earth()).norm());
		return rad_da / length;
	}

	static PathPoint toPathPoint(std::vector<LatLonAzimuth>::iterator iter, int index) {                //toPathPoint
		
		const Vector3d & p = iter->latLon().earth();
		const Vector3d & np = (iter + 1)->latLon().earth();
		float length = static_cast<float>((np - p).norm());

		const Matrix3d l2e = iter->earthToLocalMatrix().inverse();
		const Vector3f tangent = (l2e * Vector3d(0, 1, 0)).cast<float>();
		const Vector3f normal = (l2e * Vector3d(1, 0, 0)).cast<float>();

		const Matrix3d prev_l2e = (iter - 1)->earthToLocalMatrix().inverse();
		const Vector3f prev_tangent = (l2e * Vector3d(0, 1, 0)).cast<float>();
		float c = curvature(*iter, *(iter - 1));

		return PathPoint(index, p, tangent, normal, length, c);
	}

	void Path::loadPath(const std::string & path) {                          //加载路径
		auto filename = path + ".path";
		// std::vector<LatLonAzimuth> v3dpoints;
		loadPoints(filename, _v3dpoints);
		if (_v3dpoints.size() < 3)
			throw std::domain_error("path too short: " + filename);
		for (auto iter = _v3dpoints.begin() + 1; iter != _v3dpoints.end() - 1; iter++) {
			_points.push_back(toPathPoint(iter, static_cast<int>(_points.size())));
		}
		/*
		for (const auto & pp : _points) {
			std::cout << pp.length() << ", " << pp.curvature() << std::endl;

		}
		*/
	}

	void Path::loadStopMarks(const std::string & path) {              //加载停止标志
		auto filename = path + ".stopmark";
		std::vector<LatLonAzimuth> buffer;
		loadLatLonAzimuthFile(filename, buffer);
		for (const auto & i : buffer) {
			_stopMarks.push_back(i.latLon().earth());
		}
	}

	void Path::loadStations(const std::string & path) {                     //loadStations
		auto filename = path + ".station";
		std::vector<LatLonAzimuth> buffer;
		loadLatLonAzimuthFile(filename, buffer);
		for (const auto & i : buffer) {
			_stations.push_back(i.latLon().earth());
		}
	}

	Path::Path(const std::string& path, bool sim, float speed):_sim(sim),_simSpeed(speed),_current(boost::none)/*,_thread(&Path::readLoop, this)*/
	{
		loadPath(path);
		loadStopMarks(path);
		loadStations(path);
	}

	// void Path::readLoop(){
	// 	while(true){
	// 		std::this_thread::sleep_for(100ms);
	// 	}
	// }

	boost::optional<LatLonAzimuth> Path::current() {
		static std::vector<LatLonAzimuth>::const_iterator it;   //当前方位角随着时间增加
		static int count = 0;
		if(!_current){
			static int count = 0;
			it = _v3dpoints.cbegin();
			count++;
			_updateTime = std::chrono::steady_clock::now();
			_current = *it;
			// std::cout << "_latLon._lat：" << (*_current).latLon().lat() << std::endl;
			// std::cout << "_latLon._lon：" << (*_current).latLon().lon() << std::endl;
		}else{
			auto now = std::chrono::steady_clock::now();
			auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - _updateTime).count(); //转换为ms
			float totallen = _simSpeed / 1000.0  * elapsed; 
			it = nearestLatLonAzimuth(it, totallen);                                      //如果不是从0开始，则随着最近方位角增加
			_current = *it;
			_updateTime = now; //std::chrono::steady_clock::now();
			//const LatLonAzimuth &  _current_pre = const_cast<LatLonAzimuth> _current;
			//setCurrent(const_cast<LatLonAzimuth>& _current);
		}
		//if (elapsed > 500)
		//	return boost::none;
		return _current;
	}
	
	static std::vector<PathPoint>::const_iterator last(const std::vector<PathPoint> & points, float forward) {         //const_iterator构造器
		float totallen = 0;
		for (auto iter = points.end() - 1; iter != points.begin(); iter--) {
			totallen += iter->length();
			if (totallen >= forward)
				return iter;
		}
		return points.begin();
	}

	static std::vector<PathPoint>::const_iterator forwardPoint(std::vector<PathPoint>::const_iterator iter, std::vector<PathPoint>::const_iterator end, float forward) {      //forwardPoint
		if (0.0f == forward)
			return iter;
		float totallen = 0;
		for (; iter != end; iter++) {
			totallen += iter->length();
			if (totallen >= forward) {
				return iter;
			}
		}
		return end;
	}

	std::vector<LatLonAzimuth>::const_iterator Path::nearestLatLonAzimuth(std::vector<LatLonAzimuth>::const_iterator& it,float forward) const{  //nearestLatLonAzimuth
		double lat1; //纬
		double lon1; //经
		double lat2;
		double lon2;

		float totallen = 0;
		int count = 0;
        float R = 6371.0; //km

		while (totallen < forward) {
			if (it == (_v3dpoints.cend()-1)){
				it = _v3dpoints.cbegin();
				lat1 = (*it).latLon().lat();
				lon1 = (*it).latLon().lon();
				it++;
				lat2 = (*it).latLon().lat();
				lon2 = (*it).latLon().lon();
			}else{
				lat1 = (*it).latLon().lat();
				lon1 = (*it).latLon().lon();
				it++;
				lat2 = (*it).latLon().lat();
				lon2 = (*it).latLon().lon();
			}
		    double d = R*acos(cos(lat1)*cos(lat2)*cos(lon1-lon2)+sin(lat1)*sin(lat2));           //可通过经维度解算出地球上两点的球面距离
			totallen += d;
			count++;
		}
		return it;
	}

	const PathPoint & Path::nearest(const LatLonAzimuth & lla, float forward) const {                   //看不懂
		auto lastIter = last(_points, forward);
		Vector3d curr = lla.latLon().earth();
		double min = DBL_MAX;
		Matrix3f e2l = lla.earthToLocalMatrix().cast<float>();
		std::vector<PathPoint>::const_iterator nearestIter = _points.cbegin();
		for (auto iter = _points.cbegin(); iter != lastIter + 1; iter++) {
            Vector3f localTangent = e2l * iter->tangent();
            if(Vector3f::UnitY().dot(localTangent) < 0)
                continue;
			double dist = (iter->point() - curr).norm();
			if (dist < min) {
				nearestIter = iter;
				min = dist;
			}
		}
		return *forwardPoint(nearestIter, _points.cend(), forward);
	}

	float Path::maxSpeed(const PathPoint & point, float speedLimit) const {                //最大速度与最西哦速度
		float minSpeed = speedLimit;
		float distance = 0;
		for (auto iter = _points.cbegin() + point.index(); iter != _points.cend(); iter++) { // cbegin                         index为索引函数                cend
			float c = iter->curvature();                       //曲率
			float speed = distance / 3 + 1 / c;
			//std::cout << "d: " << distance << " speed: " << speed << " 1/c: " << 1/c << std::endl;
			if (speed < minSpeed)
				minSpeed = speed;
			distance += iter->length();
			if (distance > 100)
				break;
		}
		return minSpeed;
	}

    float Path::maxCurvature(const PathPoint & point, float forward) const {                           //最大曲率
        float max = 0;
        float distance = 0;
        for (auto iter = _points.cbegin() + point.index(); iter != _points.cend(); iter++) {
            if(max < iter->curvature())
                max = iter->curvature();
            distance += iter->length();
            if (distance > forward)
                break;
        }
        return max;
	}

	boost::optional<float> Path::nextStopMark(const LatLonAzimuth & lla) const {          //停止标志
		float min = FLT_MAX;
		boost::optional<float> ret = boost::none;
		Matrix3d e2l = lla.earthToLocalMatrix();
		for (Vector3d stopMark : _stopMarks) {
			Vector3f local3 = (e2l * stopMark).cast<float>();
			Vector2f local(local3.x(), local3.y());
			if (local.y() < -10)
				continue;
			if (local.x() < -2 || local.x() > 2)
				continue;
			if (local.y() < min) {
				min = local.y();
				ret = local.y();
			}
		}
		return ret;
	}

	boost::optional<float> Path::stationDistance(const LatLonAzimuth & lla, int stationIndex) const         //站距
	{
		float min = FLT_MAX;
		boost::optional<float> ret = boost::none;
		if (stationIndex >= _stations.size())
			return ret;
		Matrix3d e2l = lla.earthToLocalMatrix();
		Vector3d station = _stations[stationIndex];
		Vector3f local3 = (e2l * station).cast<float>();
		Vector2f local(local3.x(), local3.y());
		if (local.y() < -10)
			return ret;
		if (local.x() < -2 || local.x() > 5)
			return ret;
		return local.y();
	}

    Path::Directions Path::nearestDirections(const PathPoint & point, float forward) {           //最近的方向
        float distance = 0;
        for (auto iter = _points.cbegin() + point.index(); iter != _points.cend(); iter++) {
            distance += iter->length();
            if (distance > forward)
                break;
            if(iter == _points.cbegin())
                continue;
            if(iter->curvature() < 1.0f/50.0f)
                continue;
            auto prev = iter -1;
            const Eigen::Vector3f & pt = prev->tangent();
            const Eigen::Vector3f & t = iter->tangent();
            const Eigen::Vector3f up = iter->point().normalized().cast<float>();
            if(pt.cross(t).dot(up) > 0)
                return TURN_LEFT;
            else
                return TURN_RIGHT;
        }
        return GO_STRAIGHT;
    }

    float StationBehavior::maxSpeed(const LatLonAzimuth & lla, float speedLimit) {              //基站状态
		switch (_status) {
		case ON_THE_WAY:
			return statusOnTheWay(lla, speedLimit);
		case STOP:
			return statusStop(lla, speedLimit);
		default:
			throw std::runtime_error("invalid status");
		}
	}


	float StationBehavior::statusOnTheWay(const LatLonAzimuth & lla, float speedLimit) {                   //在路上时候的状态
		auto t = target();
		if (!t)
			return speedLimit;
		boost::optional<float> dist = _path.stationDistance(lla, *t);
		if (!dist)
			return speedLimit;
		if (*dist > speedLimit)
			return speedLimit;
		/*
		if (*dist <= 0.0f) {
			_status = STOP;
			if (_onStop)
				_onStop();
			return 0.0f;
		}
		*/
		return std::max(0.0f, *dist + 2);
	}

	float StationBehavior::statusStop(const LatLonAzimuth & lla, float speedLimit) {             // 状态停止
		return 0.0f;
	}
}
