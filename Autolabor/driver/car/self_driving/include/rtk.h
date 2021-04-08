#pragma once
#include <boost/endian/arithmetic.hpp>
#include <boost/asio.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/optional.hpp>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <iostream>
#include <functional>
#include <chrono>


namespace sensor {
	using namespace boost::endian;
	using namespace Eigen;
	/*
	result  = struct.unpack('<2B 2B1H1I1B8i1I6h1B',bytearray(res[2:]))
	*/
	/*
		big_uint8_t hdr1; //0xaa
		big_uint8_t hdr2; //0x55
	*/
	struct M39BStruct {
		little_uint8_t length;
		little_uint8_t mode;
		little_int16_t time1;
		little_int32_t time2;
		little_uint8_t num;
		little_uint64_t lat;
		little_uint64_t lon;
		little_int32_t height;
		little_int32_t v_n;
		little_int32_t v_e;
		little_int32_t v_earth;
		little_int32_t roll;
		little_int32_t pitch;
		little_uint32_t head;
		little_int16_t a_n;
		little_int16_t a_e;
		little_int16_t a_earth;
		little_int16_t v_roll;
		little_int16_t v_pitch;
		little_int16_t v_head;
		little_uint8_t status;
	};

	const double EARTH_RADIUS = 6371008.8;
	const double PI = 3.14159265358979323846;

	class LatLon {
	public:
		double _lat, _lon;
	public:
		LatLon(double lat, double lon) :_lat(lat), _lon(lon) {
		};
		LatLon() :LatLon(0, 0) {}
		LatLon(const LatLon & other) : _lat(other._lat), _lon(other._lon) {}
	public:
		LatLon & operator=(const LatLon & other) {
			_lat = other._lat;
			_lon = other._lon;
			return *this;
		}
	public:
		double lat() const { return _lat; };
		double lon() const { return _lon; };
		double x() const {
			return EARTH_RADIUS * std::cos(_lat * PI / 180.0) * std::cos(_lon * PI / 180.0);
		}
		double y() const {
			return EARTH_RADIUS * std::cos(_lat * PI / 180.0) * std::sin(_lon * PI / 180.0);
		}
		double z() const {
			return EARTH_RADIUS * std::sin(_lat * PI / 180.0);
		}
		Vector3d earth() const {
			return Vector3d(x(), y(), z());
		}
		Matrix3d earthToMapMatrix() const;
		Matrix3d earthToLocalMatrix(double azimuth) const;
	};

	class LatLonAzimuth {
	public:
		LatLon _latLon;
		double _azimuth;
	public:
		LatLonAzimuth(double lat, double lon, double azimuth) :_latLon(lat, lon), _azimuth(azimuth) {}
		LatLonAzimuth() :LatLonAzimuth(0, 0, 0) {}
		LatLonAzimuth(const LatLonAzimuth & other) : _latLon(other._latLon), _azimuth(other._azimuth) {
		}
	public:
		LatLonAzimuth & operator=(const LatLonAzimuth & other) {
			_latLon = other._latLon;
			_azimuth = other._azimuth;
			return *this;
		}
	public:
		const LatLon & latLon() const { return _latLon; }
		double azimuth() const { return _azimuth; }
		Matrix3d earthToLocalMatrix() const { return _latLon.earthToLocalMatrix(_azimuth); }
		Affine3d earthToLocalAffine() const {
		    Affine3d affine3D;
            affine3D.linear() = this->earthToLocalMatrix().inverse();
            affine3D.translation() = this->latLon().earth();
            affine3D = affine3D.inverse();
            return affine3D;
		}
	};

	Matrix3d azimuthMatrix(double degree);
	Matrix3d earthToMapMatrix(Vector3d earth);
	Matrix3d earthToLocalMatrix(const Vector3d & earth, double azimuth);

	class PathPoint {
	private:
		int _index;
		Vector3d _point;
		Vector3f _tangent;
		Vector3f _normal;
		float _length;
		float _curvature;
	public:
		PathPoint(int index, const Vector3d & point, const  Vector3f & tangent, const Vector3f & normal, float length, float curvature)
			:_index(index),_point(point), _tangent(tangent), _normal(normal) , _length(length), _curvature(curvature) {}
	public:
		int index() const { return _index; }
		const Vector3d & point() const { return _point; }
		const Vector3f & tangent() const { return _tangent; }
		const Vector3f & normal() const { return _normal; }
		float length() const { return _length; }
		float curvature() const { return _curvature; }
        Affine3d earthToLocalAffine() const {
		    Matrix3d l2e;
		    l2e.col(0) = this->normal().cast<double>();
            l2e.col(1) = this->tangent().cast<double>();
            l2e.col(2) = this->point().normalized();
            Affine3d affine3D;
            affine3D.linear() = l2e;
            affine3D.translation() = this->point();
            affine3D = affine3D.inverse();
            return affine3D;
        }
	};

	class Path {
	private:
		std::vector<PathPoint> _points;
		std::vector<Vector3d> _stopMarks;
		std::vector<Vector3d> _stations;
		void loadPath(const std::string & path);
		void loadStopMarks(const std::string & path);
		void loadStations(const std::string & path);
		std::thread _thread;
	public:
		Path() = default;
		explicit Path(const std::string & path, bool sim , float speed);
	public:                           
	    enum Directions {
            GO_STRAIGHT,
	        TURN_LEFT,
	        TURN_RIGHT
	    };
	    const std::vector<PathPoint> & points() const {
	        return _points;
	    }

		const PathPoint & nearest(const LatLonAzimuth & lla, float forward) const;
		std::vector<LatLonAzimuth>::const_iterator nearestLatLonAzimuth(std::vector<LatLonAzimuth>::const_iterator& it,float forward) const;
	    Directions nearestDirections(const PathPoint & point, float forward);
		float maxSpeed(const PathPoint & point, float speedLimit) const;
        float maxCurvature(const PathPoint & point, float forward) const;
		boost::optional<float> nextStopMark(const LatLonAzimuth & lla) const;
		boost::optional<float> stationDistance(const LatLonAzimuth & lla, int station) const;
	private:
		bool _sim;
		float _simSpeed;
		std::mutex _mutex;
		boost::optional<LatLonAzimuth> _current;
		std::chrono::steady_clock::time_point _updateTime;
		std::vector<LatLonAzimuth> _v3dpoints;
		
		//void readNext();
		//void readLoop(); 
	public:
		bool sim(){ return _sim; }
		float simSpeed() { return _simSpeed; }
		// void setCurrent(const LatLonAzimuth & latLonAzimuth) {
		// 	// std::lock_guard<std::mutex> lock( _mutex );
		// 	_updateTime = std::chrono::steady_clock::now();
		// 	_current = latLonAzimuth;
		// }
		boost::optional<LatLonAzimuth> current();
	};

	class StationBehavior {
	public:
		enum Status {ON_THE_WAY, STOP};
	private:
		const Path & _path;
		Status _status;
		std::function<void()> _onStop;
		boost::optional<int> _target{ boost::none };
	public:
		StationBehavior(const Path & path) :_path(path), _status(ON_THE_WAY) {
		}
	public:
		void target(int station) { _target = station; }
		boost::optional<int> target() { return _target; }
		Status status() { return _status; }
		float maxSpeed(const LatLonAzimuth & lla, float speedLimit);
		void onStop(std::function<void()> f) { _onStop = f; }
	private:
		float statusOnTheWay(const LatLonAzimuth & lla, float speedLimit);
		float statusStop(const LatLonAzimuth & lla, float speedLimit);
	};

	class M39B { //rtk型号
	private:
		std::string _ip;
		boost::asio::io_service _io;
		std::mutex _mutex;
		boost::optional<LatLonAzimuth> _current;
		std::chrono::steady_clock::time_point _updateTime;
		std::thread _thread;
		bool printLog = true;
		bool _sim;
	private:
		void setCurrent(const LatLonAzimuth & latLonAzimuth) {
			std::lock_guard<std::mutex> lock(_mutex);
			_updateTime = std::chrono::steady_clock::now();
			_current = latLonAzimuth;
		}
		void readNext(boost::asio::ip::tcp::socket & sock);
		void readLoop();
	public:
		explicit M39B(const std::string & ip,bool sim) :_ip(ip), _current(boost::none), _thread(&M39B::readLoop, this) ,_sim(sim) {}
	public:
		boost::optional<LatLonAzimuth> current();
	};
}
