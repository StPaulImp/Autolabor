#pragma once

#include <boost/endian/arithmetic.hpp>
#include <boost/asio.hpp>
#include <boost/optional.hpp>
#include <boost/core/noncopyable.hpp>
#include <boost/math/constants/constants.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#include <array>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <memory>
#include <stdexcept>
#include <chrono>
#include <functional>
#include <Eigen/Geometry>
/*
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
 */
#include <boost/progress.hpp>

namespace sensor {

    using namespace boost::endian;
    using namespace boost::asio;
    using namespace std::chrono_literals;
    //定义支持类型 有效频道数据
    //VLP16
    struct VLP16ChannelData {
        little_uint16_t distance;
        little_uint8_t reflectivity;
    };


    struct VLP16DataBlock {
        little_uint16_t flag;
        little_uint16_t azimuth;
        VLP16ChannelData channels[2][16];
    };


    struct VLP16DataPacket {
        //little_uint8_t header[42];
        VLP16DataBlock dataBlocks[12];
        little_uint32_t timestamp;
        little_uint16_t factory;
    };

    //RS16
    struct RS16ChannelData {
        big_uint16_t distance;
        big_uint8_t reflectivity;
    };

    struct RS16DataBlock {
        big_uint16_t flag;
        big_uint16_t azimuth;
        RS16ChannelData channels[2][16];
    };


    struct RS16DataPacket {
        big_uint8_t header[42];
        RS16DataBlock dataBlocks[12];
        big_uint32_t timestamp;
        big_uint16_t factory;
    };

    //RS32
    struct RS32ChannelData {
        big_uint16_t distance;
        big_uint8_t reflectivity;
    };

    struct RS32DataBlock {
        big_uint16_t flag;
        big_uint16_t azimuth;
        RS32ChannelData channels[32];
    };


    struct RS32DataPacket {
        big_uint8_t header[42];
        RS32DataBlock dataBlocks[12];
        big_uint32_t timestamp;
        big_uint16_t factory;
    };

    //LSC16
    struct LSC16ChannelData {
        little_uint16_t distance;
        little_uint8_t reflectivity;
    };


    struct LSC16DataBlock {
        little_uint16_t flag;
        little_uint16_t azimuth;
        LSC16ChannelData channels[2][16];
    };


    struct LSC16DataPacket {
        //little_uint8_t header[42];
        LSC16DataBlock dataBlocks[12];
        little_uint32_t timestamp;
        little_uint16_t factory;
    };

    //Lidar 16 Model
    struct VLP16 {
        typedef VLP16DataPacket PacketType;
        typedef VLP16DataBlock BlockType;
        typedef VLP16ChannelData ChannelType;
        static const int blockFlag = 0xeeff;
        static const float distanceUnit;
        static const float verticalAngles[16];
        static const int channelVerticalAngleIndices[16];

        static bool isValidChannelData(const ChannelType &channelData) {
            return channelData.distance > 0;

        }
    };

    ////////////////////////////////////////////结构  长度和 维度
    struct RS16 {
        typedef RS16DataPacket PacketType;
        typedef RS16DataBlock BlockType;
        typedef RS16ChannelData ChannelType;
        static const int blockFlag = 0xffee;
        static const float distanceUnit;
        static const float verticalAngles[16];
        static const int channelVerticalAngleIndices[16];

        static bool isValidChannelData(const ChannelType &channelData) {
            return channelData.distance <= 15000;

        }
    };

    struct RS32 {
        typedef RS32DataPacket PacketType;
        typedef RS32DataBlock BlockType;
        typedef RS32ChannelData ChannelType;
        static const int blockFlag = 0xffee;
        static const float distanceUnit;
        static const float verticalAngles[32][2];
        static const int channelVerticalAngleIndices[32];

        static bool isValidChannelData(const ChannelType &channelData) {
            return channelData.distance <= 40000;

        }
    };

    struct LSC16 {
        typedef LSC16DataPacket PacketType;
        typedef LSC16DataBlock BlockType;
        typedef LSC16ChannelData ChannelType;
        static const int blockFlag = 0xeeff;
        static const float distanceUnit;
        static const float verticalAngles[16];
        static const int channelVerticalAngleIndices[16];

        static bool isValidChannelData(const ChannelType &channelData) {
            return channelData.distance > 0;
        }
    };

    //Pcap file  该包的存储
    struct PcapGlobalHeader {
        little_uint32_t magicNumber;   /* magic number */
        little_uint16_t versionMajor;  /* major version number 主要版本号*/
        little_uint16_t versionMinor;  /* minor version number次要版本号 */
        little_int32_t thiszone;       /* GMT to local correction 格林尼治标准时间进行本地更正*/
        little_uint32_t sigfigs;        /* accuracy of timestamps 时间戳的准确性*/
        little_uint32_t snaplen;        /* max length of captured packets, in octets捕获的数据包的最大长度，以八位字节为单位 */
        little_uint32_t network;        /* data link type数据链接类型 */
    };

    struct PcapPacketHeader {
        little_uint32_t tsSec;         /* timestamp seconds 时间戳秒*/
        little_uint32_t tsUsec;        /* timestamp microseconds 时间戳微秒*/
        little_uint32_t inclLen;       /* number of octets of packet saved in file文件中保存的数据包的八位字节数 */
        little_uint32_t origLen;       /* actual length of packet 包的实际长度*/
    };

    //Lidar Commmon激光雷达
    const float deg2rad = boost::math::constants::degree<float>();

    class LidarPoint {
    private:
        float _distance;  //距离
        float _azimuth;  //方位角
        float _verticalAngle;//垂直角
        float _x, _y, _z;//xyz
    public:
        float x() const { return _x; }

        float y() const { return _y; }

        float z() const { return _z; }

        float distance() const { return _distance; }

        float azimuth() const { return _azimuth; }

        float verticalAngle() const { return _verticalAngle; }

    public:
        LidarPoint(float distance, float azimuth, float verticalAngle)
                : _distance(distance),
                  _azimuth(azimuth),
                  _verticalAngle(verticalAngle),
                  _x(_distance * std::cos(_verticalAngle * deg2rad) * std::sin(_azimuth * deg2rad)),  //角度还弧度----距离通过角度转为x
                  _y(_distance * std::cos(_verticalAngle * deg2rad) * std::cos(_azimuth * deg2rad)),  //角度还弧度----距离通过角度转为y
                  _z(_distance * std::sin(_verticalAngle * deg2rad)) {}
    };

    template<typename Model, typename Derived, typename Spatial> //型号  派生的  空间的
    class Lidar {
    public:
        using SpatialType = Spatial;
        using PointCloudType = std::shared_ptr<std::vector<std::vector<Eigen::Vector3f>>>;
    private:
        io_service _ios;
        ip::udp::endpoint _server; //服务器
        std::thread _readThread; //读线程
        std::function<Spatial()> _spatialProvider;//空间提供者
        std::atomic_bool _exit{false};
        std::mutex _frameBufferMutex;//帧缓冲互斥
        std::condition_variable _frameBufferCondVar;  //帧缓冲条件变量
        std::chrono::steady_clock::time_point _updateTime;
        Spatial _spatialInfo;  //空间信息
        std::shared_ptr<std::vector<typename Model::BlockType>> _frameBufferPtr; //帧缓冲区Ptr

        void doRead();  //读取

    private:
        std::thread _frameToPointCloudThread; //框架到点云线程
        std::mutex _frameToPointCloudMutex;
        std::chrono::steady_clock::time_point _currentUpdateTime;
        Spatial _currentSpatialInfo;
        boost::optional<PointCloudType> _currentPointCloud {boost::none};

        void doFrameToPointCloud() {    //坐标系到点云
            while (!_exit.load()) {
                std::chrono::steady_clock::time_point ts;
                Spatial s;
                auto pc = static_cast<Derived *>(this)->frameToPointCloud(ts, s);
                std::lock_guard<std::mutex> lock(_frameToPointCloudMutex);
                _currentPointCloud = pc;
                _currentUpdateTime = ts;
                _currentSpatialInfo = s;
            }
        }


    protected:
        ip::udp::socket _sock;  //ip
        Eigen::Affine3f _transform;

        void setFrameBuffer(std::shared_ptr<std::vector<typename Model::BlockType>> ptr);

        std::shared_ptr<std::vector<typename Model::BlockType>>
        getFrameBuffer(std::chrono::steady_clock::time_point &timestamp, Spatial &spatialInfo);

    public:
        explicit Lidar(unsigned short port, const Eigen::Affine3f & transform);

        ~Lidar();

        void setSpatialInfoProvider(std::function<Spatial()> provider) {
            _spatialProvider = provider;

        }
        boost::optional<PointCloudType> current(std::chrono::steady_clock::time_point &timestamp, Spatial &spatialInfo) {
            std::lock_guard<std::mutex> lock(_frameToPointCloudMutex);
            std::chrono::duration<float> duration = std::chrono::steady_clock::now() - _currentUpdateTime;
            if(duration.count() > 2)
                return boost::none;
            timestamp = _currentUpdateTime;
            spatialInfo = _currentSpatialInfo;
            return _currentPointCloud;
        }
    };

    template<typename Model, typename Derived, typename Spatial>
    Lidar<Model, Derived, Spatial>::Lidar(unsigned short port, const Eigen::Affine3f & transform):_server(ip::udp::v4(), port),
                                                                                    _sock(_ios, _server),
                                                                                    _transform(transform),
                                                                                    _frameBufferPtr(nullptr),
                                                                                    _updateTime(std::chrono::steady_clock::now()){
        this->_readThread = std::thread(&Lidar<Model, Derived, Spatial>::doRead, this);
        this->_frameToPointCloudThread = std::thread(&Lidar<Model, Derived, Spatial>::doFrameToPointCloud, this);
    }

    template<typename Model, typename Derived, typename Spatial>
    Lidar<Model, Derived, Spatial>::~Lidar() {
        _exit = true;
        this->_frameToPointCloudThread.join();
        _readThread.join();
    }

    template<typename Model, typename Derived, typename Spatial>
    void Lidar<Model, Derived, Spatial>::doRead() {                //读取
        while (!_exit.load())
            static_cast<Derived *>(this)->readFrame();
    }

    template<typename Model, typename Derived, typename Spatial>
    void
    Lidar<Model, Derived, Spatial>::setFrameBuffer(std::shared_ptr<std::vector<typename Model::BlockType>> ptr) {         //设置坐标
        {
            std::lock_guard<std::mutex> lock(this->_frameBufferMutex);
            _updateTime = std::chrono::steady_clock::now();
            if (this->_spatialProvider)
                this->_spatialInfo = this->_spatialProvider();
            this->_frameBufferPtr = ptr;
        }
        this->_frameBufferCondVar.notify_one();
    }

    template<typename Model, typename Derived, typename Spatial>
    std::shared_ptr<std::vector<typename Model::BlockType>>
    Lidar<Model, Derived, Spatial>::getFrameBuffer(std::chrono::steady_clock::time_point &timestamp,   //获取坐标
                                                   Spatial &spatialInfo) {
        std::unique_lock<std::mutex> lock(this->_frameBufferMutex);
        auto updateTime = _updateTime;
        this->_frameBufferCondVar.wait(lock, [&updateTime, this]() { return this->_updateTime > updateTime; });
        timestamp = _updateTime;
        spatialInfo = _spatialInfo;
        return this->_frameBufferPtr;
    }

    template<typename Model, typename Spatial>
    class Lidar16 : public Lidar<Model, Lidar16<Model, Spatial>, Spatial> {
    public:
        explicit Lidar16(unsigned short port, const Eigen::Affine3f & transform)
                : Lidar<Model, Lidar16<Model, Spatial>, Spatial>(port, transform) {
        }
    public:
        void readFrame();

        typename Lidar<Model, Lidar16<Model, Spatial>, Spatial>::PointCloudType frameToPointCloud(std::chrono::steady_clock::time_point &timestamp, Spatial &spatialInfo);

    };

    template<typename Model, typename Spatial>
    void Lidar16<Model, Spatial>::readFrame() {                              //读取坐标，方位角变化过程
        auto frameBuffer = std::make_shared<std::vector<typename Model::BlockType>>();
        float azimuthAcc = 0;
        float prevAzimuth;
        bool first = true;
        for (;;) {
            typename Model::PacketType packet;
            boost::system::error_code ec;
            ip::udp::endpoint remote;
            size_t n = this->_sock.receive_from(buffer(&packet, sizeof(packet)), remote, 0, ec);
            if (ec) {
                std::cerr << "error: " << ec << std::endl;
                continue;
            }
            if (first) {
                first = false;
                prevAzimuth = packet.dataBlocks[0].azimuth * 0.01f;
            }
            for (auto &db : packet.dataBlocks) {
                if (db.flag != Model::blockFlag) {
                    std::cerr << "bad packet" << std::endl;
                    break;
                }

                /*
                for (const auto & c : db.channels[0]) {
                    std::cout << "distance: " << c.distance << std::endl;
                    std::cout << "reflectivity: " << (int)c.reflectivity << std::endl;

                }
                */

                float azimuth = db.azimuth * 0.01f;
                float diff = azimuth - prevAzimuth;
                if (diff < 0) diff += 360;
                azimuthAcc += diff;
                prevAzimuth = azimuth;
                if (azimuthAcc > 360) {
                    this->setFrameBuffer(frameBuffer);
                    return;
                };
                int newAzimuth = db.azimuth % 36000; //方位角
                if (newAzimuth < 0) newAzimuth += 36000;
                db.azimuth = newAzimuth;

                frameBuffer->push_back(db);
            }
        }
    }

    inline long long round(long long i, long long size) {
        if (i < 0) {
            return size + (i % size);
        } else {
            return i % size;
        }
    }

    template<typename Model>
    static long long startIndex(const std::vector<typename Model::BlockType> &blocks) { //起始索引
        long long size = blocks.size();
        for (long long i = 0; i < size; i++) {
            if (blocks[i].azimuth < blocks[round(i - 1, size)].azimuth)
                return i;
        }
        throw std::logic_error("unreachable");
    }

    template<typename Model, typename Spatial>
    typename Lidar<Model, Lidar16<Model, Spatial>, Spatial>::PointCloudType
    Lidar16<Model, Spatial>::frameToPointCloud(std::chrono::steady_clock::time_point &timestamp, Spatial &spatialInfo) {  //frameToPointCloud
        auto frameBuffer = this->getFrameBuffer(timestamp, spatialInfo);
        auto t1 = std::chrono::steady_clock::now();
        auto retPtr = std::make_shared<std::vector<std::vector<Eigen::Vector3f>>>();
        retPtr->resize(16);
        auto &ret = *retPtr;
        auto size = frameBuffer->size();
        auto offset = startIndex<Model>(*frameBuffer);
        for (std::size_t i = 0; i < size; i++) {
            const auto &block = (*frameBuffer)[round(i + offset, size)];
            const auto &nextBlock = (*frameBuffer)[round(i + offset + 1, size)];
            for (auto j = 0; j < 16; j++) {
                auto index = Model::channelVerticalAngleIndices[j]; //通道垂直角度指标
                const auto &c = block.channels[0][index];
                if (!Model::isValidChannelData(c))
                    continue;
                auto va = Model::verticalAngles[index];
                LidarPoint lidarPoint(c.distance * Model::distanceUnit, block.azimuth * 0.01f, va);
                Eigen::Vector3f p = this->_transform * Eigen::Vector3f(lidarPoint.x(), lidarPoint.y(), lidarPoint.z());
                if (p.z() < -1.2 || p.z() > 0.2)
                    continue;
                ret[j].push_back(p);
            }
            float azimuth = (block.azimuth + nextBlock.azimuth) / 2.0f;
            for (auto j = 0; j < 16; j++) {
                auto index = Model::channelVerticalAngleIndices[j];
                const auto &c = block.channels[1][index];
                if (!Model::isValidChannelData(c))
                    continue;
                auto va = Model::verticalAngles[index];
                LidarPoint lidarPoint(c.distance * Model::distanceUnit, block.azimuth * 0.01f, va);
                Eigen::Vector3f p = this->_transform * Eigen::Vector3f(lidarPoint.x(), lidarPoint.y(), lidarPoint.z());
                if (p.z() < -1.2 || p.z() > 0.2)
                    continue;
                ret[j].push_back(p);
            }
        }

        auto t2 = std::chrono::steady_clock::now();  //稳定时钟
        std::chrono::duration<float> duration = t2 - t1;
        // std::cout << " 16frame d: " << duration.count() << std::endl;
        return retPtr;
    }

    template<typename Model, typename Spatial>
    class Lidar32 : public Lidar<Model, Lidar32<Model, Spatial>, Spatial> {
    public:
        explicit Lidar32(unsigned short port, const Eigen::Affine3f & transform)
                : Lidar<Model, Lidar32<Model, Spatial>, Spatial>(port, transform) {

        }

    public:
        void readFrame();
        typename Lidar<Model, Lidar32<Model, Spatial>, Spatial>::PointCloudType frameToPointCloud(std::chrono::steady_clock::time_point &timestamp, Spatial &spatialInfo);


    };

    template<typename Model, typename Spatial>
    void Lidar32<Model, Spatial>::readFrame() {
        auto frameBuffer = std::make_shared<std::vector<typename Model::BlockType>>();
        float azimuthAcc = 0;
        float prevAzimuth;
        bool first = true;
        for (;;) {
            typename Model::PacketType packet;
            boost::system::error_code ec;
            ip::udp::endpoint remote;
            size_t n = this->_sock.receive_from(buffer(&packet, sizeof(packet)), remote, 0, ec);
            if (ec) {
                std::cerr << "error: " << ec << std::endl;
                continue;
            }
            if (first) {
                first = false;
                prevAzimuth = packet.dataBlocks[0].azimuth * 0.01f;
            }
            for (auto &db : packet.dataBlocks) {
                if (db.flag != Model::blockFlag) {
                    std::cerr << "bad packet" << std::endl;
                    break;
                }
                /*
                for (const auto & c : db.channels[0]) {
                    std::cout << "distance: " << c.distance << std::endl;
                    std::cout << "reflectivity: " << (int)c.reflectivity << std::endl;

                }
                */
                float azimuth = db.azimuth * 0.01f;
                float diff = azimuth - prevAzimuth;
                if (diff < 0) diff += 360;
                azimuthAcc += diff;
                prevAzimuth = azimuth;
                if (azimuthAcc > 360) {
                    this->setFrameBuffer(frameBuffer);
                    return;
                };
                int newAzimuth = db.azimuth % 36000;
                if (newAzimuth < 0) newAzimuth += 36000;
                db.azimuth = newAzimuth;

                frameBuffer->push_back(db);
            }
        }

    }

    template<typename Model, typename Spatial>
    typename Lidar<Model, Lidar32<Model, Spatial>, Spatial>::PointCloudType
    Lidar32<Model, Spatial>::frameToPointCloud(std::chrono::steady_clock::time_point &timestamp, Spatial &spatialInfo) {
        auto frameBuffer = this->getFrameBuffer(timestamp, spatialInfo);
        auto t1 = std::chrono::steady_clock::now();
        auto retPtr = std::make_shared<std::vector<std::vector<Eigen::Vector3f>>>();
        retPtr->resize(32);
        auto &ret = *retPtr;
        auto size = frameBuffer->size();
        auto offset = startIndex<Model>(*frameBuffer);
        for (std::size_t i = 0; i < size; i++) {
            const auto &block = (*frameBuffer)[round(i + offset, size)];
            const auto &nextBlock = (*frameBuffer)[round(i + offset + 1, size)];
            for (auto j = 0; j < 32; j++) {
                auto index = Model::channelVerticalAngleIndices[j];
                const auto &c = block.channels[index];
                if (!Model::isValidChannelData(c))
                    continue;
                float distance = c.distance * Model::distanceUnit;
                if(distance > 40 || distance < 2)
                    continue;
                auto va = Model::verticalAngles[index];
                if(va[0] > 0)
                    continue;
                LidarPoint lidarPoint(distance, block.azimuth * 0.01f + va[1], va[0]);
                Eigen::Vector3f p = this->_transform * Eigen::Vector3f(lidarPoint.x(), lidarPoint.y(), lidarPoint.z());
                if (p.z() < -1.2 || p.z() > 0.2)
                    continue;
                ret[j].push_back(p);
            }
        }

        auto t2 = std::chrono::steady_clock::now();
        std::chrono::duration<float> duration = t2 - t1;
        std::cout << " 32frame d: " << duration.count() << std::endl;
        return retPtr;
    }

    /*
    template<typename Lidar>
    class ObjectTracking {
    public:
        class Object {
            friend ObjectTracking<Lidar>;
        private:
            float _score{9999.9f};
        private:
            Eigen::Vector3d _position;
            Eigen::Vector3f _velocity;
            Eigen::Vector3f _AABBSize;
            int _lifeTime{0};
        public:
            Object(const Eigen::Vector3d &position, const Eigen::Vector3f &velocity, const Eigen::Vector3f &AABBSize)
                    : _position(position), _velocity(velocity), _AABBSize(AABBSize) {}

            const Eigen::Vector3d &position() const {
                return _position;
            };

            const Eigen::Vector3f &velocity() const {
                return _velocity;
            };

            void velocity(const Eigen::Vector3f &value) {
                _velocity = value;
            };

            const Eigen::Vector3f &AABBSize() const {
                return _AABBSize;
            }

            int lifeTime() const {
                return _lifeTime;
            }

            void lifeTime(int value) {
                _lifeTime = value;
            }


        };

        using ObjectsPtr = std::shared_ptr<std::vector<Object>>;

    private:
        std::thread _trackingThread;
        std::mutex _mutex;
        ObjectsPtr _objects{std::make_shared<std::vector<Object>>()};
        std::atomic_bool _exit{false};
        Lidar &_lidar;

        ObjectsPtr getObjects() {
            std::lock_guard<std::mutex> lock(this->_mutex);
            return _objects;

        }

        void setObjects(ObjectsPtr newObjects) {
            std::lock_guard<std::mutex> lock(this->_mutex);
            _objects = newObjects;

        }

        void extract_AABB(const pcl::PointCloud<pcl::PointXYZ> &pointCloud, pcl::PointXYZ &min, pcl::PointXYZ &max,
                          pcl::PointXYZ &center) {
            min = max = pointCloud.points.front();
            center.x = center.y = center.z = 0;
            for (const pcl::PointXYZ &p: pointCloud.points) {
                min.x = std::min(min.x, p.x);
                min.y = std::min(min.y, p.y);
                min.z = std::min(min.z, p.z);

                max.x = std::max(max.x, p.x);
                max.y = std::max(max.y, p.y);
                max.z = std::max(max.z, p.z);

                center.x += p.x;
                center.y += p.y;
                center.z += p.z;
            }
            center.x /= pointCloud.points.size();
            center.y /= pointCloud.points.size();
            center.z /= pointCloud.points.size();
        }

        template<typename LidarFrame>
        ObjectsPtr extractObjects(LidarFrame &lidarFrame) {
            std::vector<pcl::PointIndices> cluster_indices;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudDownSamplePtr = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

            for (const auto &ch: *lidarFrame) {
                for (const sensor::LidarPoint &lpt: ch) {
                    if (lpt.distance() > 60)
                        continue;
                    if (lpt.z() < -1.2 || lpt.z() > 0.2 || lpt.distance() < 0.6)
                        continue;
                    cloudPtr->push_back(pcl::PointXYZ(lpt.x(), lpt.y(), 0));
                }
            }
            cloudPtr->width = cloudPtr->points.size();
            cloudPtr->height = 1;
            cloudPtr->is_dense = true;
            pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
            voxelGrid.setInputCloud(cloudPtr);
            voxelGrid.setLeafSize(0.2, 0.2, 0.2);
            voxelGrid.filter(*cloudDownSamplePtr);
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(1.0);
            ec.setMinClusterSize(3);
            ec.setMaxClusterSize(100000);
            ec.setInputCloud(cloudDownSamplePtr);
            ec.extract(cluster_indices);
            std::cout << "size: " << cluster_indices.size() << std::endl;

            ObjectsPtr ret = std::make_shared<std::vector<Object>>();

            for (const pcl::PointIndices &ci : cluster_indices) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr clusterPtr = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

                for (int i: ci.indices) {
                    clusterPtr->push_back(cloudDownSamplePtr->points[i]);
                }
                pcl::PointXYZ min, max, center;
                extract_AABB(*clusterPtr, min, max, center);
                if (max.x - min.x > 10 || max.y - min.y > 10)
                    continue;
                ret->push_back(
                        Object(Eigen::Vector3d(center.x, center.y, center.z), Eigen::Vector3f(0, 0, 0),
                               Eigen::Vector3f(max.x - min.x, max.y - min.y, 0)));
            }
            return ret;
        }

        bool match(std::vector<Object> &objs, const Object &prev, std::size_t &index) {
            bool matched = false;
            double min = (prev.position() - objs[0].position()).norm();
            index = 0;
            for (std::size_t i = 0; i < objs.size(); i++) {
                Object &obj = objs[i];
                double norm = (prev.position() - obj.position()).norm();
                if (norm < min) {
                    min = norm;
                    index = i;
                }
            }
            if (min < objs[index]._score) {
                matched = true;
                objs[index]._score = min;
            }
            return matched;
        }

        void
        calcVelocity(const std::vector<Object> &prevObjs, std::vector<Object> &objs, std::chrono::duration<float> d) {
            for (const Object &prev: prevObjs) {
                std::size_t matchedIndex = 0;
                if (!match(objs, prev, matchedIndex)) {
                    continue;
                }
                Object &matched = objs[matchedIndex];
                Eigen::Vector3d dist = matched.position() - prev.position();
                Eigen::Vector3f v = dist.cast<float>() / d.count();
                if (v.norm() > 30)
                    continue;

                //if(prev.lifeTime() == 1 && (v-prev.velocity()).norm()/d.count() > 5)
                //    continue;
                matched.velocity(v);
            }
        }


        void trackingLoop() {
            std::chrono::steady_clock::time_point prevTimestamp = std::chrono::steady_clock::now();
            typename Lidar::SpatialType affine;
            while (!_exit.load()) {
                std::this_thread::sleep_for(20ms);
                boost::progress_timer timer;
                std::chrono::steady_clock::time_point timestamp;
                auto lidarFrameOpt = _lidar.current(timestamp, affine);
                if (!lidarFrameOpt)
                    continue;
                if (timestamp == prevTimestamp)
                    continue;
                std::chrono::duration<float> d = timestamp - prevTimestamp;
                prevTimestamp = timestamp;
                ObjectsPtr objs = extractObjects(*lidarFrameOpt);
                if (objs->empty())
                    continue;
                calcVelocity(*getObjects(), *objs, d);
                setObjects(objs);
            }
        }

    public:
        explicit ObjectTracking(Lidar &lidar) : _lidar(lidar) {
            this->_trackingThread = std::thread(&ObjectTracking<Lidar>::trackingLoop, this);

        }

        ~ObjectTracking() {
            this->_exit = true;
            this->_trackingThread.join();
        }

        ObjectsPtr objects() {
            return getObjects();
        }
    };
    */
}
