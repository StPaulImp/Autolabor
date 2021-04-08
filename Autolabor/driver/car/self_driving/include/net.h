#pragma once
#include <boost/optional.hpp>
#include "picojson.h"
#include <boost/asio.hpp>
#include <boost/core/noncopyable.hpp>
#include <mutex>
#include <thread>
#include <string>
#include <stdexcept>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace net {
	using namespace boost::asio;
	class UDPJSONSender : private boost::noncopyable {
	private:
		boost::optional<picojson::value> _value{ boost::none };
		io_service _ios;
		ip::udp::endpoint _remote;
		ip::udp::socket _sock;
		std::thread _sendThread;
		std::mutex _mutex;
	private:
		void sendLoop();
	public:
		UDPJSONSender(const std::string & ip, unsigned short port) :  //sender
			_remote(ip::address::from_string(ip), port),
			_sock(_ios) {

			boost::system::error_code ec;
			_sock.open(ip::udp::v4(), ec);
			if (ec) {
				throw std::runtime_error(ec.message());
			}
			std::thread t(&UDPJSONSender::sendLoop, this);
			this->_sendThread = std::move(t);
		}
		boost::optional<picojson::value> value() {
			std::lock_guard<std::mutex> lock(_mutex);
			return _value;
		}
		void value(const boost::optional<picojson::value> & v) {
			std::lock_guard<std::mutex> lock(_mutex);
			_value = v;
		}
	};
	
	class UDPJSONReceiver : private boost::noncopyable {         //reciever
	private:
		boost::optional<picojson::value> _value{ boost::none };
		io_service _ios;
		ip::udp::endpoint _server;
		ip::udp::socket _sock;
		std::thread _sendThread;
		std::mutex _mutex;
	private:
		void receiveLoop();
		void receive();
	public:
		explicit UDPJSONReceiver(unsigned short port) :
			_server(ip::udp::v4(), port),
			_sock(_ios, _server) {

			std::thread t(&UDPJSONReceiver::receiveLoop, this);
			this->_sendThread = std::move(t);
		}

		boost::optional<picojson::value> value() {
			std::lock_guard<std::mutex> lock(_mutex);
			return _value;
		}

		void value(const boost::optional<picojson::value> & v) {
			std::lock_guard<std::mutex> lock(_mutex);
			_value = v;

		}
	};
    class CanRawEndpoint; //原始点 终点
    class CanRaw {
    public:
        typedef CanRawEndpoint endpoint;
        typedef boost::asio::basic_raw_socket<net::CanRaw> socket;
        int family() const {
            return PF_CAN;
        }
        int type() const {
            return SOCK_RAW;

        }
        int protocol() const {
            return CAN_RAW;

        }

    };
    class CanRawEndpoint {
    private:
        sockaddr_can _addr;
    public:
        CanRawEndpoint() = default;
        explicit CanRawEndpoint(const std::string & ifname) {
            ifreq ifr;
            _addr.can_family  = AF_CAN;
            unsigned ifindex = if_nametoindex(ifname.c_str());
            if(0 == ifindex) {
                std::string what = "interface name not exists: ";
                what += ifname;
                throw std::invalid_argument(what);
            }
            _addr.can_ifindex = ifindex;
        }
    public:
        typedef CanRaw protocol_type;
        protocol_type protocol() const {
            return CanRaw();
        }
        sockaddr * data() {
            return reinterpret_cast<sockaddr*>(&_addr);
        }
        const sockaddr * data() const {
            return reinterpret_cast<const sockaddr*>(&_addr);
        }
        std::size_t size() const {
            return sizeof(sockaddr_can);
        }
        void resize(std::size_t new_size) {
            if(new_size > sizeof(_addr)) {
                std::string what = "invalid new_size";
                throw std::invalid_argument(what);
            }

        }
        std::size_t capacity() const {
            return sizeof(_addr);
        }



    };
}
