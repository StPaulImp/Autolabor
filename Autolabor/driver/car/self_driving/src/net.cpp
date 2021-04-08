#include "../include/net.h"
#include <iostream>
#include <chrono>
using namespace boost::asio;
using namespace std::chrono_literals;

void net::UDPJSONSender::sendLoop() {                       //发送loop环形
	for (;;) {
		boost::system::error_code ec;
		std::string json;
		auto ov = value();
		if (ov) {
			auto v = *ov;
			json = v.serialize();
		}
		else {
			json = "null";
		}
		_sock.send_to(buffer(json), _remote, 0, ec);
		if (ec) {
			std::cerr << "socket send error: " << ec.message() << std::endl;
		}
		std::this_thread::sleep_for(100ms);
	}
}

void net::UDPJSONReceiver::receiveLoop() {            //接受loop环形
	for (;;) {
		try {
			receive();
		}
		catch (const std::exception & e) {
			std::cerr << "exception: " << e.what() << std::endl;
		}
	}
}

void net::UDPJSONReceiver::receive() {                   //接受
	boost::system::error_code ec;
	ip::udp::endpoint remote;
	char buff[4096];
	size_t n = this->_sock.receive_from(buffer(buff, sizeof(buff)), remote, 0, ec);
	if (ec) {
		std::cerr << "socket receive error: " << ec.message() << std::endl;
		return;
	}
	std::string json(buff, n);
	picojson::value v;
	std::string err = picojson::parse(v, json);
	if (!err.empty()) {
		std::cerr << "parse json error: " << err << std::endl;   //parse解析
		return;
	}
	if (!v.is<picojson::null>())
		value(v);
	else
		value(boost::none);
	std::cout << "value: " << v << std::endl;
}
