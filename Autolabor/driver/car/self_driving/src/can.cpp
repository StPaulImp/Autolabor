#include"../include/can.h"

	template<> void can::CanData::set<bool>(unsigned int start, unsigned int len, Endian endian, bool value) {   //函数名：CanData，括号里面的是参数列表   //set
		unsigned int byte = start >> 0x3;
		unsigned int bit = start & 0x7;
		if (value) {
			_data[byte] |= (1 << bit);
		}
		else {
			_data[byte] &= ~(1 << bit);
		}
	}
	
	template<> bool can::CanData::get<bool>(unsigned int start, unsigned int len, Endian endian) const {   //get
		unsigned int byte = start >> 0x3;
		unsigned int bit = start & 0x7;
		return _data[byte] & (1 << bit);
	}

