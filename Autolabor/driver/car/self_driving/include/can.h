#pragma once
#include<vector> //容器
#include<algorithm>
#include<climits> //定义符号常量
#include<stdint.h>//c库文件

#undef LITTLE_ENDIAN
#undef BIG_ENDIAN  //大端字节序

namespace can { //定义can的命名空间，以candata设置他的公私空间
	class CanData {   //class为名称空间，class为一次性的，而namespace是开放的，可以追加多个内容的
	private:
		std::vector<unsigned char> _data;  //内存连续的char数据类型容器
	public:
		enum Endian {
			LITTLE_ENDIAN, BIG_ENDIAN           //枚举：LITTLE_ENDIAN是inter格式，BIG_ENDIAN摩托罗拉格式
		};
	public:
		explicit CanData(unsigned int len) :
				_data(len, 0) {

		}
		CanData(unsigned char* data, unsigned int len) :
				_data(len) {
			std::copy(data, data + len, _data.begin());    //把一个序列（sequence）拷贝到一个容器（container）中去，通常用std::copy算法
		}
		CanData(const CanData & other) = default; //const表示值是固定不变的
		CanData &operator=(const CanData & other) = default;
	public:
		std::vector<unsigned char> & data() {
			return _data;
		}

		[[nodiscard]] const std::vector<unsigned char> & data() const {
			return _data;
		}
		//定义数据类型为整型和T的值
		template<typename T> void set(unsigned int start, unsigned int len, //定义类型
				Endian endian, T value) {
			static_assert(std::numeric_limits<T>::is_integer, "T must be integer type"); //integer整数
			unsigned int byte = start >> 0x3;
			unsigned int bit = start & 0x7;

			for (unsigned int i = 0; i < len; i++) {
				T mask = 0x01 << i;
				if (value & mask) {
					_data[byte] |= (1 << bit);
				} else {
					_data[byte] &= ~(1 << bit);
				}
				bit++;
				if (bit >= 8) {
					bit = 0;
					byte += (endian == BIG_ENDIAN ? -1 : 1);
				}
			}
		}

		template<typename T> T get(unsigned int start, unsigned int len,
				Endian endian) const {
			static_assert(std::numeric_limits<T>::is_integer, "T must be integer type");
			T ret = T(); //ret为返回
			unsigned int byte = start >> 0x3;
			unsigned int bit = start & 0x7;

			for (unsigned int i = 0; i < len; i++) {
				T mask = 0x1 << i;
				if (_data[byte] & (1 << bit)) {
					ret |= mask;
				} else {
					ret &= ~mask;
				}
				bit++;
				if (bit >= 8) {
					bit = 0;
					byte += (endian == BIG_ENDIAN ? -1 : 1);
				}
			}
			if (std::numeric_limits<T>::is_signed) {
				T sign = ret & (1 << (len - 1));
				for (unsigned int i = len; i < sizeof(T) * CHAR_BIT; i++) {
					sign <<= 1;
					ret |= sign;
				}
			}
			return ret;
		}
	};
	
	template<> void CanData::set<bool>(unsigned int start, unsigned int len,
			Endian endian, bool value);
	template<> bool CanData::get<bool>(unsigned int start, unsigned int len,
			Endian endian) const;

	class CanMessage {
	private:
		uint32_t _id;
		CanData _data;
	public:
		CanMessage(uint32_t id, CanData & data) :
				_id(id), _data(data) {
		}
	public:
		uint32_t id() {
			return _id;
		}
		CanData & data() {
			return _data;
		}
		const CanData & data() const {  
	
			return _data;
		}
	};
}
