#ifndef PACKET_DEFINITIONS_H
#define PACKET_DEFINITIONS_H

#include <cstdint>
#include <cstring>
#include <vector>
#include <boost/crc.hpp>

namespace UDP_PACKETS {

class header {//24 bytes
public:
	uint8_t msg_id;
  uint8_t msg_type;
	int32_t time_stamp[2];
  uint32_t data_size;
	int8_t info[10];

	int pack(std::vector<uint8_t> &buffer) {
    buffer[0] = msg_id;
    buffer[1] = msg_type;
    std::memcpy(&buffer[2], &time_stamp, 8);
    std::memcpy(&buffer[10], &data_size, 4);
    std::memcpy(&buffer[14], &info, 10);
    return 24;
  }

  int unpack(const std::vector<uint8_t> &buffer) {
    msg_id = buffer[0];
    msg_type = buffer[1];
    std::memcpy(&time_stamp, &buffer[2], 8);
    std::memcpy(&data_size, &buffer[10], 4);
    std::memcpy(&info, &buffer[14], 10);
    return 24;
  }
};

}  // namespace UDP_PACKETS
#endif  // PACKET_DEFINITIONS_H