#ifndef PACKET_DEFINITIONS_MKX_PD_HPP
#define PACKET_DEFINITIONS_MKX_PD_HPP

#include <apsrc_udp_rosbridge/packet_definitions/core_pd.hpp>

namespace UDP_PACKETS
{
class GPS_Msg { // share gps with MKx
public:
	uint64_t time_ms;
	float latitude;
	float longitude;
	float elevation;
	float speed;
	float yaw;

	std::vector<uint8_t> pack() {
		std::vector<uint8_t> buffer(28);
		std::memcpy(&buffer[0], &time_ms, 8);
		std::memcpy(&buffer[8], &latitude, 4);
		std::memcpy(&buffer[12], &longitude, 4);
		std::memcpy(&buffer[16], &elevation, 4);
		std::memcpy(&buffer[20], &speed, 4);
		std::memcpy(&buffer[24], &yaw, 4);
		return buffer;
	}

  int unpack(std::vector<uint8_t> &buffer, int i) {
    std::memcpy(&time_ms, &buffer[i+0], 8);
    std::memcpy(&latitude, &buffer[i+8], 4);
    std::memcpy(&longitude, &buffer[i+12], 4);
    std::memcpy(&elevation, &buffer[i+16], 4);
    std::memcpy(&speed, &buffer[i+20], 4);
    std::memcpy(&yaw, &buffer[i+24], 4);
    return i+28;
  }
};

} // namespace UDP_PACKETS
#endif // PACKET_DEFINITIONS_MKX_PD_HPP