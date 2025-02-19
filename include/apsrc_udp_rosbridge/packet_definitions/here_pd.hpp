#ifndef PACKET_DEFINITIONS_HERE_PD_HPP
#define PACKET_DEFINITIONS_HERE_PD_HPP

#include <apsrc_udp_rosbridge/packet_definitions/core_pd.hpp>

namespace UDP_PACKETS
{

struct waypoint_here_t { 
	float lat;
	float lng;
	float elv;
	float speed = -1;
	float base_speed = -1;
	float action = 0;
}; // 24 bytes


class Here_Msg { // Andrew's devel -- needs work
public:
	UDP_PACKETS::header header;
	uint8_t route_id;
	uint8_t total_number_of_routes;
	int32_t number_of_waypoint;
	int32_t length;
	int32_t duration;
	int32_t base_duration;
	struct waypoint_here_t here_waypoints[2664];
	
	std::vector<uint8_t> pack() {
		std::vector<uint8_t> buffer(64000);
		int cb = header.pack(buffer);
		buffer[cb] = route_id;
		buffer[cb+1] = total_number_of_routes;
		std::memcpy(&buffer[cb+2], &number_of_waypoint, 4);
		std::memcpy(&buffer[cb+6], &length, 4);
		std::memcpy(&buffer[cb+10], &duration, 4);
		std::memcpy(&buffer[cb+14], &base_duration, 4);
		std::memcpy(&buffer[cb+18], &here_waypoints, 63936);
		return buffer;
	}

  int unpack(std::vector<uint8_t> &buffer) {
    int cb = header.unpack(buffer);
    route_id = buffer[cb];
    total_number_of_routes = buffer[cb+1];
    std::memcpy(&number_of_waypoint, &buffer[cb+2], 4);
    std::memcpy(&length, &buffer[cb+6], 4);
    std::memcpy(&duration, &buffer[cb+10], 4);
    std::memcpy(&base_duration, &buffer[cb+14], 4);
    std::memcpy(&here_waypoints, &buffer[cb+18], 63936);
    return cb + 18 + 63936;
  }    
};

}
#endif // PACKET_DEFINITIONS_HERE_PD_HPP