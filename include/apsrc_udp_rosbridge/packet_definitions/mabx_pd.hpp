#ifndef PACKET_DEFINITIONS_MABX_PD_HPP
#define PACKET_DEFINITIONS_MABX_PD_HPP

#include <apsrc_udp_rosbridge/packet_definitions/core_pd.hpp>
namespace UDP_PACKETS
{
struct waypoint_t { 
	float x = 0;
	float y = 0;
	float z = 0;
	float yaw = 0;
	float velocity = 0;
};// 20 bytes

struct smoothing_ctrl_t {
  uint8_t beginning; //1:fix steps 2: fix number
  uint8_t ending; //1:fix steps 2: fix number
  float beginning_smoothing_extra;
  float ending__smoothing_extra;
}; // 10 bytes

struct velocityCMD_t {
  int32_t waypoint_id = 0;
  int32_t number_of_waypoints = 0;
  uint8_t action = 0; //modify(0) set(1)
  float magnitude = 0; // velocity in mps for unit=0
  uint8_t unit = 0; // m/s(0) km/h (1) mph (2)
  uint8_t smoothingEn = 0; // disable(0) enable(1) manual(2)
  struct smoothing_ctrl_t smoothingCtrl;
}; //15 or 25 bytes

struct positionCMD_t {
  int32_t waypoint_id = 0;
  int32_t number_of_waypoints = 0;
  uint8_t action = 0; //modify(0) add(1) remove(2)
  float direction; //lateral in meters/ + for right/ - for left
  uint8_t unit = 0; // m(0) cm(1) inch(2)
  uint8_t smoothingEn = 0; // disable(0) enable(1) manual(2)
  struct smoothing_ctrl_t smoothingCtrl;
}; // 15 or 25 bytes

struct velocityProfileCMD_t
{
  int32_t waypoint_id;
  uint8_t num_of_waypoints;
  float velocity_vector[200];
}; // 805 bytes

struct positionProfileCMD_t
{
  int32_t waypoint_id;
  uint8_t num_of_waypoints;
  float lat_shift_vector[200];
}; // 805 bytes

class replyMsg{// 19 bytes
public:
	uint8_t response_to_msg_id;
	uint8_t response_to_request_id;
	int32_t request_stamp[2];
	int32_t acknowledge_stamp[2];
	bool request_accomplished;

	int pack(std::vector<uint8_t> &buffer, int i) {
    buffer[i] = response_to_msg_id;
    buffer[i+1] = response_to_request_id;
    std::memcpy(&buffer[i+2], &request_stamp, 8);
    std::memcpy(&buffer[i+10], &acknowledge_stamp, 8);
    buffer[i+18] = request_accomplished;
    return i+19;
  }

  int unpack(std::vector<uint8_t> &buffer, int i) {
    response_to_msg_id = buffer[i];
    response_to_request_id = buffer[i+1];
    std::memcpy(&request_stamp, &buffer[i+2], 8);
    std::memcpy(&acknowledge_stamp, &buffer[i+10], 8);
    request_accomplished = buffer[i+18];
    return i+19;
  }

};

class waypointsArrayMsg {// (2000/9)=2009 bytes
public:
	int32_t closest_global_waypoint_id;
	uint8_t num_waypoints = 0;
	float path_curvature_score = 0;
	struct waypoint_t waypoints_array[100];

	int pack(std::vector<uint8_t> &buffer, int i) {
		std::memcpy(&buffer[i], &closest_global_waypoint_id, 4);
		buffer[i+4] = num_waypoints;
		std::memcpy(&buffer[i+5], &path_curvature_score, 4);
		std::memcpy(&buffer[i+9], waypoints_array, 2000);
		return i+2009;
	}

  int unpack(std::vector<uint8_t> &buffer, int i) {
    std::memcpy(&closest_global_waypoint_id, &buffer[i], 4);
    num_waypoints = buffer[i+4];
    std::memcpy(&path_curvature_score, &buffer[i+5], 4);
    std::memcpy(waypoints_array, &buffer[i+9], 2000);
    return i+2009;
  }
};

class leadDetectMsg { // 57 bytes
public:
	int8_t detected;
	double gap_lat; // Logan devel
	double gap_lng; // Logan devel
	double scale_x; // Logan devel
	double scale_y; // Logan devel
	double radar_lng; // apsrc_lead_vehicle_detection
	double lead_vel_mps_abs; // apsrc_lead_vehicle_detection
	double lead_vel_mps_rel; // apsrc_lead_vehicle_detection

	int pack(std::vector<uint8_t> &buffer, int i){
		buffer[i] = detected;
		std::memcpy(&buffer[i+1], &gap_lat, 8);
		std::memcpy(&buffer[i+9], &gap_lng, 8);
		std::memcpy(&buffer[i+17], &scale_x, 8);
		std::memcpy(&buffer[i+25], &scale_y, 8);
		std::memcpy(&buffer[i+33], &radar_lng, 8);
		std::memcpy(&buffer[i+41], &lead_vel_mps_abs, 8);
		std::memcpy(&buffer[i+49], &lead_vel_mps_abs, 8);
		return i+57;
	}

  int unpack(std::vector<uint8_t> &buffer, int i){
    detected = buffer[i];
    std::memcpy(&gap_lat, &buffer[i+1], 8);
    std::memcpy(&gap_lng, &buffer[i+9], 8);
    std::memcpy(&scale_x, &buffer[i+17], 8);
    std::memcpy(&scale_y, &buffer[i+25], 8);
    std::memcpy(&radar_lng, &buffer[i+33], 8);
    std::memcpy(&lead_vel_mps_abs, &buffer[i+41], 8);
    std::memcpy(&lead_vel_mps_abs, &buffer[i+49], 8);
    return i+57;
  }
};

class spatMsg { // 10 bytes
public:
  int32_t intersection_id = -1;
	int32_t stop_wp_id = -1;
  int32_t depart_wp_id = -1;
	float distance = 0;
	uint8_t phase = 0;
	float time_to_change = 0;
  float cycle_time[3] = {-1.0, -1.0, -1.0};


	int pack(std::vector<uint8_t> &buffer, int i){
    std::memcpy(&buffer[i], &intersection_id, 4);
    std::memcpy(&buffer[i+4], &stop_wp_id, 4);
    std::memcpy(&buffer[i+8], &depart_wp_id, 4);
    std::memcpy(&buffer[i+12], &distance, 4);
    buffer[i+16] = phase;
    std::memcpy(&buffer[i+17], &time_to_change, 4);
    std::memcpy(&buffer[i+21], &cycle_time, 12);
    return i+33;
	}

  int unpack(std::vector<uint8_t> &buffer, int i){
    std::memcpy(&intersection_id, &buffer[i], 4);
    std::memcpy(&stop_wp_id, &buffer[i+4], 4);
    std::memcpy(&depart_wp_id, &buffer[i+8], 4);
    std::memcpy(&distance, &buffer[i+12], 4);
    phase = buffer[i+16];
    std::memcpy(&time_to_change, &buffer[i+17], 4);
    std::memcpy(&cycle_time, &buffer[i+21], 12);
    return i+33;
  }
};

class Message_general { // published periodically 
public:
	header header_msg;
	replyMsg replies_msg;
	waypointsArrayMsg waypoints_array_msg;
	leadDetectMsg lead_msg;
	spatMsg spat_msg;
	uint32_t crc;
	

	std::vector<uint8_t> pack() {
		std::vector<uint8_t> buffer(4096);
		int cb = header_msg.pack(buffer);
		cb = replies_msg.pack(buffer, cb);
		cb = waypoints_array_msg.pack(buffer, cb);
		cb = lead_msg.pack(buffer, cb);
		cb = spat_msg.pack(buffer, cb);
		
		// Calculate CRC
		boost::crc_32_type msg_crc;
		msg_crc.process_bytes(&buffer[0], 4092);
		crc = msg_crc.checksum();
		std::memcpy(&buffer[4092], &crc, 4);
		return buffer;
	}
};

class velocityCMD {
public:
  struct velocityCMD_t cmd;

  bool unpack(const std::vector<uint8_t> &buffer, int idx) {
    std::memcpy(&cmd.waypoint_id, &buffer[idx], 4);
    std::memcpy(&cmd.number_of_waypoints, &buffer[idx+4], 4);
    std::memcpy(&cmd.action, &buffer[idx+8], 1);
    std::memcpy(&cmd.magnitude, &buffer[idx+9], 4);
    std::memcpy(&cmd.unit, &buffer[idx+13], 1);
    std::memcpy(&cmd.smoothingEn, &buffer[idx+14], 1);
    if (cmd.smoothingEn == 2){
      std::memcpy(&cmd.smoothingCtrl.beginning, &buffer[idx+15], 1);
      std::memcpy(&cmd.smoothingCtrl.ending, &buffer[idx+16], 4);
      std::memcpy(&cmd.smoothingCtrl.beginning_smoothing_extra, &buffer[idx+20], 1);
      std::memcpy(&cmd.smoothingCtrl.ending__smoothing_extra, &buffer[idx+21], 4);
      return idx+25;
    }
    return idx+15;
  }
};

class positionCMD {
public:
  struct positionCMD_t cmd;

  int unpack(const std::vector<uint8_t> &buffer, int idx){
    std::memcpy(&cmd.waypoint_id, &buffer[idx], 4);
    std::memcpy(&cmd.number_of_waypoints, &buffer[idx+4], 4);
    std::memcpy(&cmd.action, &buffer[idx+8], 1);
    switch (cmd.action) {
      case 0: std::memcpy(&cmd.direction, &buffer[idx+9], 4);
    }
    std::memcpy(&cmd.unit, &buffer[idx+13], 1);
    std::memcpy(&cmd.smoothingEn, &buffer[idx+14], 1);
    if (cmd.smoothingEn == 2){
      std::memcpy(&cmd.smoothingCtrl.beginning, &buffer[idx+15], 1);
      std::memcpy(&cmd.smoothingCtrl.ending, &buffer[idx+16], 4);
      std::memcpy(&cmd.smoothingCtrl.beginning_smoothing_extra, &buffer[idx+20], 1);
      std::memcpy(&cmd.smoothingCtrl.ending__smoothing_extra, &buffer[idx+21], 4);
      return idx+25;
    }
    return idx+15;
  }
};

class veocityVectorCMD {
public:
  struct velocityProfileCMD_t cmd;
  int unpack(const std::vector<uint8_t> &buffer, int idx){
    std::memcpy(&cmd.waypoint_id, &buffer[idx], 4);
    std::memcpy(&cmd.num_of_waypoints, &buffer[idx+4], 1);
    std::memcpy(&cmd.velocity_vector, &buffer[idx+5], 800);
    return idx+805;
  }
};

class positionVectorCMD {
public:
  struct positionProfileCMD_t cmd;
  int unpack(const std::vector<uint8_t> &buffer, int idx){
    std::memcpy(&cmd.waypoint_id, &buffer[idx], 4);
    std::memcpy(&cmd.num_of_waypoints, &buffer[idx+4], 1);
    std::memcpy(&cmd.lat_shift_vector, &buffer[idx+5], 800);
  return idx+805;
  }
};

class RequestMsgs { // 1024 bytes
public:
  header header_msg;
  velocityCMD velocityCmd;
  positionCMD positionCmd;
  positionVectorCMD positionVectorCmd;
  veocityVectorCMD velocityVectorCmd;
  uint32_t crc;

  bool unpack(const std::vector<uint8_t> &buffer) {
    int idx = header_msg.unpack(buffer);
    switch (header_msg.msg_type) {
      case 1: //Deprecated
        break;
      case 2:
        idx = velocityCmd.unpack(buffer, idx); //velocity CMD
        break;
      case 3:
        idx = positionCmd.unpack(buffer, idx); //position CMD
        break;
      case 4:
        idx = velocityVectorCmd.unpack(buffer, idx); //velocity vector CMD
        break;
      case 5:
        idx = positionVectorCmd.unpack(buffer, idx); //position vector CMD
        break;
			case 254: // Connection Test CMD
				break;
      case 255: // Reset CMD
        break;
    }
    std::memcpy(&crc, &buffer[1020], 4);
    boost::crc_32_type msg_crc;
    msg_crc.process_bytes(&buffer[0], 1020);
    if (crc == msg_crc.checksum()) {
      return true;
    } else {
      return false;
    }
  }
};

}// namespace UDP_PACKETS

#endif // PACKET_DEFINITIONS_MABX_PD_HPP