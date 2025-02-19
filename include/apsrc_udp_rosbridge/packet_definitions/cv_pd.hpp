#ifndef PACKET_DEFINITIONS_CV_PD_HPP
#define PACKET_DEFINITIONS_CV_PD_HPP

#include <apsrc_udp_rosbridge/packet_definitions/core_pd.hpp>

namespace UDP_PACKETS
{
class lane_centering_msg
{
public:
  float offset;

  std::vector<uint8_t> pack()
  {
    std::vector<uint8_t> buffer(4);
    std::memcpy(&buffer[0], &offset, 4);
    return buffer;
  }

  bool unpack(const std::vector<uint8_t>& buffer)
  {
    if (buffer.size() != 4)
    {
      return false;
    }
    std::memcpy(&offset, &buffer[0], 4);
    return true;
  }
};

class traffic_sign_msg
{
public:
  uint8_t sign_type;
  float distance;

  std::vector<uint8_t> pack()
  {
    std::vector<uint8_t> buffer(5);
    buffer[0] = sign_type;
    std::memcpy(&buffer[1], &distance, 4);
    return buffer;
  }

  bool unpack(const std::vector<uint8_t>& buffer)
  {
    if (buffer.size() != 5)
    {
      return false;
    }
    sign_type = buffer[0];
    std::memcpy(&distance, &buffer[1], 4);
    return true;
  }
};
} // namespace UDP_PACKETS
#endif // PACKET_DEFINITIONS_CV_PD_HPP