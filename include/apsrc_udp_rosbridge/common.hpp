#ifndef APSRC_UDP_ROSBridge_NETWOLK_HPP
#define APSRC_UDP_ROSBridge_NETWOLK_HPP

#include "ros/ros.h"
#include <network_interface/udp_server.h>
#include <network_interface/network_interface.h>
#include <mutex>
#include <string>
#include <vector>
#include <thread>


#include <apsrc_udp_rosbridge/packet_definitions/core_pd.hpp>
#include <autoware_msgs/Lane.h>

namespace apsrc_udp_rosbridge
{
bool openConnection(AS::Network::UDPInterface &udp_interface, std::string destination_ip, int destination_port){
  AS::Network::ReturnStatuses status = udp_interface.open(destination_ip, destination_port);
  if (status != AS::Network::ReturnStatuses::OK)
  {
    ROS_ERROR("Could not open UDP interface: %d - %s", static_cast<int>(status), return_status_desc(status).c_str());
    return false;
  }
  else
  {
    ROS_INFO("UDP interface opened, sending to %s:%d", destination_ip.c_str(), destination_port);
    return true;
  }
}

bool startServer(AS::Network::UDPServer &udp_server, std::string server_ip, int server_port)
{
  AS::Network::ReturnStatuses status = udp_server.open(server_ip, server_port);

  if (status != AS::Network::ReturnStatuses::OK)
  {
    ROS_ERROR("Could not start UDP server: %d - %s", static_cast<int>(status), return_status_desc(status).c_str());
    return false;
  }
  else
  {
    ROS_INFO("UDP server started at %s (%s)",server_ip.c_str(), std::to_string(server_port).c_str());
    return true;
  }
}

float path_curvature_score(size_t num_of_wp, autoware_msgs::Lane base_waypoints_, int32_t closest_waypoint_id_)
{
  if (base_waypoints_.waypoints.size() < (closest_waypoint_id_ + num_of_wp)){
    return 0;
  }
  double dx = base_waypoints_.waypoints[closest_waypoint_id_].pose.pose.position.x - base_waypoints_.waypoints[closest_waypoint_id_+num_of_wp].pose.pose.position.x;
  double dy = base_waypoints_.waypoints[closest_waypoint_id_].pose.pose.position.y - base_waypoints_.waypoints[closest_waypoint_id_+num_of_wp].pose.pose.position.y;
  double dz = base_waypoints_.waypoints[closest_waypoint_id_].pose.pose.position.z - base_waypoints_.waypoints[closest_waypoint_id_+num_of_wp].pose.pose.position.z;

  return static_cast<float>(sqrt(dx*dx + dy*dy + dz*dz)/num_of_wp);
}

std::string toLowerCase(const std::string& str) {
  std::string lowerStr = str;
  std::transform(lowerStr.begin(), lowerStr.end(), lowerStr.begin(),
                  [](unsigned char c) { return std::tolower(c); });
  return lowerStr;
}

}
#endif //APSRC_UDP_ROSBridge_NETWOLK_HPP