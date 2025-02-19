#include <ros/ros.h>
#include <ros/package.h>

#include <apsrc_udp_rosbridge/common.hpp>
#include <apsrc_udp_rosbridge/packet_definitions/core_pd.hpp>
#include <apsrc_udp_rosbridge/packet_definitions/cv_pd.hpp>

#include <apsrc_msgs/LaneCentering.h>

using namespace apsrc_udp_rosbridge;

class cv_to_ros
{
private:
  // Nodehandles
  ros::NodeHandle nh_;

  // Internal state
  AS::Network::UDPServer udp_server_;
  std::thread udp_server_thread_;
  bool udp_server_running_ = false;
  std::vector<uint8_t> empty_udp_msg_;

  // Parameters
  std::string server_ip_;
  int server_port_;

  // Publishers
  ros::Publisher lc_pub_;
public:
  cv_to_ros(){
    nh_ = ros::NodeHandle();
    loadParams();
    if (startServer(udp_server_, server_ip_, server_port_))
    {
      // Publishers
      lc_pub_ = nh_.advertise<apsrc_msgs::LaneCentering>("/cv_core/offset_to_centerline", 10, true);

      // Start the UDP server
      udp_server_.registerReceiveHandler(
        std::bind(&cv_to_ros::serverResponseHandler, this, std::placeholders::_1));
      udp_server_running_ = true;
      udp_server_thread_ = std::thread(std::bind(&AS::Network::UDPServer::run, &udp_server_));
      ROS_INFO("Interface to CV-core established");
    }
    else
    {
      udp_server_running_ = false;
      ros::shutdown();
    }
  }

  ~cv_to_ros()
  {
    if (udp_server_running_)
    {
      udp_server_.stop();
      udp_server_thread_.join();
    }
  }

  void loadParams()
  {
    nh_.param<std::string>("server_ip", server_ip_, "127.0.0.1");
    nh_.param("server_port", server_port_, 1551);
    ROS_INFO("Parameters Loaded");
  }
  
  std::vector<uint8_t> serverResponseHandler(const std::vector<uint8_t>& msg)
  {
    std::vector<uint8_t> udp_msg = {};
    UDP_PACKETS::lane_centering_msg request_msg;
    if (!request_msg.unpack(msg)) // Check if packet is correct
    {
      ROS_WARN("Packet content doesn't match! dropping packet");
      return empty_udp_msg_;
    }

    apsrc_msgs::LaneCentering ros_msg = {};
    ros_msg.offset_to_centerline = request_msg.offset;
    lc_pub_.publish(ros_msg);

    return empty_udp_msg_;
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cv_to_ros");
  cv_to_ros cv_to_ros;
  ros::spin();
  return 0;
}