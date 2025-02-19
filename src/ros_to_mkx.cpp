#include <string>
#include <vector>
#include <ros/ros.h>
#include <mutex>
#include "apsrc_udp_rosbridge/common.hpp"
#include <apsrc_udp_rosbridge/packet_definitions/core_pd.hpp>


#include <gps_common/GPSFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <apsrc_udp_rosbridge/packet_definitions/mkx_pd.hpp>

using namespace apsrc_udp_rosbridge;
class ros_to_mkx
{

private:
  ros::NodeHandle nh_;
  ros::Subscriber gps_sub_, current_velocity_sub_, imu_sub_;
  
  AS::Network::UDPInterface udp_interface_;

  std::string destination_ip_;
  int destination_port_;

  std::mutex udp_mtx_;
  UDP_PACKETS::GPS_Msg msg_;
  
  ros::Timer timer_;
  double frequency_;

public:
  ros_to_mkx(){
    nh_ = ros::NodeHandle();
    loadParams();

    if (openConnection(udp_interface_, destination_ip_, destination_port_))
    {
      gps_sub_                = nh_.subscribe("/gps/gps", 10, &ros_to_mkx::gpsCallback, this);
      current_velocity_sub_   = nh_.subscribe("/current_velocity", 10, &ros_to_mkx::velocityCallback, this);
      imu_sub_                = nh_.subscribe("/current_pose", 10, &ros_to_mkx::imuCallback, this);
      timer_                  = nh_.createTimer(ros::Duration(1/frequency_), std::bind(&ros_to_mkx::publish, this));
      ROS_INFO("Interface to MKx enabled");
    }
    
  }

  ~ros_to_mkx(){
    if (udp_interface_.is_open()){
      udp_interface_.close();
    }
  }

  void loadParams()
  {
    nh_.param<std::string>("destination_ip", destination_ip_, "127.0.0.1");
    nh_.param("destination_port", destination_port_, 1552);
    nh_.param("frequency", frequency_, 10.0);
    ROS_INFO("Parameters Loaded");
    return;
  }

  void imuCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    std::unique_lock<std::mutex> imu_lock(udp_mtx_);
    msg_.yaw = M_PI/2 + tf::getYaw(msg->pose.orientation);
  }
 
  void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
  {
    std::unique_lock<std::mutex> vel_lock(udp_mtx_);
    msg_.speed = msg->twist.linear.x;
  }

  void gpsCallback(const gps_common::GPSFix::ConstPtr& msg)
  {
    std::unique_lock<std::mutex> gps_lock(udp_mtx_);
    msg_.latitude   = static_cast<float>(msg->latitude);
    msg_.longitude  = static_cast<float>(msg->longitude);
    msg_.elevation  = static_cast<float>(msg->altitude);
    msg_.time_ms    = static_cast<uint64_t>((msg->header.stamp.sec + msg->header.stamp.nsec)*1000);
  }

  void publish(){
    std::unique_lock<std::mutex> main_lock(udp_mtx_);
    udp_interface_.write(msg_.pack());
    msg_ = {};
  }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_to_mkx");
    ros_to_mkx ros_to_mkx;
    ros::spin();
    return 0;
}