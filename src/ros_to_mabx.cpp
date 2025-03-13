#include <string>
#include <vector>
#include <ros/ros.h>
#include "apsrc_udp_rosbridge/common.hpp"
#include <apsrc_udp_rosbridge/packet_definitions/core_pd.hpp>

#include <autoware_msgs/Lane.h>
#include <std_msgs/Int32.h>
#include <gps_common/GPSFix.h>
#include <visualization_msgs/Marker.h>
#include <apsrc_msgs/LeadVehicle.h>
#include <apsrc_msgs/SPaTnMAP.h>
#include <tf/transform_datatypes.h>

#include "apsrc_udp_rosbridge/packet_definitions/mabx_pd.hpp"

using namespace apsrc_udp_rosbridge;

class ros_to_mabx
{
private:
  ros::NodeHandle nh_;
  std::mutex udp_mtx_, cwp_mtx_, wp_mtx_, gps_mtx_;

  ros::Timer timer_;
  ros::Subscriber base_waypoints_sub_;
  ros::Subscriber closest_waypoint_sub_;
  ros::Subscriber gps_sub_;
  // ros::Subscriber udp_report_sub_;
  // ros::Subscriber udp_request_sub_;
  ros::Subscriber backplane_marker_sub_;
  ros::Subscriber lead_vehicle_sub_;
  ros::Subscriber SPaTnMAP_sub_;

  UDP_PACKETS::Message_general udp_msg_;

  // Internal State
  AS::Network::UDPInterface udp_interface_;
  uint8_t msg_id_                     = 255;

  // Parameters
  std::string destination_ip_;
  int destination_port_;
  double frequency_;
  int path_eval_size_;

  // GPS
  gps_common::GPSFix gps_;

  // Current base_waypoints
  bool received_base_waypoints_   = false;
  autoware_msgs::Lane base_waypoints_;

  // Closest global waypoint id
  int32_t closest_waypoint_id_ = -1;

public:
  ros_to_mabx()
  {
    nh_ = ros::NodeHandle();
    loadParams();

    if (openConnection(udp_interface_, destination_ip_, destination_port_))
    {
      base_waypoints_sub_     = nh_.subscribe("base_waypoints", 1, &ros_to_mabx::baseWaypointCallback, this);
      closest_waypoint_sub_   = nh_.subscribe("closest_waypoint", 1, &ros_to_mabx::closestWaypointCallback, this);
      gps_sub_                = nh_.subscribe("/gps/gps", 10, &ros_to_mabx::gpsCallback, this);
      SPaTnMAP_sub_           = nh_.subscribe("/v2x/SPaTnMAP", 1, &ros_to_mabx::spatnmapCallback, this);
      // TODO: udp_report_sub_         = nh_.subscribe("apsrc_udp/received_commands_report", 1, &udpReceivedReportCallback, this); 
      backplane_marker_sub_   = nh_.subscribe("backplane_estimation/backplane_filtered_marker", 1, &ros_to_mabx::backPlaneMarkerCallback, this);
      lead_vehicle_sub_       = nh_.subscribe("/lead_vehicle/track", 1, &ros_to_mabx::leadVehicleCallback, this);
      timer_                  = nh_.createTimer(ros::Duration(1/frequency_), std::bind(&ros_to_mabx::publish, this));

      ROS_INFO("Interface to MABx enabled");
    }
  }
  ~ros_to_mabx()
  {
    if (udp_interface_.is_open()){
      udp_interface_.close();
    }
  }

  void loadParams()
  {
    nh_.param<std::string>("destination_ip", destination_ip_, "127.0.0.1");
    nh_.param("destination_port", destination_port_, 1552);
    nh_.param("frequency", frequency_, 10.0);
    nh_.param("path_eval_size", path_eval_size_, 20);
  }

  void baseWaypointCallback(const autoware_msgs::Lane::ConstPtr& msg)
  {
    std::unique_lock<std::mutex> wp_lock(wp_mtx_);
    base_waypoints_ = *msg;
    received_base_waypoints_ = true;
  }
  
  void closestWaypointCallback(const std_msgs::Int32::ConstPtr& msg)
  {
    std::unique_lock<std::mutex> cwp_lock(cwp_mtx_);
    closest_waypoint_id_ = msg->data;
  }

  void gpsCallback(const gps_common::GPSFix::ConstPtr& msg)
  {
    std::unique_lock<std::mutex> gps_lock(gps_mtx_);
    gps_ = *msg;
  }

  
  void spatnmapCallback(const apsrc_msgs::SPaTnMAP::ConstPtr& msg)
  {
    std::unique_lock<std::mutex> v2x_lock(udp_mtx_);
    udp_msg_.spat_msg.distance = msg->distance_to_stop;
    udp_msg_.spat_msg.phase = msg->phase;
    udp_msg_.spat_msg.time_to_change = msg->time_to_stop;
    udp_msg_.spat_msg.stop_wp_id = msg->stop_waypoint;

  }

  // TODO: void udpReceivedReportCallback(){}
  
  void backPlaneMarkerCallback(const visualization_msgs::Marker::ConstPtr& msg)
  {
    std::unique_lock<std::mutex> bp_lock(udp_mtx_);
    if (udp_msg_.lead_msg.detected % 2 == 0){
      udp_msg_.lead_msg.detected += 1;
    } 

    udp_msg_.lead_msg.gap_lat = msg->pose.position.y;
    udp_msg_.lead_msg.gap_lng = msg->pose.position.x - msg->scale.x/2;
    udp_msg_.lead_msg.scale_x = msg->scale.x;
    udp_msg_.lead_msg.scale_y = msg->scale.y;
  }

  void leadVehicleCallback(const apsrc_msgs::LeadVehicle::ConstPtr& msg)
  {
    std::unique_lock<std::mutex> ld_lock(udp_mtx_);
    if (msg->lead_detected){
      if (udp_msg_.lead_msg.detected < 2){
        udp_msg_.lead_msg.detected += 2;
      }
      udp_msg_.lead_msg.radar_lng = msg->range;
      udp_msg_.lead_msg.lead_vel_mps_abs = msg->speed_mps;
      udp_msg_.lead_msg.lead_vel_mps_rel = msg->relative_speed_mps;
    }
  }

  void pathShare()
  {
    if (received_base_waypoints_ and closest_waypoint_id_!=-1) {
      std::unique_lock<std::mutex> wp_share_lock(udp_mtx_);
      udp_msg_.waypoints_array_msg.closest_global_waypoint_id = closest_waypoint_id_;
      if (base_waypoints_.waypoints.size() - closest_waypoint_id_ < 100) {
        udp_msg_.waypoints_array_msg.num_waypoints = base_waypoints_.waypoints.size() - closest_waypoint_id_;
      } else {
        udp_msg_.waypoints_array_msg.num_waypoints = 100;
      }
      for (uint i = 0; i < udp_msg_.waypoints_array_msg.num_waypoints; i++) {
        uint wp_id = i + closest_waypoint_id_;
        udp_msg_.waypoints_array_msg.waypoints_array[i].x =
                base_waypoints_.waypoints[wp_id].pose.pose.position.x;
        udp_msg_.waypoints_array_msg.waypoints_array[i].y =
                base_waypoints_.waypoints[wp_id].pose.pose.position.y;
        udp_msg_.waypoints_array_msg.waypoints_array[i].z =
                base_waypoints_.waypoints[wp_id].pose.pose.position.z;
        udp_msg_.waypoints_array_msg.waypoints_array[i].yaw =
                tf::getYaw(base_waypoints_.waypoints[wp_id].pose.pose.orientation);
        udp_msg_.waypoints_array_msg.waypoints_array[i].velocity =
                base_waypoints_.waypoints[wp_id].twist.twist.linear.x;
      }
      udp_msg_.waypoints_array_msg.path_curvature_score = path_curvature_score(path_eval_size_, base_waypoints_, closest_waypoint_id_);
    }
  }

  void publish()
  {
    if (udp_interface_.is_open()){
      std::unique_lock<std::mutex> msg_lock(udp_mtx_);
      udp_msg_.header_msg.info[0] = 0;
      udp_msg_.header_msg.msg_id = ++msg_id_;
      udp_msg_.header_msg.time_stamp[0] = gps_.header.stamp.sec;
      udp_msg_.header_msg.time_stamp[1] = gps_.header.stamp.nsec;
      udp_msg_.crc = 0;
      pathShare();
      
      udp_interface_.write(udp_msg_.pack());
      udp_msg_ = {};
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_to_mabx");
  ros_to_mabx node;
  ros::spin();
  return 0;
}