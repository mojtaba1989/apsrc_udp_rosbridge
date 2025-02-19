#include <string>
#include <vector>
#include <ros/ros.h>
#include "apsrc_udp_rosbridge/common.hpp"
#include <apsrc_msgs/Response.h>
#include <apsrc_udp_rosbridge/packet_definitions/core_pd.hpp>
#include <apsrc_udp_rosbridge/packet_definitions/here_pd.hpp>


using namespace apsrc_udp_rosbridge;
class here_to_mabx
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber here_app_sub_;
  AS::Network::UDPInterface udp_interface_;

  std::string destination_ip_;
  int destination_port_;
  
public:
  here_to_mabx(){
    nh_ = ros::NodeHandle();
    loadParams();

    if (openConnection(udp_interface_, destination_ip_, destination_port_))
    {
      here_app_sub_ = nh_.subscribe("here/route", 1, &here_to_mabx::hereCallback, this);
      ROS_INFO("Interface to HERE map enabled");
    }
    
  }
  ~here_to_mabx(){
    if (udp_interface_.is_open()){
      udp_interface_.close();
    }
  }

  void loadParams()
  {
    nh_.param<std::string>("destination_ip", destination_ip_, "127.0.0.1");
    nh_.param("destination_port", destination_port_, 1552);
    ROS_INFO("Parameters Loaded");
    return;
  }

  void hereCallback(const apsrc_msgs::Response::ConstPtr& msg)
  {
    UDP_PACKETS::Here_Msg udp_msg;
    uint8_t number_of_routes = msg->routes.size();
    for (int ridx = 0; ridx < number_of_routes; ridx++){
      ros::Duration(1.0).sleep();
      udp_msg = {};
      udp_msg.route_id = ridx + 1;
      udp_msg.total_number_of_routes = number_of_routes;
      udp_msg.number_of_waypoint = static_cast<uint32_t>(msg->routes[ridx].waypoints.size()/3);
      if (udp_msg.number_of_waypoint > 2664){
        ROS_WARN("Number of Waypoints exceeded max capacity! Extra points dropped ...");
        udp_msg.number_of_waypoint = static_cast<uint32_t>(2664);
      }
      udp_msg.length = static_cast<uint32_t>(msg->routes[ridx].distance);
      udp_msg.duration = static_cast<uint32_t>(msg->routes[ridx].duration);
      udp_msg.base_duration = static_cast<uint32_t>(msg->routes[ridx].baseDuration);
    
      size_t max_id = udp_msg.number_of_waypoint;
      for (size_t idx = 0; idx < max_id; idx++){
        udp_msg.here_waypoints[idx].lat = msg->routes[ridx].waypoints[idx * 3];
        udp_msg.here_waypoints[idx].lng = msg->routes[ridx].waypoints[idx * 3 + 1];
        udp_msg.here_waypoints[idx].elv = msg->routes[ridx].waypoints[idx * 3 + 2];
      }

      for (size_t span_id = 0; span_id < msg->routes[ridx].spans.spans.size(); span_id++){
        if (msg->routes[ridx].spans.spans[span_id].offset < 2665){
          udp_msg.here_waypoints[msg->routes[ridx].spans.spans[span_id].offset].speed = msg->routes[ridx].spans.spans[span_id].trafficSpeed; 
          udp_msg.here_waypoints[msg->routes[ridx].spans.spans[span_id].offset].base_speed = msg->routes[ridx].spans.spans[span_id].baseSpeed;
        }
      }

      for (size_t span_id = 1; span_id < max_id; span_id++){
        if (udp_msg.here_waypoints[span_id].speed == -1){
          udp_msg.here_waypoints[span_id].speed = udp_msg.here_waypoints[span_id -1].speed;
          udp_msg.here_waypoints[span_id].base_speed = udp_msg.here_waypoints[span_id -1].base_speed;
        }
      }

      for (size_t action_id = 0; action_id < msg->routes[ridx].actions.actions.size(); action_id++){
        if (msg->routes[ridx].actions.actions[action_id].offset < 2665){
          std::string lower_instruction = toLowerCase(msg->routes[ridx].actions.actions[action_id].instruction);
          if (lower_instruction.find("ramp") != std::string::npos){
            udp_msg.here_waypoints[msg->routes[ridx].actions.actions[action_id].offset].action = 2;
          } else  if (lower_instruction.find("turn left") != std::string::npos){
            udp_msg.here_waypoints[msg->routes[ridx].actions.actions[action_id].offset].action = 3;
          } else  if (lower_instruction.find("turn right") != std::string::npos){
            udp_msg.here_waypoints[msg->routes[ridx].actions.actions[action_id].offset].action = 4;
          } else  if (lower_instruction.find("exit") != std::string::npos){
            udp_msg.here_waypoints[msg->routes[ridx].actions.actions[action_id].offset].action = 5;
          } else  if (lower_instruction.find("roundabout") != std::string::npos){
            udp_msg.here_waypoints[msg->routes[ridx].actions.actions[action_id].offset].action = 6;
          } else  if (lower_instruction.find("u-turn") != std::string::npos){
            udp_msg.here_waypoints[msg->routes[ridx].actions.actions[action_id].offset].action = 7;
          } else  if (lower_instruction.find("continue") != std::string::npos){
            udp_msg.here_waypoints[msg->routes[ridx].actions.actions[action_id].offset].action = 8;
          } else  if (lower_instruction.find("keep") != std::string::npos){
            udp_msg.here_waypoints[msg->routes[ridx].actions.actions[action_id].offset].action = 8;
          } else {
            udp_msg.here_waypoints[msg->routes[ridx].actions.actions[action_id].offset].action = 1;
          }
        }
      }

      udp_interface_.write(udp_msg.pack());
      ROS_INFO("HERE map route (%d out of %d) has been shared", ridx + 1, number_of_routes);
    }
  }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "here_to_mabx");
    here_to_mabx here_to_mabx;
    ros::spin();
    return 0;
}

