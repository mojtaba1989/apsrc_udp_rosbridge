#include <ros/ros.h>
#include <ros/package.h>

#include <apsrc_udp_rosbridge/common.hpp>
#include <apsrc_udp_rosbridge/packet_definitions/core_pd.hpp>
#include <apsrc_udp_rosbridge/packet_definitions/mabx_pd.hpp>

#include <apsrc_msgs/VelocityCommand.h>
#include <apsrc_msgs/VelocityArrayCommand.h>
#include <apsrc_msgs/PositionCommand.h>
#include <apsrc_msgs/PositionArrayCommand.h>
#include <apsrc_msgs/DriverInputCommand.h>

using namespace apsrc_udp_rosbridge;

class ros_to_mabx
{
private:
  // Nodehandles
  ros::NodeHandle nh_;
  
  // Internal state
  AS::Network::UDPServer udp_server_;
  std::thread udp_server_thread_;
  bool udp_server_running_ = false;
  std::vector<uint8_t> empty_udp_msg_;
  ros::Time last_msg_time_ = ros::Time::now();

  // Parameters
  std::string server_ip_;
  int server_port_;
  double msg_interval_ = 0.5;

  // Publishers
  ros::Publisher vel_pub_, vel_arr_pub_, pos_pub_, pos_arr_pub_, reset_pub_;

public:
  ros_to_mabx()
  {
    nh_ = ros::NodeHandle();
    loadParams();
    if (startServer(udp_server_, server_ip_, server_port_))
    {
      // Publishers
      vel_pub_      = nh_.advertise<apsrc_msgs::VelocityCommand>("/mabx_commands/velocity_cmd", 10, true);
      vel_arr_pub_  = nh_.advertise<apsrc_msgs::VelocityArrayCommand>("/mabx_commands/velocity_array_cmd", 10, true);
      pos_pub_      = nh_.advertise<apsrc_msgs::PositionCommand>("/mabx_commands/position_cmd", 10, true);
      pos_arr_pub_  = nh_.advertise<apsrc_msgs::PositionArrayCommand>("/mabx_commands/position_array_cmd", 10, true);
      reset_pub_    = nh_.advertise<apsrc_msgs::DriverInputCommand>("/mabx_commands/reset_cmd", 10, true);

      // Start the UDP server
      udp_server_.registerReceiveHandler(
        std::bind(&ros_to_mabx::serverResponseHandler, this, std::placeholders::_1));
      udp_server_running_ = true;
      udp_server_thread_ = std::thread(std::bind(&AS::Network::UDPServer::run, &udp_server_));
      ROS_INFO("Interface to MABX established");
    }
    else
    {
      udp_server_running_ = false;
      ros::requestShutdown();
    }
  }

  ~ros_to_mabx()
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
    nh_.param("msg_interval", msg_interval_, 0.5);
    const ros::Duration OUTDATED_DATA_TIMEOUT(msg_interval_);
    ROS_INFO("Parameters Loaded");
  }

  std::vector<uint8_t> serverResponseHandler(const std::vector<uint8_t>& msg)
  {
    std::vector<uint8_t> udp_msg = {};
    UDP_PACKETS::RequestMsgs request_msg;
    if (!request_msg.unpack(msg)) // Check if checksum matches
    {
      ROS_WARN("Checksum doesn't match! dropping packet");
      return empty_udp_msg_;
    }

    if (request_msg.header_msg.msg_type == 0) // Ignore msg_type == 0
    {
      return empty_udp_msg_;
    }

    if ((ros::Time::now() - last_msg_time_).toSec() <= msg_interval_)
    {
      ROS_WARN("Possibility of redundant packet! dropping packet");
      return empty_udp_msg_;
    }
    
    switch (request_msg.header_msg.msg_type){
      case 1: //Deprecated
        break;
      case 2:
        veocity_cmd_handler(request_msg);
        last_msg_time_ = ros::Time::now();
        break;
      case 3:
        position_cmd_handler(request_msg);
        last_msg_time_ = ros::Time::now();
        break;
      case 4:
        velocity_vector_cmd_handler(request_msg);
        last_msg_time_ = ros::Time::now();
        break;
      case 5:
        position_vector_cmd_handler(request_msg);
        last_msg_time_ = ros::Time::now();
        break;
      case 254: // Connection Test CMD // TODO
        udp_comm_test(request_msg);
        break;
      case 255: // Reset CMD
        waypoint_reset(request_msg);
        break;
    }

    return empty_udp_msg_;
  }

  void veocity_cmd_handler(UDP_PACKETS::RequestMsgs msg)
  {
    apsrc_msgs::VelocityCommand ros_msg = {};
    ros_msg.waypoint_id = msg.velocityCmd.cmd.waypoint_id;
    ros_msg.number_of_waypoints = msg.velocityCmd.cmd.number_of_waypoints;
    switch (msg.velocityCmd.cmd.action)
    {
    case 0:
      ros_msg.action = apsrc_msgs::VelocityCommand::ACTION_MODIFTY;
      break;
    case 1:
      ros_msg.action = apsrc_msgs::VelocityCommand::ACTION_SET;
      break;
    default:
      ros_msg.action = apsrc_msgs::VelocityCommand::ACTION_MODIFTY;
      break;
    }
    ros_msg.magnitude = msg.velocityCmd.cmd.magnitude;
    switch (msg.velocityCmd.cmd.unit)
    {
    case 0:
      ros_msg.unit = apsrc_msgs::VelocityCommand::UNIT_MPS;
      break;
    case 1:
      ros_msg.unit = apsrc_msgs::VelocityCommand::UNIT_KMPH;
      break;
    case 2:
      ros_msg.unit = apsrc_msgs::VelocityCommand::UNIT_MPH;
      break;
    default:
      ros_msg.unit = apsrc_msgs::VelocityCommand::UNIT_MPS;
      break;
    }
    switch (msg.velocityCmd.cmd.smoothingEn)
    {
    case 0:
      ros_msg.smoothingEn = apsrc_msgs::VelocityCommand::SMOOTHING_DISABLE;
      break;
    case 1:
      ros_msg.smoothingEn = apsrc_msgs::VelocityCommand::SMOOTHING_ENABLE;
      break;
    case 2:
      ros_msg.smoothingEn = apsrc_msgs::VelocityCommand::SMOOTHING_MANUAL;
      break;
    }
    vel_pub_.publish(ros_msg);
  }

  void position_cmd_handler(UDP_PACKETS::RequestMsgs msg)
  {
    apsrc_msgs::PositionCommand ros_msg = {};
    ros_msg.waypoint_id = msg.positionCmd.cmd.waypoint_id;
    ros_msg.number_of_waypoints = msg.positionCmd.cmd.number_of_waypoints;
    switch (msg.positionCmd.cmd.action)
    {
    case 0:
      ros_msg.action = apsrc_msgs::PositionCommand::ACTION_MODIFTY;
      break;
    case 1:
      ros_msg.action = apsrc_msgs::PositionCommand::ACTION_ADD;
      break;
    case 2:
      ros_msg.action = apsrc_msgs::PositionCommand::ACTION_REMOVE;
      break;
    default:
      ros_msg.action = apsrc_msgs::PositionCommand::ACTION_MODIFTY;
      break;
    }
    ros_msg.direction = msg.positionCmd.cmd.direction;
    switch (msg.positionCmd.cmd.unit)
    {
    case 0:
      ros_msg.unit = apsrc_msgs::PositionCommand::UNIT_M;
      break;
    case 1:
      ros_msg.unit = apsrc_msgs::PositionCommand::UNIT_CM;
      break;
    case 2:
      ros_msg.unit = apsrc_msgs::PositionCommand::UNIT_INCH;
      break;
    default:  
      ros_msg.unit = apsrc_msgs::PositionCommand::UNIT_M;
      break;
    }
    switch (msg.positionCmd.cmd.smoothingEn)
    {
    case 0:
      ros_msg.smoothingEn = apsrc_msgs::PositionCommand::SMOOTHING_DISABLE;
      break;
    case 1:
      ros_msg.smoothingEn = apsrc_msgs::PositionCommand::SMOOTHING_ENABLE;
      break;
    case 2:
      ros_msg.smoothingEn = apsrc_msgs::PositionCommand::SMOOTHING_MANUAL;
      break;
    }
    pos_pub_.publish(ros_msg);
  }

  void velocity_vector_cmd_handler(UDP_PACKETS::RequestMsgs msg)
  {
    apsrc_msgs::VelocityArrayCommand ros_msg = {};
    ros_msg.waypoint_id = msg.velocityVectorCmd.cmd.waypoint_id;
    ros_msg.num_of_waypoints = msg.velocityVectorCmd.cmd.num_of_waypoints;
    for (int i = 0; i < msg.velocityVectorCmd.cmd.num_of_waypoints; i++)
    {
      ros_msg.velocity_vector.push_back(msg.velocityVectorCmd.cmd.velocity_vector[i]);
    }
    vel_arr_pub_.publish(ros_msg);
  }

  void position_vector_cmd_handler(UDP_PACKETS::RequestMsgs msg)
  {
    apsrc_msgs::PositionArrayCommand ros_msg = {};
    ros_msg.waypoint_id = msg.positionVectorCmd.cmd.waypoint_id;
    ros_msg.num_of_waypoints = msg.positionVectorCmd.cmd.num_of_waypoints;
    for (int i = 0; i < msg.positionVectorCmd.cmd.num_of_waypoints; i++)
    {
      ros_msg.lat_shift_vector.push_back(msg.positionVectorCmd.cmd.lat_shift_vector[i]);
    }
    pos_arr_pub_.publish(ros_msg);
  }

  void udp_comm_test(UDP_PACKETS::RequestMsgs msg)
  {
    return;
  // TODO
  }

  void waypoint_reset(UDP_PACKETS::RequestMsgs msg)
  {
    apsrc_msgs::DriverInputCommand ros_msg = {};
    ros_msg.value = apsrc_msgs::DriverInputCommand::RESET_WAYPOINTS;
    reset_pub_.publish(ros_msg);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_to_mabx");
  ros_to_mabx node;
  ros::spin();
  return 0;
}