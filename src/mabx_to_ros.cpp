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

#include <apsrc_msgs/VehicleIntelInfo.h>
#include <apsrc_msgs/ecoMode.h>
#include <apsrc_msgs/MassLearn.h>
#include <apsrc_msgs/RoadLoadLearn.h>
#include <apsrc_msgs/ReducedOrderModel.h>
#include <apsrc_msgs/ecoDrive.h>
#include <apsrc_msgs/ecoCruise.h>


using namespace apsrc_udp_rosbridge;

class mabx_to_ros
{
private:
  // Nodehandles
  ros::NodeHandle nh_, pnh_;
  
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
  ros::Publisher vel_pub_, vel_arr_pub_, pos_pub_, pos_arr_pub_, reset_pub_, intel_pub_;

public:
  mabx_to_ros()
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
      intel_pub_    = nh_.advertise<apsrc_msgs::VehicleIntelInfo>("/vehicel_intel", 10, true);

      // Start the UDP server
      udp_server_.registerReceiveHandler(
        std::bind(&mabx_to_ros::serverResponseHandler, this, std::placeholders::_1));
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

  ~mabx_to_ros()
  {
    if (udp_server_running_)
    {
      udp_server_.stop();
      udp_server_thread_.join();
    }
  }

  void loadParams()
  {
    pnh_.param<std::string>("server_ip", server_ip_, "127.0.0.1");
    pnh_.param("server_port", server_port_, 1551);
    pnh_.param("msg_interval", msg_interval_, 0.5);
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
    return;
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
    return;
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
    return;
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
    return;
  }

  void udp_comm_test(UDP_PACKETS::RequestMsgs msg)
  {
    if (msg.header_msg.info[0] == 1)
    {
    }
    return;
  // TODO
  }

  void waypoint_reset(UDP_PACKETS::RequestMsgs msg)
  {
    if (msg.header_msg.info[0] == 1)
    {
    }
    apsrc_msgs::DriverInputCommand ros_msg = {};
    ros_msg.value = apsrc_msgs::DriverInputCommand::RESET_WAYPOINTS;
    reset_pub_.publish(ros_msg);
    return;
  }

  void vehicle_intel_info(UDP_PACKETS::RequestMsgs msg)
  {
    apsrc_msgs::VehicleIntelInfo ros_msg = {};
    ros_msg.header.stamp = ros::Time::now();
    ros_msg.header.frame_id = "base_link";

    // ECO MODE
    ros_msg.ecoMode.ecoMode_enabled = msg.mabxinfo.mode_blending_flag == 1 ? true : false;
    switch (msg.mabxinfo.mode_in_use)
    {
    case 0:
      ros_msg.ecoMode.mode_in_use = apsrc_msgs::ecoMode::PRODUCTION;
      break;
    case 1:
      ros_msg.ecoMode.mode_in_use = apsrc_msgs::ecoMode::HYBRYD;
      break;
    case 2:
      ros_msg.ecoMode.mode_in_use = apsrc_msgs::ecoMode::EV;
      break;
    default:
      ros_msg.ecoMode.mode_in_use = apsrc_msgs::ecoMode::UNAVAILABLE;
      break;
    }
    ros_msg.ecoMode.ecoMode_saving = msg.mabxinfo.mode_blending_saving;

    // ECO ROUTE
    ros_msg.ecoRoute.ecoRoute_enabled = (msg.mabxinfo.eco_route_flag == 1) ? true : false;
    ros_msg.ecoRoute.optimum_route = msg.mabxinfo.opt_route;
    ros_msg.ecoRoute.default_route_energy = msg.mabxinfo.def_route_energy;
    ros_msg.ecoRoute.default_route_time = msg.mabxinfo.def_route_time;
    ros_msg.ecoRoute.saving_time = msg.mabxinfo.opt_route_time_save_min;
    ros_msg.ecoRoute.saving_energy = msg.mabxinfo.opt_route_energy_save_Kwh;
    ros_msg.ecoRoute.saving_energy_percent = msg.mabxinfo.opt_route_energy_save_percent;

    // Range
    ros_msg.range.range_is_available = msg.mabxinfo.range_estimation_flag == 1 ? true : false;
    ros_msg.range.required_energy = msg.mabxinfo.energy_to_complete_opt_route;
    ros_msg.range.range_if_required_energy_is_not_met = msg.mabxinfo.range_if_opt_route_not_doable;
    ros_msg.range.required_energy_is_met = msg.mabxinfo.opt_route_doable==1 ? true : false;

    // Mass Learn
    switch (msg.mabxinfo.mass_learn_status)
    {
    case 0:
      ros_msg.massLearn.mass_learn_status = apsrc_msgs::MassLearn::OFF;
      break;
    case 1:
      ros_msg.massLearn.mass_learn_status = apsrc_msgs::MassLearn::ACTIVE;
      break;
    case 2:
      ros_msg.massLearn.mass_learn_status = apsrc_msgs::MassLearn::LEARNING;
      break;
    case 3:
      ros_msg.massLearn.mass_learn_status = apsrc_msgs::MassLearn::LEARNED;
      break;
    default:
      ros_msg.massLearn.mass_learn_status = apsrc_msgs::MassLearn::UNAVAILABLE;
      break;
    }
    ros_msg.massLearn.mass = msg.mabxinfo.mass;
    ros_msg.massLearn.mass_change_from_base_percentage = msg.mabxinfo.change_vs_normal;

    // RRL
    switch (msg.mabxinfo.road_load_learn_status)
    {
    case 0:
      ros_msg.roadLoadLearn.road_load_learn_status = apsrc_msgs::RoadLoadLearn::OFF;
      break;
    case 1:
      ros_msg.roadLoadLearn.road_load_learn_status = apsrc_msgs::RoadLoadLearn::ACTIVE;
      break;
    case 2:
      ros_msg.roadLoadLearn.road_load_learn_status = apsrc_msgs::RoadLoadLearn::LEARNING;
      break;
    case 3:
      ros_msg.roadLoadLearn.road_load_learn_status = apsrc_msgs::RoadLoadLearn::LEARNED;
      break;
    default:
      ros_msg.roadLoadLearn.road_load_learn_status = apsrc_msgs::RoadLoadLearn::UNAVAILABLE;
      break;
    }
    ros_msg.roadLoadLearn.F0.coefficient = msg.mabxinfo.road_load_coefficients[0];
    ros_msg.roadLoadLearn.F0.EPA_offset_percentage = msg.mabxinfo.chnage_vs_epa[0];
    ros_msg.roadLoadLearn.F1.coefficient = msg.mabxinfo.road_load_coefficients[1];
    ros_msg.roadLoadLearn.F1.EPA_offset_percentage = msg.mabxinfo.chnage_vs_epa[1];
    ros_msg.roadLoadLearn.F2.coefficient = msg.mabxinfo.road_load_coefficients[2];
    ros_msg.roadLoadLearn.F2.EPA_offset_percentage = msg.mabxinfo.chnage_vs_epa[2];

    // ECO DRIVE
    switch (msg.mabxinfo.eco_drive_status)
    {
    case 0:
      ros_msg.ecoDrive.ecoDrive_status = apsrc_msgs::ecoDrive::OFF;
      break;
    case 1:
      ros_msg.ecoDrive.ecoDrive_status = apsrc_msgs::ecoDrive::INACTIVE;
      break;
    case 2:
      ros_msg.ecoDrive.ecoDrive_status = apsrc_msgs::ecoDrive::CRUISING;
      break;
    case 3:
      ros_msg.ecoDrive.ecoDrive_status = apsrc_msgs::ecoDrive::OPTIMIZING;
      break;
    case 4:
      ros_msg.ecoDrive.ecoDrive_status = apsrc_msgs::ecoDrive::STOPPING;
      break;
    case 5:
      ros_msg.ecoDrive.ecoDrive_status = apsrc_msgs::ecoDrive::DEPARTING;
      break;
    default:
      ros_msg.ecoDrive.ecoDrive_status = apsrc_msgs::ecoDrive::UNAVAILABLE;
      break;
    }
    ros_msg.ecoDrive.saving_per_signal_percentage = msg.mabxinfo.saving_per_signal;
    ros_msg.ecoDrive.time_offset_per_signal = msg.mabxinfo.time_def_per_signal;

    // ECO CRUISE
    switch (msg.mabxinfo.vehicle_follow_status)
    {
    case 0:
      ros_msg.ecoCruise.ecoCruise_status = apsrc_msgs::ecoCruise::OFF;
      break;
    case 1:
      ros_msg.ecoCruise.ecoCruise_status = apsrc_msgs::ecoCruise::INACTIVE;
      break;
    case 2:
      ros_msg.ecoCruise.ecoCruise_status = apsrc_msgs::ecoCruise::ACTIVE;
      break;
    default:
      ros_msg.ecoCruise.ecoCruise_status = apsrc_msgs::ecoCruise::UNAVAILABLE;
      break;
    }
    ros_msg.ecoCruise.ecoCruise_saving = msg.mabxinfo.vehicle_follow_saving;

    // Reduced Order Model
    switch (msg.mabxinfo.reduced_order_model_status)
    {
    case 0:
      ros_msg.reducedOrderModel.reduced_order_model_status = apsrc_msgs::ReducedOrderModel::OFF;
      break;
    case 1:
      ros_msg.reducedOrderModel.reduced_order_model_status = apsrc_msgs::ReducedOrderModel::INACTIVE;
      break;
    case 2:
      ros_msg.reducedOrderModel.reduced_order_model_status = apsrc_msgs::ReducedOrderModel::ACTIVE;
      break;
    default:
      ros_msg.reducedOrderModel.reduced_order_model_status = apsrc_msgs::ReducedOrderModel::UNAVAILABLE;
      break;
    }
    ros_msg.reducedOrderModel.reduced_order_model_accuracy = msg.mabxinfo.reduced_order_model_accuracy;
    
    intel_pub_.publish(ros_msg);
    return;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mabx_to_ros");
  mabx_to_ros node;
  ros::spin();
  return 0;
}