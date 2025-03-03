#include RADL_HEADER

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "sensor_msgs/msg/battery_state.hpp"
#include "mavros_msgs/msg/mavlink.hpp"
#include "mavlink/v1.0/common/mavlink.h"
#include "mavros_msgs/msg/gpsraw.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/waypoint_list.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include <iostream>
#include <string>

using namespace std;

class AFS_Gateway
{
public:
 	AFS_Gateway();
 	void step(const radl_in_t* i, const radl_in_flags_t* i_f, radl_out_t* o, radl_out_flags_t* o_f);
private:
  std::shared_ptr<rclcpp::Node> node;

  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr mavros_battery_subscriber;
 	sensor_msgs::msg::BatteryState::SharedPtr battery_status_mailbox;
	void mavros_battery_state_callback(const sensor_msgs::msg::BatteryState::ConstSharedPtr bs);

  rclcpp::Subscription<mavros_msgs::msg::Mavlink>::SharedPtr mavlink_from_subscriber;
  mavros_msgs::msg::Mavlink::SharedPtr geofence_status_mailbox;
  void mavlink_fence_status_callback(const mavros_msgs::msg::Mavlink::ConstSharedPtr fs);

  rclcpp::Subscription<mavros_msgs::msg::GPSRAW>::SharedPtr mavros_gpsraw_subscriber;
  mavros_msgs::msg::GPSRAW::SharedPtr gps_status_mailbox;
  void mavros_gps_status_callback(const mavros_msgs::msg::GPSRAW::ConstSharedPtr gs);

  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr flight_controls_mode;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_autopilotstate_subscriber;
  mavros_msgs::msg::State::SharedPtr autopilotstate_status_mailbox;
  void mavros_autopilotstate_callback(const mavros_msgs::msg::State::ConstSharedPtr aps);

  rclcpp::Subscription<mavros_msgs::msg::WaypointList>::SharedPtr mavros_missionwaypoints_subscriber;
  mavros_msgs::msg::WaypointList::SharedPtr missionwaypoints_status_mailbox;
  void mavros_missionwaypoints_callback(const mavros_msgs::msg::WaypointList::ConstSharedPtr mws);

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr mavros_globalposition_subscriber;
  sensor_msgs::msg::NavSatFix::SharedPtr globalposition_status_mailbox;
  void mavros_globalposition_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr gps);

  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr mavros_diagnostics_subscriber;
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr diagnostics_status_mailbox;
  void mavros_diagnostics_callback(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr das);
  rclcpp::Time previous_diagnostics_status_time;
  int previous_diagnostics_heartbeat_value;
  int current_diagnostics_heartbeat_value;
  double elapsed_diagnostics_status_duration;
  double current_heartbeat_loss_duration;

  int previous_flight_controls_cmd_id;

};
