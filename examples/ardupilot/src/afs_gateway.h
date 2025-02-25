#include RADL_HEADER

#include "ros/ros.h"

#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/Mavlink.h>
#include <mavlink/v1.0/common/mavlink.h>
#include <mavros_msgs/GPSRAW.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointList.h>
#include <sensor_msgs/NavSatFix.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <iostream>
#include <string>

using namespace std;

class AFS_Gateway
{
public:
 	AFS_Gateway();
 	void step(const radl_in_t* i, const radl_in_flags_t* i_f, radl_out_t* o, radl_out_flags_t* o_f);
private:
 	ros::NodeHandle nh;

 	ros::Subscriber mavros_battery_subcriber;
 	sensor_msgs::BatteryState::ConstPtr battery_status_mailbox;
	void mavros_battery_state_callback(const sensor_msgs::BatteryState::ConstPtr& bs);

  ros::Subscriber mavlink_from_subcriber;
 	mavros_msgs::Mavlink::ConstPtr geofence_status_mailbox;
  void mavlink_fence_status_callback(const mavros_msgs::Mavlink::ConstPtr& fs);

  ros::Subscriber mavros_gpsraw_subcriber;
 	mavros_msgs::GPSRAW::ConstPtr gps_status_mailbox;
	void mavros_gps_status_callback(const mavros_msgs::GPSRAW::ConstPtr& gs);

  ros::ServiceClient flight_controls_mode;

  ros::Subscriber mavros_autopilotstate_subcriber;
  mavros_msgs::State::ConstPtr autopilotstate_status_mailbox;
  void mavros_autopilotstate_callback(const mavros_msgs::State::ConstPtr& aps);

  ros::Subscriber mavros_missionwaypoints_subcriber;
  mavros_msgs::WaypointList::ConstPtr missionwaypoints_status_mailbox;
  void mavros_missionwaypoints_callback(const mavros_msgs::WaypointList::ConstPtr& mws);

  ros::Subscriber mavros_globalposition_subcriber;
  sensor_msgs::NavSatFix::ConstPtr globalposition_status_mailbox;
	void mavros_globalposition_callback(const sensor_msgs::NavSatFix::ConstPtr& gps);

  ros::Subscriber mavros_diagnostics_subcriber;
  diagnostic_msgs::DiagnosticArray::ConstPtr diagnostics_status_mailbox;
  void mavros_diagnostics_callback(const diagnostic_msgs::DiagnosticArray::ConstPtr& das);
  ros::Time previous_diagnostics_status_time;
  int previous_diagnostics_heartbeat_value;
  int current_diagnostics_heartbeat_value;
  double elapsed_diagnostics_status_duration;
  double current_heartbeat_loss_duration;

  int previous_flight_controls_cmd_id;

};
